#!/usr/bin/env python3
"""Wideband sweep driven by scheduled quick tunes, calibrated in dBm.

Builds a sweep plan over one or more frequency ranges, captures a quick tune
for every stop once, and then replays those tunes as timestamped scheduled
retunes so the FPGA hops on the sample clock instead of the host asking for a
retune and waiting for it.

Why this is fast
----------------
bladerf_set_frequency() programs the synthesiser and blocks for milliseconds.
bladerf_get_quick_tune() does that work once and stores the result as an
AD9361 fastlock profile, copied into one of the Nios' 256 profile slots.
bladerf_schedule_retune() then hands the Nios a slot number and a sample
timestamp, and the hop happens in the FPGA with no host involvement at all.
The host meanwhile just reads at absolute timestamps it already knows.

Two profile pools matter and they are not the same size:

  * The Nios holds NUM_BBP_FASTLOCK_PROFILES (256) saved profiles. That is
    what caps a bank of the plan: a 257th quick tune wraps and overwrites the
    first. Plans longer than that are split into banks, each bank re-acquiring
    its tunes when it starts.
  * The RFIC holds NUM_RFFE_FASTLOCK_PROFILES (8) scratch slots, which is what
    a recall loads into. Retunes are pipelined several deep, so each scheduled
    retune is given a different scratch slot -- reusing one that a pending
    recall still needs would clobber it.

Before the profile counter was allowed to wrap, a sweep like this died after
256 stops with BLADERF_ERR_UNEXPECTED and could only be recovered by reopening
the device.

Absolute power
--------------
Every packet carries the RFIC gain index the AGC actually used, four times per
packet, so a hop can be read under AGC and still reported in dBm. The samples
are divided by their own per-chunk gain before the FFT, which is the part a
single gain figure per step cannot do. Load a gain calibration table
(--gain-cal) for the front-end offset to be right as well; without one the
output is still gain-corrected but sits on an uncalibrated reference.

Output is hackrf_sweep-compatible CSV on stdout:

    date, time, hz_low, hz_high, hz_bin_width, num_samples, dB, dB, ...

Bins are normalised so that summing their linear powers over a span gives the
power in that span, which is what makes a channel measurement just a sum. A CW
tone is therefore spread over the Hann window's three bins and its peak bin
reads about 1.8 dB below the tone's actual power -- sum the bins, do not read
the peak.

Examples:
    # what the plan looks like, no hardware needed
    ./rx_fast_sweep.py -f 70:6000 --plan-only

    # the real thing, 20 MSps / 20 MHz, AGC, calibrated
    ./rx_fast_sweep.py -f 70:6000 -s 20e6 -b 20e6 --gain-cal auto > sweep.csv

    # one pass over the ISM band with manual gain
    ./rx_fast_sweep.py -f 2400:2500 --agc off -g 40 --sweeps 1
"""

import argparse
import collections
import datetime
import math
import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gain_profile as gp                # noqa: E402
from bladerf import _bladerf             # noqa: E402

ffi = _bladerf.ffi

# Retunes scheduled ahead of the read pointer. Bounded by the RFIC scratch
# slots, since each one in flight needs its own.
PIPELINE_DEPTH = _bladerf.NUM_RFFE_FASTLOCK_PROFILES

Step = collections.namedtuple("Step", "freq bands")


# ---------------------------------------------------------------- sweep plan

def parse_ranges(specs):
    """['70:6000', '2400:2500'] -> [(70e6, 6000e6), (2400e6, 2500e6)]."""
    out = []
    for spec in specs:
        for piece in spec.split(","):
            piece = piece.strip()
            if not piece:
                continue
            if ":" not in piece:
                raise ValueError(f"range {piece!r} is not MIN:MAX in MHz")
            lo_s, hi_s = piece.split(":", 1)
            lo, hi = float(lo_s) * 1e6, float(hi_s) * 1e6
            if hi <= lo:
                raise ValueError(f"range {piece!r}: max must exceed min")
            out.append((lo, hi))
    if not out:
        raise ValueError("no frequency ranges given")
    return out


def build_plan(ranges, sample_rate, style, usable):
    """Tune frequencies plus the span of each that will be reported.

    linear
        Each tune reports the central `usable` fraction of its band, and the
        tunes are spaced by exactly that much so the kept parts tile. Fewest
        hops, but it reports straight through DC, where the AD9361's LO
        leakage puts a spike.

    interleaved
        Two tunes per sample_rate, offset by sample_rate/4, each reporting the
        two quarter-bands at +/- (1/8 .. 3/8) of the rate. Neither the DC bin
        nor the band edges are ever reported, and the four quarter-bands from a
        tune pair tile a full sample_rate with no seam. Twice the hops for a
        clean spectrum -- the default, and the reason the hop has to be cheap.
    """
    sr = float(sample_rate)
    steps = []

    for lo, hi in ranges:
        span = hi - lo

        if style == "interleaved":
            n = int(math.ceil(span / sr))
            for i in range(n):
                # Coverage of this iteration is [lo + i*sr, lo + (i+1)*sr).
                # A tune at f reports [f-3sr/8, f-sr/8) and [f+sr/8, f+3sr/8);
                # the partner at f+sr/4 reports the two gaps either side.
                base = lo + i * sr + 3.0 * sr / 8.0
                steps.append(Step(base, ((base - 3 * sr / 8, base - sr / 8),
                                         (base + sr / 8, base + 3 * sr / 8))))
                mate = base + sr / 4.0
                steps.append(Step(mate, ((mate - 3 * sr / 8, mate - sr / 8),
                                         (mate + sr / 8, mate + 3 * sr / 8))))
        else:
            # Spacing follows the kept width, not the sample rate, so the
            # reported bands butt up against each other whatever `usable` is.
            keep = usable * sr
            n = int(math.ceil(span / keep))
            for i in range(n):
                f = lo + (i + 0.5) * keep
                steps.append(Step(f, ((f - keep / 2.0, f + keep / 2.0),)))

    return steps


def plan_banks(steps, bank_size):
    return [steps[i:i + bank_size] for i in range(0, len(steps), bank_size)]


# ------------------------------------------------------------------ spectrum

def periodogram(y, window, norm):
    """Power per bin, fftshifted, normalised so the bins sum to mean(|y|^2)."""
    spec = np.fft.fft(y * window)
    return np.fft.fftshift(np.abs(spec) ** 2 / norm)


def band_slice(freq, sample_rate, n, lo, hi):
    """Bin range of an fftshifted spectrum covering [lo, hi)."""
    first = freq - sample_rate / 2.0
    width = sample_rate / n
    k0 = int(math.ceil((lo - first) / width))
    k1 = int(math.floor((hi - first) / width))
    return max(0, k0), min(n, k1)


# ---------------------------------------------------------------- the device

def open_device(args):
    dev = _bladerf.BladeRF(args.device)
    ch = _bladerf.CHANNEL_RX(args.channel)

    fpga = dev.get_fpga_version()
    if (fpga.major, fpga.minor) < (0, 17):
        raise RuntimeError(
            f"FPGA {fpga} does not tag RX packets with the RFIC gain; "
            f"v0.17.0 or later is required for calibrated output")

    dev.set_sample_rate(ch, int(args.sample_rate))
    dev.set_bandwidth(ch, int(args.bandwidth))

    if args.gain_cal:
        path = args.gain_cal
        if path == "auto":
            path = f"{dev.get_serial()}_rx_gain_cal.tbl"
        else:
            m = re.search(r"([0-9a-fA-F]{32})", os.path.basename(path))
            if m and m.group(1).lower() != dev.get_serial().lower():
                print(f"# WARNING: {os.path.basename(path)} was swept on "
                      f"{m.group(1)}, this device is {dev.get_serial()}",
                      file=sys.stderr)
        dev.set_gain_calibration(ch, path)

    # Loading a table does not change the gain mode, so set it afterwards
    # either way and the two settings stay independent.
    if args.agc:
        dev.set_gain_mode(ch, _bladerf.GainMode.Default)
    else:
        dev.set_gain_mode(ch, _bladerf.GainMode.Manual)
        dev.set_gain(ch, args.gain)

    if args.bias_tee:
        dev.set_bias_tee(ch, True)

    dev.sync_config(_bladerf.ChannelLayout.RX_X1,
                    _bladerf.Format.SC16_Q11_META,
                    args.num_buffers, args.buffer_size,
                    args.num_transfers, args.stream_timeout)
    dev.enable_module(ch, True)
    return dev, ch


def acquire_tunes(dev, ch, steps, verbose):
    """One QuickTune per step, in plan order.

    This is the expensive half of the whole tool: each call retunes for real
    and stores a fastlock profile. It is also the only reason the sweep loop
    can be cheap, so it happens once per bank and is then replayed.
    """
    t0 = time.monotonic()
    tunes = []
    for i, step in enumerate(steps):
        dev.set_frequency(ch, int(step.freq))
        tunes.append(dev.get_quick_tune(ch))
        if verbose and (i + 1) % 64 == 0:
            print(f"# quick tunes {i + 1}/{len(steps)}", file=sys.stderr)
    if verbose:
        dt = time.monotonic() - t0
        print(f"# acquired {len(tunes)} quick tunes in {dt:.2f} s "
              f"({1e3 * dt / max(1, len(tunes)):.2f} ms each)",
              file=sys.stderr)
    return tunes


class Pipeline:
    """Scheduled retunes running ahead of the reader.

    Holds the invariant that matters: every retune in flight owns a distinct
    RFIC scratch slot, and every entry queued here has a read timestamp that
    is already past its own settling time.
    """

    def __init__(self, dev, ch, steps, tunes, settle, dwell, start_delay):
        self.dev, self.ch = dev, ch
        self.steps, self.tunes = steps, tunes
        self.settle, self.dwell = settle, dwell
        self.start_delay = start_delay
        self.slots = min(PIPELINE_DEPTH, len(steps))
        self.queue = collections.deque()
        self.next_step = 0
        self.scratch = 0
        self.sched_ts = 0
        #: Sample time allotted to each stop: the settling the radio needs
        #: plus the dwell, and nothing more. See pace() for why nothing more.
        self.stride = settle + dwell

    def prime(self):
        """(Re)start the pipeline from the current sample counter."""
        self.dev.cancel_scheduled_retunes(self.ch)
        self.queue.clear()
        self.next_step = 0
        self.scratch = 0
        self.sched_ts = (int(self.dev.get_timestamp(_bladerf.Direction.RX))
                         + self.start_delay)
        for _ in range(self.slots):
            self._schedule()

    def _schedule(self):
        step = self.steps[self.next_step]
        qt = self.tunes[self.next_step]
        # The Nios profile carries the frequency; this only picks which RFIC
        # scratch slot the recall lands in, so it must differ from every other
        # recall still pending.
        qt.rffe_profile = self.scratch
        self.dev.schedule_retune(self.ch, self.sched_ts, 0, qt)

        self.queue.append((step, self.sched_ts + self.settle, self.next_step))
        self.scratch = (self.scratch + 1) % self.slots
        self.sched_ts += self.stride
        self.next_step = (self.next_step + 1) % len(self.steps)

    def pop(self):
        """The next entry to read: (step, read_timestamp, step_index)."""
        return self.queue.popleft()

    def advance(self):
        self._schedule()

    def lead(self):
        """Samples between the live edge of the stream and the schedule head.

        One USB round trip, so this is sampled every so often rather than per
        stop.
        """
        now = int(self.dev.get_timestamp(_bladerf.Direction.RX))
        return self.sched_ts - now

    def pace(self, lead, target):
        """Keep some headroom between the live edge and the schedule head.

        The lead is headroom: the distance from the sample the radio is
        capturing now to the head of the schedule. Reads consume it whenever
        the host costs more per stop than a stop is allotted, and once it is
        gone sync_rx() raises TimePastError and the pipeline has to be rebuilt.
        So when it has been eaten -- by stream start-up, or a stall -- put it
        back in one jump.

        **The stride is not a control knob here, and trying to use it as one
        made the sweep four times slower.** The idea was reasonable: if a stop
        is allotted less sample time than the host needs, give it more, and
        find the smallest sustainable spacing by nulling the observed drift.
        It fails because the host's cost per stop is not independent of the
        spacing. sync_rx() seeks to an absolute timestamp by discarding the
        buffered samples before it, so it is charged for the whole stride --
        measured on an xA4 at 20 Msps, with the stride pinned:

            stride  1.60  2.50  4.00  6.00  10.00 ms
            sync_rx 0.96  1.76  3.17  5.06   8.72 ms

        That is a slope of about 0.87. Feeding drift back into the stride is
        therefore a loop with gain 0.87 rather than 0, and it has a stable
        fixed point at roughly c / (1 - 0.87), some seven times wider than the
        floor: widening the stride by the drift makes the host slower by 87%
        of the widening, which reads as more drift. Measured over 600 stops it
        climbed from the 1.513 ms floor to 13.05 ms and stayed there, then
        unwound only at the probe's 2% per check -- about 2700 stops. 96
        stops/s, against 250 for the same hardware at the floor.

        There is no operating point up there worth reaching. A wider stride
        buys the host time it then spends skipping the extra samples, so the
        floor -- the settling the radio actually needs, plus the dwell -- is
        both the fastest spacing and the cheapest one. Widen it with
        --settle-ms if a retune needs longer.
        """
        if lead < target // 2:
            self.sched_ts += target - lead

    @property
    def last_index(self):
        return len(self.steps) - 1


# ------------------------------------------------------------------- sweeping

def run_bank(dev, ch, steps, tunes, args, gdb, msg_samples, emit, passes):
    """Sweep a bank until `passes` passes are done (None = until interrupted).

    Returns (stops, ambiguous_fraction, reprimes). The pipeline is primed and
    runs continuously, so a pass boundary costs nothing -- which is the whole
    point of scheduling retunes ahead.
    """
    n = args.fft_size
    window = np.hanning(n)
    norm = n * float(np.sum(window ** 2))
    dwell = n * args.average

    buf = bytearray(4 * dwell)
    meta = ffi.new("struct bladerf_metadata *")

    pipe = Pipeline(dev, ch, steps, tunes, args.settle, dwell,
                    args.start_delay)
    pipe.prime()

    stops = 0
    done = 0
    amb_samples = 0
    tot_samples = 0
    repriming = 0
    # The start delay doubles as the lead the pacer aims to hold: enough room
    # for the host to be late, not so much that reads sit blocked.
    lead_target = args.start_delay
    since_check = 0

    while True:
        step, read_ts, idx = pipe.pop()

        meta.flags = 0
        meta.timestamp = read_ts
        meta.status = 0
        # A successful timestamped read starts exactly at read_ts or raises
        # TimePastError -- sync_rx() seeks to the target and refuses to
        # substitute anything else. So the samples need no further validation.
        #
        # meta.status will have BLADERF_META_STATUS_OVERRUN set on essentially
        # every read, and that is correct rather than a fault: a sweep skips
        # the settling samples between stops, so consecutive reads are
        # deliberately discontinuous and the library says so.
        try:
            dev.sync_rx(buf, dwell, args.stream_timeout, meta)
        except _bladerf.TimePastError:
            # The reader fell behind its own schedule, so everything queued is
            # stale. Rebuild from the current sample counter and carry on.
            repriming += 1
            if args.verbose:
                print("# read timestamp already past, re-priming",
                      file=sys.stderr)
            pipe.prime()
            since_check = 0
            continue

        count = int(meta.actual_count)
        if count >= n:
            tags = dev.rx_gain_tags()
            if tags is None:
                raise RuntimeError(
                    "no RX gain tag: needs FPGA v0.17.0 or later and a "
                    "metadata RX format")

            # The gain index -> dB mapping is band dependent, so the cache has
            # to be told which frequency these samples were captured at before
            # gain_db_array() consults it.
            gdb.at(step.freq)

            v = np.frombuffer(buf, dtype=np.int16, count=2 * count)
            f32 = v.astype(np.float32)
            iq = (f32[0::2] + 1j * f32[1::2]) / gp.FULL_SCALE

            gain_db, _filled, ambiguous = gp.gain_db_array(tags, msg_samples,
                                                           count, gdb)
            # Per sample, not per stop: the AGC can move inside a single dwell,
            # and the four chunks per packet are what let that be undone.
            y = iq * (10.0 ** (-gain_db[:count] / 20.0))

            acc = np.zeros(n)
            segs = 0
            for k in range(0, count - n + 1, n):
                acc += periodogram(y[k:k + n], window, norm)
                segs += 1

            emit(step, acc / segs, count)

            stops += 1
            amb_samples += ambiguous
            tot_samples += count
        # else: a discontinuity truncated the read. Reporting a spectrum from
        # samples that are not contiguous would be worse than a gap.

        since_check += 1
        if since_check >= args.pace_every:
            pipe.pace(pipe.lead(), lead_target)
            since_check = 0

        pipe.advance()

        if idx == pipe.last_index:
            done += 1
            if passes is not None and done >= passes:
                return stops, amb_samples / max(1, tot_samples), repriming


def make_emitter(args, out):
    sr = float(args.sample_rate)
    n = args.fft_size
    bin_width = sr / n

    def emit(step, p_lin, num_samples):
        stamp = datetime.datetime.now()
        date = stamp.strftime("%Y-%m-%d")
        clock = stamp.strftime("%H:%M:%S.%f")
        for lo, hi in step.bands:
            k0, k1 = band_slice(step.freq, sr, n, lo, hi)
            if k1 <= k0:
                continue
            dbm = 10.0 * np.log10(p_lin[k0:k1] + 1e-300)
            row = [date, clock,
                   f"{int(round(step.freq - sr / 2 + k0 * bin_width))}",
                   f"{int(round(step.freq - sr / 2 + k1 * bin_width))}",
                   f"{bin_width:.2f}", str(num_samples)]
            row.extend(f"{x:.2f}" for x in dbm)
            out.write(", ".join(row) + "\n")
        if args.flush:
            out.flush()

    return emit


# ----------------------------------------------------------------------- main

def next_pow2(x):
    return 1 << max(0, int(math.ceil(math.log2(max(1, x)))))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ap.add_argument("-f", "--freq", action="append", required=True,
                    metavar="MIN:MAX",
                    help="frequency range in MHz, repeatable and "
                         "comma-separated (e.g. -f 70:6000)")
    ap.add_argument("-s", "--sample-rate", type=float, default=20e6)
    ap.add_argument("-b", "--bandwidth", type=float, default=20e6)
    ap.add_argument("-w", "--bin-width", type=float, default=100e3,
                    help="target FFT bin width in Hz (default 100k); the FFT "
                         "size is rounded up to a power of two")
    ap.add_argument("--fft-size", type=int, default=None,
                    help="override the FFT size implied by --bin-width")
    ap.add_argument("--style", choices=("interleaved", "linear"),
                    default="interleaved",
                    help="interleaved avoids DC and the filter skirts at the "
                         "cost of twice the hops (default)")
    ap.add_argument("--usable", type=float, default=0.75,
                    help="fraction of the band reported per tune, linear "
                         "style only (default 0.75)")

    ap.add_argument("--average", type=int, default=1, metavar="N",
                    help="average N periodograms per stop; the dwell grows "
                         "with it and the sweep slows (default 1)")
    ap.add_argument("--settle-ms", dest="settle_ms", type=float, default=1.5,
                    help="settling time allowed after each scheduled retune, "
                         "in ms (default 1.5)")
    ap.add_argument("--pace-every", type=int, default=32, metavar="STOPS",
                    help="how often to check the schedule lead and re-space "
                         "the stops; one USB round trip each (default 32)")
    ap.add_argument("--start-delay-ms", dest="start_delay_ms", type=float,
                    default=20.0,
                    help="how far ahead of the sample counter the first "
                         "retune is scheduled, and the headroom the pacer "
                         "keeps thereafter (default 20). This is the single "
                         "biggest knob on sweep rate, and it goes the "
                         "unintuitive way: more headroom is slower, because "
                         "restoring it injects dead schedule and because "
                         "sync_rx() is charged for seeking across whatever "
                         "it is given. Measured at 20 Msps on a 210-stop "
                         "plan, 1260 stops a run: 150 ms gives 164 stops/s, "
                         "40 gives 406, 20 gives 572 and 15 gives 646 "
                         "against a 661 stops/s plan. 20 is the default "
                         "rather than 15 because 15 cost a re-prime in two "
                         "runs out of three and 20 cost none; drop it if you "
                         "want the last 12% and can absorb an occasional "
                         "rebuild. Below about 6 ms it re-primes constantly")

    ap.add_argument("-g", "--gain", type=int, default=40,
                    help="manual gain in dB, used when --agc is off")
    ap.add_argument("--agc", choices=("on", "off"), default="on",
                    help="AGC is usable here because every packet reports the "
                         "gain it used (default on)")
    ap.add_argument("--gain-cal", metavar="PATH|auto", default=None,
                    help="load an RX gain calibration table so dBm is "
                         "absolute; 'auto' finds it by serial number")

    ap.add_argument("-d", "--device", default=None, help="device identifier")
    ap.add_argument("-c", "--channel", type=int, default=0)
    ap.add_argument("--bias-tee", action="store_true")

    ap.add_argument("--num-buffers", type=int, default=256)
    ap.add_argument("--buffer-size", type=int, default=8192)
    ap.add_argument("--num-transfers", type=int, default=32)
    ap.add_argument("--stream-timeout", type=int, default=3500)

    ap.add_argument("-n", "--sweeps", type=int, default=0,
                    help="stop after N passes over the plan (0 = forever)")
    ap.add_argument("-1", "--one-shot", action="store_true",
                    help="equivalent to --sweeps 1")
    ap.add_argument("--bank-size", type=int,
                    default=_bladerf.NUM_BBP_FASTLOCK_PROFILES,
                    help="stops per bank of quick tunes; cannot exceed the "
                         "Nios profile count (default 256)")
    ap.add_argument("-o", "--output", default=None,
                    help="write CSV here instead of stdout")
    ap.add_argument("--no-flush", dest="flush", action="store_false",
                    help="do not flush after every row")
    ap.add_argument("--plan-only", action="store_true",
                    help="print the sweep plan and exit, touching no hardware")
    ap.add_argument("-v", "--verbose", action="store_true")

    args = ap.parse_args(argv)

    if args.one_shot:
        args.sweeps = 1
    args.agc = (args.agc == "on")
    if args.fft_size is None:
        args.fft_size = next_pow2(args.sample_rate / args.bin_width)
    if args.average < 1:
        ap.error("--average must be at least 1")
    if args.bank_size < 1 or \
            args.bank_size > _bladerf.NUM_BBP_FASTLOCK_PROFILES:
        ap.error("--bank-size must be between 1 and "
                 f"{_bladerf.NUM_BBP_FASTLOCK_PROFILES}")

    args.settle = int(args.settle_ms * args.sample_rate / 1000.0)
    args.start_delay = int(args.start_delay_ms * args.sample_rate / 1000.0)
    return args


def describe(steps, banks, args, out):
    sr = args.sample_rate
    n = args.fft_size
    dwell = n * args.average
    per_stop = args.settle + dwell
    print(f"# {len(steps)} stops in {len(banks)} bank(s), {args.style} style",
          file=out)
    print(f"# {sr / 1e6:.3f} MSps, {args.bandwidth / 1e6:.3f} MHz bandwidth, "
          f"FFT {n} -> {sr / n / 1e3:.2f} kHz bins", file=out)
    print(f"# {args.settle} samples settling + {dwell} dwell = "
          f"{1e3 * per_stop / sr:.3f} ms per stop", file=out)
    print(f"# {sr / per_stop:.0f} stops/s -> "
          f"{sr / per_stop / max(1, len(steps)):.2f} sweeps/s", file=out)
    if len(banks) > 1:
        print(f"# NOTE: the plan exceeds the Nios' "
              f"{_bladerf.NUM_BBP_FASTLOCK_PROFILES} profile slots, so each "
              f"pass re-acquires quick tunes {len(banks)} times", file=out)


def main(argv=None):
    args = parse_args(argv)

    try:
        ranges = parse_ranges(args.freq)
    except ValueError as exc:
        print(f"{os.path.basename(sys.argv[0])}: {exc}", file=sys.stderr)
        return 2

    steps = build_plan(ranges, args.sample_rate, args.style, args.usable)
    banks = plan_banks(steps, args.bank_size)

    if args.plan_only:
        describe(steps, banks, args, sys.stdout)
        for i, s in enumerate(steps):
            spans = " ".join(f"[{lo / 1e6:.3f},{hi / 1e6:.3f})"
                             for lo, hi in s.bands)
            print(f"{i:5d}  {s.freq / 1e6:11.4f} MHz  {spans}")
        return 0

    describe(steps, banks, args, sys.stderr)

    out = open(args.output, "w") if args.output else sys.stdout
    dev, ch = open_device(args)
    try:
        msg_samples = gp.message_samples(dev)
        gdb = gp.GainDb(dev, ch)
        emit_row = make_emitter(args, out)

        # Quick tunes stay valid for as long as nothing else touches the RFIC's
        # fastlock state, so a single-bank plan pays for them exactly once.
        cached = [None] * len(banks)

        single = len(banks) == 1
        passes = 0

        while True:
            stops = 0
            worst_amb = 0.0
            reprimes = 0
            # Acquisition and sweeping are timed apart. They are different
            # rates: acquiring a quick tune costs a real retune plus a profile
            # store, about 21 ms a stop, while replaying one costs a scheduled
            # retune and a read. Charging the first to the second reported 34
            # stops/s for a loop running at 120, and hid the fact that a
            # multi-bank plan pays the 21 ms again on every visit.
            t_acq = 0.0
            t_run = 0.0

            for i, bank in enumerate(banks):
                # A single-bank plan keeps its profiles for the life of the
                # run. With more than one bank, each overwrites the previous
                # one's Nios slots, so every visit has to re-acquire.
                if cached[i] is None or not single:
                    t0 = time.monotonic()
                    cached[i] = acquire_tunes(dev, ch, bank, args.verbose)
                    t_acq += time.monotonic() - t0
                # One pass per bank per lap, except a single-bank plan, which
                # runs its whole quota without ever dropping the pipeline.
                want = (args.sweeps or None) if single else 1
                t0 = time.monotonic()
                n, amb, rep = run_bank(dev, ch, bank, cached[i], args, gdb,
                                       msg_samples, emit_row, want)
                t_run += time.monotonic() - t0
                stops += n
                worst_amb = max(worst_amb, amb)
                reprimes += rep

            passes += args.sweeps if (single and args.sweeps) else 1
            if args.verbose:
                print(f"# {stops} stops in {t_run:.3f} s "
                      f"({stops / max(1e-9, t_run):.0f} stops/s)"
                      + (f" + {t_acq:.3f} s acquiring quick tunes"
                         if t_acq else "")
                      + f", {100 * worst_amb:.2f}% of "
                      f"samples had a gain known only to chunk resolution"
                      + (f", {reprimes} re-primes" if reprimes else ""),
                      file=sys.stderr)
            if args.sweeps and passes >= args.sweeps:
                return 0
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            # Leave nothing queued behind us. The RFIC is left in fastlock by
            # a recall, but _rfic_host_set_frequency() clears that before the
            # next ordinary tune, so a later user of this handle is unaffected.
            dev.cancel_scheduled_retunes(ch)
            dev.enable_module(ch, False)
        finally:
            dev.close()
            if out is not sys.stdout:
                out.close()


if __name__ == "__main__":
    sys.exit(main())
