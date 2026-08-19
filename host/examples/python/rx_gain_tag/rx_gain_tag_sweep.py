#!/usr/bin/env python3
"""Log per-packet timestamps and RFIC gain across a frequency hop sequence.

Dwells on each center frequency in turn and records, for every RX metadata
packet that arrives, the packet timestamp and the gain profile the FPGA tagged
onto it. The point is to see what the AGC does as the frequency changes -- both
how long the retune takes to show up in the sample stream and how the AGC
re-converges afterwards.

Each packet ("message") is one FPGA DMA buffer: 2044 samples on SuperSpeed,
1020 on Hi-Speed. Its header carries the gain index at the first sample plus one
index per chunk of the message, so the gain travels with the IQ instead of
having to be polled asynchronously.

A read covers several packets (--read-packets, 2 by default) and
bladerf_get_rx_gain_tags() then returns one entry per packet, so there is still a
row per packet here while the loop makes half as many calls -- or fewer. The dBm
column is built from each packet's own chunk profile, dividing every span of
constant gain by its own linear gain before averaging, so a packet the AGC moved
inside is handled correctly rather than being assigned one gain.

How literally to read that per-chunk profile depends on -g. Slow-attack updates
gain at most once per 1000 us, so with a 102 us packet at 20 Msps the profile is
exact: measured 95.7% of packets flat, none with two transitions. Fast-attack
updates every 1 us, roughly 25 decisions per chunk, and each chunk records only
its final value -- measured 85.8% flat, 8.6% with more than one transition, and
excursions up to 29 indices in a single packet. In fast-attack use the min/max
pair and drop packets flagged C rather than trusting the four values.

Requires FPGA v0.17.0 or later.

Examples:

  # three frequencies, 200 ms each, two passes
  ./rx_gain_tag_sweep.py -f 731e6,915e6,2400e6 -d 0.2 -r 2

  # log to CSV, include measured IQ power so the gain correction is visible
  ./rx_gain_tag_sweep.py -f 731,915,2400 --mhz -d 0.5 --power --csv sweep.csv
"""

import argparse
import csv
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gain_profile as gp                                      # noqa: E402
from bladerf import _bladerf                                   # noqa: E402

GAIN_MODES = {
    "slow": _bladerf.GainMode.SlowAttack_AGC,
    "fast": _bladerf.GainMode.FastAttack_AGC,
    "hybrid": _bladerf.GainMode.Hybrid_AGC,
    "manual": _bladerf.GainMode.Manual,
    "default": _bladerf.GainMode.Default,
}


def parse_freqs(text, mhz):
    out = []
    for tok in text.replace(" ", "").split(","):
        if not tok:
            continue
        val = float(tok)
        out.append(int(val * 1e6) if mhz else int(val))
    if not out:
        raise argparse.ArgumentTypeError("no frequencies given")
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-f", "--freqs", required=True,
                    help="comma-separated center frequencies (Hz, or MHz with "
                         "--mhz), e.g. 731e6,915e6,2400e6")
    ap.add_argument("--mhz", action="store_true",
                    help="treat --freqs as MHz")
    ap.add_argument("-d", "--dwell", type=float, default=0.25,
                    help="seconds to dwell at each frequency (default 0.25)")
    ap.add_argument("-r", "--repeat", type=int, default=1,
                    help="passes through the frequency list (default 1)")
    ap.add_argument("-s", "--sample-rate", type=float, default=30.72e6,
                    help="sample rate in Hz (default 30.72e6)")
    ap.add_argument("-b", "--bandwidth", type=float, default=None,
                    help="RX bandwidth in Hz (default: 0.8 * sample rate)")
    ap.add_argument("-c", "--channel", type=int, default=0,
                    help="RX channel index (default 0; the gain tag is RX1 only)")
    ap.add_argument("-g", "--gain-mode", choices=sorted(GAIN_MODES),
                    default="slow", help="RX gain mode (default slow)")
    ap.add_argument("--manual-gain", type=int, default=None,
                    help="gain in dB, only with --gain-mode manual")
    ap.add_argument("--gain-cal", metavar="PATH",
                    help="load an RX gain calibration table so that the dBm "
                         "column is an absolute reference. Accepts the binary "
                         ".tbl form or a .csv sweep (converted to .tbl beside "
                         "it on load). Pass 'auto' to use the device's own "
                         "<serial>_rx_gain_cal.tbl from the libbladeRF search "
                         "path")
    ap.add_argument("--no-gain-cal", action="store_true",
                    help="explicitly disable an already-loaded gain "
                         "calibration table for this run")
    ap.add_argument("--power", action="store_true",
                    help="also compute IQ power per packet and the absolute "
                         "power after removing the packet's own per-chunk gain")
    ap.add_argument("--csv", metavar="PATH", help="write a row per packet")
    ap.add_argument("--read-packets", type=int, default=2, metavar="N",
                    help="packets per sync_rx() call (default 2, i.e. 4088 "
                         "samples on SuperSpeed). Reading whole packets keeps "
                         "every gain tag entry a complete packet, so there is "
                         "still one CSV row per packet; larger values just mean "
                         "fewer calls per second")
    ap.add_argument("--buffer-size", type=int, default=8192, metavar="SAMPLES",
                    help="samples per USB buffer, rounded up to whole packets "
                         "(default 8192 = 4 packets on SuperSpeed). Independent "
                         "of --read-packets: this sizes the USB transfers, that "
                         "sizes the reads")
    ap.add_argument("--num-buffers", type=int, default=256,
                    help="stream buffers (default 256). With --buffer-size this "
                         "sets how long a stall the loop can absorb before it "
                         "must resync and lose samples: "
                         "num_buffers * buffer_size / sample_rate")
    ap.add_argument("--num-transfers", type=int, default=32,
                    help="USB transfers in flight (default 32; must be less "
                         "than --num-buffers)")
    ap.add_argument("--no-schedule", action="store_true",
                    help="hop with bladerf_set_frequency() instead of a "
                         "scheduled retune. That call takes milliseconds and the "
                         "RF moves somewhere inside it -- measured 13.5 ms wide "
                         "with the change 5.8 ms in at 20 Msps -- so every packet "
                         "in that window has an unknown frequency, and an unknown "
                         "gain band with it. Scheduled retunes narrow that to the "
                         "one packet the boundary falls inside")
    ap.add_argument("--hop-lead", type=float, default=5.0, metavar="MS",
                    help="schedule each hop this far ahead of the stream "
                         "(default 5 ms), enough for the command to reach the "
                         "FPGA before the timestamp it names")
    ap.add_argument("--now", action="store_true",
                    help="request each read with RX_NOW instead of reading "
                         "contiguously. Simpler, but discards whatever arrived "
                         "between calls, so packets have gaps and the AGC's "
                         "evolution cannot be followed packet to packet")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="only print the per-frequency summary")
    args = ap.parse_args()

    freqs = parse_freqs(args.freqs, args.mhz)
    bandwidth = args.bandwidth if args.bandwidth else 0.8 * args.sample_rate
    ch = _bladerf.CHANNEL_RX(args.channel)

    dev = _bladerf.BladeRF()

    fpga = dev.get_fpga_version()
    if (fpga.major, fpga.minor) < (0, 17):
        print(f"FPGA {fpga} does not tag RX packets with the RFIC gain; "
              f"v0.17.0 or later is required.", file=sys.stderr)
        return 2

    speed_name = gp.speed_name(dev)
    nsamples = gp.MESSAGE_SAMPLES[speed_name]
    read_n = gp.read_samples(nsamples, args.read_packets)
    lead_samples = max(1, int(args.hop_lead * 1e-3 * args.sample_rate))

    # Load the gain calibration table before anything reads gain back. Loading
    # resets the gain (libbladeRF restores the gain mode afterwards), so it has
    # to happen before the mode and frequency are set up below.
    cal_loaded = False
    if args.gain_cal:
        path = None if args.gain_cal == "auto" else args.gain_cal

        # A table from another board loads with only a log warning, which is easy
        # to miss and silently makes every absolute figure wrong. The filename
        # normally carries the serial it was swept on, so check it up front.
        if path is not None:
            serial = dev.get_serial()
            m = re.search(r"([0-9a-fA-F]{32})", os.path.basename(path))
            if m and m.group(1).lower() != serial.lower():
                print(f"WARNING: {os.path.basename(path)} was swept on device "
                      f"{m.group(1)}, but this device is {serial}. Absolute "
                      f"power figures will be wrong.", file=sys.stderr)
        try:
            if path is None:
                # libbladeRF's own default needs a NULL path, which the binding
                # cannot express (it always encodes the string), so build the
                # same name it would: <serial>_rx_gain_cal.tbl, resolved through
                # the search path it uses for FPGA images.
                path = f"{dev.get_serial()}_rx_gain_cal.tbl"
            dev.set_gain_calibration(ch, path)
            cal_loaded = True
        except Exception as exc:
            print(f"failed to load gain calibration "
                  f"{'(auto)' if path is None else path}: {exc}",
                  file=sys.stderr)
            return 2
    if args.no_gain_cal:
        try:
            dev.enable_gain_calibration(ch, False)
        except Exception as exc:
            print(f"could not disable gain calibration: {exc}", file=sys.stderr)

    dev.set_sample_rate(ch, int(args.sample_rate))
    dev.set_bandwidth(ch, int(bandwidth))
    dev.set_frequency(ch, freqs[0])
    if args.gain_mode == "manual":
        dev.set_gain_mode(ch, GAIN_MODES["manual"])
        if args.manual_gain is not None:
            dev.set_gain(ch, args.manual_gain)
    else:
        dev.set_gain_mode(ch, GAIN_MODES[args.gain_mode])

    print(f"# FPGA {fpga}, {speed_name}: {nsamples} samples/packet, "
          f"{args.sample_rate/1e6:.3f} Msps, gain mode {args.gain_mode}")
    print(f"# reading {args.read_packets} packet(s) = {read_n} samples per call, "
          f"one gain profile per packet")
    print(f"# {len(freqs)} frequencies x {args.dwell:g} s x {args.repeat} pass"
          f"{'es' if args.repeat != 1 else ''}")
    if args.power:
        if cal_loaded:
            print(f"# gain calibration loaded"
                  f"{'' if args.gain_cal == 'auto' else ' from ' + args.gain_cal}"
                  f": the dBm column is an absolute reference")
        else:
            print("# no gain calibration loaded: the dBm column is relative to "
                  "full scale and carries an uncalibrated offset")
    print(f"# packet timestamps are in samples at the RX sample rate")

    # Retunes at an exact sample timestamp, so a packet's frequency follows from
    # its timestamp instead of from a window. A quick tune profile is captured
    # from the tuned state, so each frequency is visited once up front -- with
    # full calibration, which the fast lock profile then restores on every hop.
    scheduled = not args.no_schedule
    qt = {}
    if scheduled:
        try:
            for f in freqs:
                dev.set_frequency(ch, f)
                qt[f] = dev.get_quick_tune(ch)
            dev.set_frequency(ch, freqs[0])
        except Exception as exc:
            print(f"# scheduled retunes unavailable ({exc}); falling back to "
                  f"set_frequency()", file=sys.stderr)
            scheduled = False
            qt = {}
    print(f"# hops via {'scheduled retune at an exact timestamp'
                        if scheduled else 'set_frequency() (windowed)'}")

    if args.num_transfers >= args.num_buffers:
        print("--num-transfers must be less than --num-buffers", file=sys.stderr)
        return 2
    slack_ms = args.num_buffers * args.buffer_size / args.sample_rate * 1e3
    print(f"# {args.num_buffers} buffers x {args.buffer_size} samples / "
          f"{args.num_transfers} transfers = {slack_ms:.1f} ms of slack before "
          f"a resync")
    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11_META,
                    num_buffers=args.num_buffers, buffer_size=args.buffer_size,
                    num_transfers=args.num_transfers, stream_timeout=3500)
    dev.enable_module(ch, True)

    ffi = _bladerf.ffi
    meta = ffi.new("struct bladerf_metadata *")
    buf = bytearray(4 * read_n)

    rows = []
    header_printed = False
    # Contiguous reads: after the first packet, ask for exactly the samples that
    # follow the previous one. RX_NOW instead returns whatever is current and
    # silently drops the rest, which leaves gaps between packets.
    next_ts = None
    resyncs = 0
    # Reading contiguously means asking for the samples after the previous
    # packet, so the reader runs behind real time by up to
    # num_buffers * nsamples / sample_rate. set_frequency() takes effect in real
    # time, so for that long after a retune the packets still contain the
    # PREVIOUS frequency's samples. Record the sample counter at each retune and
    # attribute every packet by its timestamp instead of by the last request.
    # (ts_lo, ts_hi, freq): the RF certainly changed somewhere in [ts_lo, ts_hi].
    #
    # A scheduled retune makes that an instant -- ts_lo == ts_hi == the timestamp
    # the retune was scheduled for -- so the only uncertain packet is the one the
    # boundary falls inside, and it genuinely holds samples of both frequencies.
    #
    # set_frequency() cannot do better than a bracket. It moves the LO early in
    # its sequence and then spends milliseconds on band selection and
    # recalibration, so a timestamp read after it returns is late: measured 13.5 ms
    # wide at 20 Msps with the RF actually changing 5.8 ms in. Every packet in that
    # window has an unknown frequency, and since the index -> dB mapping is band
    # dependent, an unknown gain too -- so those rows carry dBFS but no dB or dBm.
    bounds = [(0, 0, freqs[0])]
    # index -> dB depends on the tuned band as well as the index, and the call
    # costs a USB round trip in FPGA tuning mode, so it is memoised per
    # (frequency, index) rather than paid per packet -- let alone per chunk.
    gains = gp.GainDb(dev, ch)
    try:
        for lap in range(args.repeat):
            for seg, freq in enumerate(freqs):
                if scheduled:
                    # Far enough ahead that the command reaches the FPGA first,
                    # and ahead of the reader so no already-consumed sample can
                    # fall on the far side of the boundary.
                    now = int(dev.get_timestamp(_bladerf.Direction.RX))
                    t_hop = max(now, next_ts or 0) + lead_samples
                    dev.schedule_retune(ch, t_hop, freq, qt[freq])
                    bounds.append((t_hop, t_hop, freq))
                else:
                    ts_before = int(dev.get_timestamp(_bladerf.Direction.RX))
                    dev.set_frequency(ch, freq)
                    ts_after = int(dev.get_timestamp(_bladerf.Direction.RX))
                    bounds.append((ts_before, ts_after, freq))
                retune_wall = time.monotonic()
                # The dwell is counted in samples from the retune, not in wall
                # clock from the request: the reader runs behind the stream, so a
                # wall clock dwell ends before that many samples have been read.
                dwell_end = bounds[-1][1] + int(args.dwell * args.sample_rate)
                retune_ts = bounds[-1][1]
                prev_end = None
                pkt = 0

                # ... with a wall clock backstop in case the stream stalls
                while (next_ts is None or next_ts < dwell_end) and \
                        time.monotonic() - retune_wall < 3 * args.dwell + 1.0:
                    meta.status = 0
                    if args.now or next_ts is None:
                        meta.flags = 0x80000000  # BLADERF_META_FLAG_RX_NOW
                        meta.timestamp = 0
                    else:
                        meta.flags = 0
                        meta.timestamp = next_ts
                    try:
                        dev.sync_rx(buf, read_n, 3500, meta)
                    except Exception as exc:
                        # Usually means the requested timestamp already went by,
                        # i.e. this loop fell behind the stream. Resynchronise on
                        # the current sample instead of giving up.
                        if next_ts is not None and not args.now:
                            next_ts = None
                            resyncs += 1
                            continue
                        print(f"  sync_rx failed: {exc}", file=sys.stderr)
                        break
                    # actual_count, not read_n: a discontinuity inside the read
                    # ends it early, and the next request must follow what did
                    # arrive.
                    next_ts = int(meta.timestamp) + int(meta.actual_count)

                    # One entry per packet the read spanned, each with its own
                    # timestamp and chunk profile. Whole-packet reads mean every
                    # entry is a complete packet, so a row is still a packet.
                    tags = dev.rx_gain_tags()
                    if tags is None:
                        print("  no gain tag: needs FPGA v0.17.0+ and a metadata "
                              "RX format", file=sys.stderr)
                        break
                    v = (gp.samples_view(buf, int(meta.actual_count))
                         if args.power else None)

                    for t in tags:
                        ts_i = int(t.timestamp) + t.msg_sample_offset
                        count = int(t.sample_count)
                        if retune_ts is None:
                            retune_ts = ts_i

                        # Samples missing between this packet and the previous
                        # one. Non-zero means the consumer fell behind, and a
                        # gain change spanning the gap cannot be pinned to a
                        # single moment.
                        gap = 0 if prev_end is None else int(ts_i - prev_end)
                        prev_end = ts_i + count

                        # Which frequency was actually tuned when these samples
                        # were captured, as opposed to the one most recently
                        # requested.
                        j = 0
                        for k in range(len(bounds) - 1, -1, -1):
                            if bounds[k][0] < ts_i + count:
                                j = k
                                break
                        b_lo, b_hi, b_freq = bounds[j]
                        if ts_i >= b_hi:
                            true_freq, certain = b_freq, True
                        elif ts_i + count <= b_lo:
                            true_freq = bounds[j - 1][2] if j > 0 else bounds[0][2]
                            certain = True
                        else:
                            true_freq, certain = b_freq, False

                        # Convert indices against the band that was really tuned
                        # for these samples, not the one last asked for -- and not
                        # at all when the band is unknown, since a conversion
                        # against the wrong band is a plausible-looking wrong
                        # number (1.46 dB apart for index 72 between 951 and
                        # 731 MHz).
                        gdb = gains.at(true_freq) if certain else None
                        gain_db = gdb(t.gain_index) if certain else None
                        idx_min, idx_max, changed = gp.profile(t)

                        dbfs = dbm = None
                        if args.power:
                            dbfs, dbm = gp.packet_power(v, t, nsamples, gdb)

                        row = {
                            "lap": lap,
                            "seg": seg,
                            "freq_hz": freq,
                            "pkt": pkt,
                            "timestamp": ts_i,
                            "ts_since_retune": ts_i - retune_ts,
                            "wall_s": round(time.monotonic() - retune_wall, 6),
                            "count": count,
                            "gap_samples": gap,
                            "true_freq_hz": true_freq,
                            "mislabeled": int(certain and true_freq != freq),
                            "certain": int(certain),
                            "status": int(meta.status),
                            "gain_index": t.gain_index,
                            "gain_db": (round(gain_db, 2)
                                        if gain_db is not None else None),
                            "idx_min": idx_min,
                            "idx_max": idx_max,
                            "chunks": ",".join(str(c) for c in
                                               t.chunk_gain_index[:4]),
                            "changed": int(changed),
                            "locked": int(t.locked),
                            "carried": int(t.carried),
                            "pkts_in_read": len(tags),
                            "dbfs": round(dbfs, 2) if dbfs is not None else None,
                            "dbm": round(dbm, 2) if dbm is not None else None,
                        }
                        rows.append(row)

                        if not args.quiet:
                            if not header_printed:
                                hdr = (f"{'freq_MHz':>9} {'pkt':>4} "
                                       f"{'timestamp':>12} {'dts':>8} {'gap':>8} "
                                       f"{'idx':>4} {'gain_dB':>8} "
                                       f"{'chunk profile':>17} {'flg':>4}")
                                if args.power:
                                    hdr += f" {'dBFS':>7} {'dBm':>8}"
                                print(hdr)
                                header_printed = True
                            # L is fast-attack AGC only: the AD9361 drives gain
                            # lock from the fast-attack state machine, so it stays
                            # '-' in slow and hybrid modes by design.
                            flg = ("C" if changed else "-") + \
                                  ("L" if t.locked else "-") + \
                                  ("?" if not certain else "")
                            line = (f"{freq/1e6:9.3f} {pkt:4d} {ts_i:12d} "
                                    f"{row['ts_since_retune']:8d} {gap:8d} "
                                    f"{t.gain_index:4d} "
                                    f"{gain_db if gain_db is not None else float('nan'):8.2f} "
                                    f"{row['chunks']:>17} {flg:>4}")
                            if args.power:
                                line += (f" {dbfs:7.2f} "
                                         f"{dbm if dbm is not None else float('nan'):8.2f}")
                            print(line)
                        pkt += 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        dev.enable_module(ch, False)

    if args.csv and rows:
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {len(rows)} packets to {args.csv}")

    # ---- per-dwell summary: what the AGC did at each frequency
    print("\nper-dwell summary")
    print(f"{'lap':>4} {'freq_MHz':>10} {'pkts':>5} {'idx first':>10} "
          f"{'idx last':>9} {'idx range':>11} {'changed':>8} {'settle pkt':>11} "
          f"{'settle ms':>10} {'dropped':>9}")
    for lap in range(args.repeat):
        for seg, freq in enumerate(freqs):
            # Exclude packets whose samples predate this retune: they belong to
            # the previous step and would otherwise dominate the "settling".
            sub = [r for r in rows if r["lap"] == lap and r["seg"] == seg
                   and r["gain_index"] is not None and not r["mislabeled"]
                   and r["certain"]]
            if not sub:
                continue
            idx = [r["gain_index"] for r in sub]
            nchanged = sum(r["changed"] for r in sub)
            # first packet after which the index stops moving for the rest of
            # the dwell -- i.e. how long the AGC took to re-converge
            settle = len(sub) - 1
            for i in range(len(sub)):
                if all(v == idx[i] for v in idx[i:]):
                    settle = i
                    break
            dropped = sum(r["gap_samples"] for r in sub if r["gap_samples"] > 0)
            settle_ms = sub[settle]["ts_since_retune"] / args.sample_rate * 1e3
            print(f"{lap:>4} {freq/1e6:>10.3f} {len(sub):>5} {idx[0]:>10} "
                  f"{idx[-1]:>9} {min(idx):>4}..{max(idx):<6} {nchanged:>8} "
                  f"{settle:>11} {settle_ms:>10.1f} {dropped:>9}")

    overruns = sum(1 for r in rows if r["status"] & 0x1)
    untagged = sum(1 for r in rows if r["gain_index"] is None)
    dropped = sum(r["gap_samples"] for r in rows if r["gap_samples"] > 0)
    mislabeled = sum(r["mislabeled"] for r in rows)
    print(f"\n{len(rows)} packets in {len(rows) // max(1, args.read_packets)} "
          f"reads, {overruns} with overrun status, {untagged} without a gain "
          f"tag, {dropped} samples dropped, {resyncs} resyncs")
    uncertain = sum(1 for r in rows if not r["certain"])
    if mislabeled or uncertain:
        print(f"{mislabeled} packets ({100*mislabeled/len(rows):.1f}%) held "
              f"samples captured at the PREVIOUS frequency: the reader lags real "
              f"time, so a retune reaches the data later than it is requested. "
              f"{uncertain} packets straddled a retune "
              f"{'boundary' if scheduled else 'window'} and have no gain_db/dbm, "
              f"since the band -- and so the index -> dB mapping -- is not known "
              f"for them. Both are marked in the CSV (mislabeled / certain, plus "
              f"true_freq_hz) and excluded from the summary above.")
    if not scheduled and uncertain:
        print(f"that is {uncertain/max(1,len(freqs)*args.repeat):.0f} packets per "
              f"hop; scheduled retunes (drop --no-schedule) reduce it to the one "
              f"packet the boundary lands inside.")
    if dropped:
        print("note: samples were dropped, so a gain change spanning a gap "
              "cannot be pinned to one moment. Lower --sample-rate to keep up.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
