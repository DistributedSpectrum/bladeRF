#!/usr/bin/env python3
"""Capture contiguous IQ and produce a Welch PSD calibrated in dBm.

Captures N contiguous samples in a *single* bladerf_sync_rx() call, applies the
per-chunk RFIC gain of every packet the call spanned, and computes a Welch PSD
normalised so that the linear sum of any set of bins is the absolute power in
those bins. That makes RSSI a sum:

    rssi_dbm = 10*log10(sum(P_lin[k] for k in bins_of_interest))

Two things about this are easy to get wrong.

1. The gain must be applied per sample, in the time domain, BEFORE the FFT. A
   2048-point Welch segment is larger than the 2044-sample packet payload, so a
   segment can never sit inside one packet -- it always straddles a boundary and
   in general spans four gain chunks. There is no such thing as "the gain for
   this segment".

2. The normalisation must be the one whose bins sum to total power:

       P[k] = |FFT(y*w)[k]|^2 / (nperseg * sum(w^2))     averaged over segments

   Then sum(P) == mean(|y|^2). This is scipy's scaling='density' multiplied by
   the bin width, NOT scaling='spectrum' (which normalises by sum(w)^2 so a tone's
   peak bin reads its amplitude, but summing bins does not give total power).

The gain profile for a whole multi-packet read comes from
bladerf_get_rx_gain_tags() (dev.rx_gain_tags()), which returns one entry per
packet the call consumed, each with its own four-chunk profile and the slice of
the sample buffer it applies to. The summary tag overlaid on
bladerf_metadata.reserved cannot do this job: it has room for only the *first*
packet's chunk_gain_index, so with one packet per call it is exact and with 129
packets per call it describes 1/129th of the data.

The gain relation used is the one the tag was built for and validated against:

       dBm_in = dBFS - gain_db          gain_db from rx_gain_tag_to_gain_db()

with dBFS = 10*log10(mean(|raw/2048|^2)). So correcting a sample is a divide by
the linear voltage gain, y[n] = x[n] * 10**(-gain_db[n]/20), after which
mean(|y|^2) is the input power in mW.

A gain calibration table must be loaded for the result to be absolute; without one
the numbers carry a fixed offset. Requires FPGA v0.17.0 or later.

Example:
    ./rx_psd_dbm.py -f 731e6 -n 18 --gain-cal auto --plot psd.png
"""

import argparse
import os
import re
import sys

import numpy as np

from bladerf import _bladerf

FULL_SCALE = 2048.0
CHUNKS = 4

GAIN_MODES = {
    "slow": _bladerf.GainMode.SlowAttack_AGC,
    "fast": _bladerf.GainMode.FastAttack_AGC,
    "hybrid": _bladerf.GainMode.Hybrid_AGC,
    "manual": _bladerf.GainMode.Manual,
}


def welch_dbm(y, nperseg=2048, overlap=0.5, fs=20e6):
    """Welch PSD of gain-corrected samples, normalised so bins sum to power.

    Returns (freqs_hz, p_lin) where sum(p_lin) == mean(|y|**2). Sum a subset of
    p_lin and take 10*log10 to get the power in those bins.
    """
    w = np.hanning(nperseg)
    hop = max(1, int(nperseg * (1.0 - overlap)))
    acc = np.zeros(nperseg)
    nseg = 0
    for i in range(0, len(y) - nperseg + 1, hop):
        acc += np.abs(np.fft.fft(y[i:i + nperseg] * w)) ** 2
        nseg += 1
    if nseg == 0:
        raise ValueError("capture shorter than one segment")
    p = acc / nseg / (nperseg * np.sum(w ** 2))
    return (np.fft.fftshift(np.fft.fftfreq(nperseg, 1.0 / fs)),
            np.fft.fftshift(p)), nseg


def rssi_dbm(p_lin, mask):
    """Absolute power in the selected bins, dBm."""
    total = float(np.sum(p_lin[mask]))
    return 10.0 * np.log10(total) if total > 0 else float("-inf")


def gain_per_sample(tags, msg_samples, count, gdb):
    """Per-sample gain in dB across one read, from one tag per packet.

    Chunk boundaries are in *packet payload* coordinates while the output is in
    buffer coordinates, and the two differ whenever a read starts part way into a
    packet (msg_sample_offset) -- so each chunk is clipped to the part of its
    packet the read actually returned, then translated.

    A chunk whose end index differs from its start contains a gain transition at
    an unknown position. Those are split at the midpoint: the first half takes the
    gain the chunk started at, the second half the gain it ended at. If the
    transition is uniformly distributed within the chunk that is the midpoint
    estimator, halving the expected mis-assigned fraction from 0.50 of the chunk
    to 0.25 (worst case 1.00 to 0.50). In slow-attack, where a decision moves the
    gain by at most 2 dB and 4.3% of packets contain one, the residual is under
    0.01 dB on total power -- so nothing has to be discarded.

    Returns (gain_db, samples_filled, ambiguous_samples). Entries tile the
    returned samples, so samples_filled == count unless the tag array was
    truncated.
    """
    gain = np.zeros(count, dtype=np.float32)
    clen = msg_samples // CHUNKS
    filled = 0
    ambiguous = 0

    for t in tags:
        # base index at the packet's first sample, then the index at the end of
        # each chunk: a CHUNKS+1 point profile across the packet
        pts = [t.gain_index] + list(t.chunk_gain_index[:CHUNKS])
        end = t.msg_sample_offset + t.sample_count

        for c in range(CHUNKS):
            lo = max(c * clen, t.msg_sample_offset)
            hi = min((c + 1) * clen if c < CHUNKS - 1 else msg_samples, end)
            if hi <= lo:
                continue                      # chunk outside this entry's span
            blo = t.sample_offset + (lo - t.msg_sample_offset)
            bhi = t.sample_offset + (hi - t.msg_sample_offset)

            if pts[c] == pts[c + 1]:
                gain[blo:bhi] = gdb(pts[c + 1])
            else:
                mid = blo + (bhi - blo) // 2
                gain[blo:mid] = gdb(pts[c])
                gain[mid:bhi] = gdb(pts[c + 1])
                ambiguous += bhi - blo
            filled += bhi - blo

    return gain, filled, ambiguous


def capture(dev, ch, msg_samples, want, args):
    """One contiguous read of `want` samples.

    Returns (iq_normalised, gain_db_per_sample, info).

    A single sync_rx() call is contiguous by construction: the sync layer walks
    consecutive packets and, if it ever meets a timestamp discontinuity, stops
    there and reports BLADERF_META_STATUS_OVERRUN with a short actual_count. So
    the check is simply "did it return everything asked for", and a short read is
    retried rather than stitched.
    """
    import time
    ffi = _bladerf.ffi
    meta = ffi.new("struct bladerf_metadata *")
    buf = bytearray(4 * want)

    # Let the AGC converge before capturing. Streaming starts at maximum gain, so
    # without this the capture contains the whole ramp down -- and any part the
    # front end compressed at high gain is nonlinear, which no amount of gain
    # correction can undo.
    if args.settle > 0:
        settle_buf = bytearray(4 * msg_samples)
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.settle:
            meta.flags = 0x80000000
            meta.timestamp = 0
            meta.status = 0
            try:
                dev.sync_rx(settle_buf, msg_samples, 3500, meta)
            except Exception:
                break

    # The read itself takes want/fs seconds, so the timeout has to cover that
    # plus the time the samples spend queued in the buffer pool.
    timeout_ms = int(1000.0 * want / args.sample_rate) + 3000
    cache = {}

    def gdb(idx):
        """Index -> dB, memoised. In FPGA tuning mode each miss costs a USB round
        trip to read the LO frequency, and there are only a handful of distinct
        indices in a capture."""
        if idx not in cache:
            g = dev.rx_gain_tag_to_gain_db(ch, idx)
            if g is None:
                raise RuntimeError(f"gain index {idx} outside the gain table. Is "
                                   f"RFIC register 0x035 still 0x16?")
            cache[idx] = g
        return cache[idx]

    short = 0
    for attempt in range(args.max_tries):
        meta.flags = 0x80000000                    # RX_NOW: start wherever we are
        meta.timestamp = 0
        meta.status = 0
        dev.sync_rx(buf, want, timeout_ms, meta)

        if meta.actual_count != want or (meta.status & 0x1):
            # discontinuity part way through; the capture would be invalid
            short += 1
            continue

        tags = dev.rx_gain_tags()
        if tags is None:
            raise RuntimeError("no gain tag: needs FPGA v0.17.0+ and a metadata "
                               "RX format")
        if not tags:
            raise RuntimeError("the read returned no gain tag entries")

        summary = _bladerf.rx_gain_tag(meta)
        gain, filled, ambiguous = gain_per_sample(tags, msg_samples, want, gdb)
        if filled != want:
            raise RuntimeError(f"gain tags covered {filled} of {want} samples")

        v = np.frombuffer(buf, dtype=np.int16, count=2 * want)
        iq = (v[0::2].astype(np.float32)
              + 1j * v[1::2].astype(np.float32)) / FULL_SCALE

        info = {"packets": len(tags), "restarts": short,
                "ambiguous_samples": ambiguous,
                "carried": sum(1 for t in tags if t.carried),
                # num_messages counts headers read, which can exceed the number
                # of entries: a header read but contributing no samples -- the
                # last one before a discontinuity ended the call, or one walked
                # past while seeking to a requested timestamp -- is counted there
                # and dropped from the array, which only holds entries that
                # describe samples.
                "num_messages": summary.num_messages,
                "index_min": summary.gain_index_min,
                "index_max": summary.gain_index_max,
                "distinct_gains": sorted(set(cache.values()))}
        return iq, gain, info

    raise RuntimeError(f"could not obtain {want} contiguous samples in "
                       f"{args.max_tries} attempts ({short} short reads). Try a "
                       f"larger --num-buffers, or a smaller -n.")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-f", "--freq", type=float, required=True,
                    help="center frequency in Hz")
    ap.add_argument("-n", "--log2n", type=int, default=18,
                    help="capture 2**N samples (default 18 = 262144)")
    ap.add_argument("-s", "--sample-rate", type=float, default=20e6)
    ap.add_argument("-b", "--bandwidth", type=float, default=None)
    ap.add_argument("-c", "--channel", type=int, default=0)
    ap.add_argument("-g", "--gain-mode", choices=sorted(GAIN_MODES),
                    default="slow")
    ap.add_argument("--manual-gain", type=int, default=None)
    ap.add_argument("--nperseg", type=int, default=2048)
    ap.add_argument("--overlap", type=float, default=0.5)
    ap.add_argument("--gain-cal", metavar="PATH",
                    help="RX gain calibration table, or 'auto'. Required for the "
                         "result to be absolute")
    ap.add_argument("--settle", type=float, default=0.25, metavar="SEC",
                    help="stream and discard for this long before capturing "
                         "(default 0.25) so the AGC converges first. Without it "
                         "the capture starts at maximum gain and walks down, and "
                         "any samples the front end compressed at high gain "
                         "cannot be rescued by a gain correction")
    ap.add_argument("--num-buffers", type=int, default=256)
    ap.add_argument("--num-transfers", type=int, default=32)
    ap.add_argument("--buffer-size", type=int, default=16384, metavar="SAMPLES",
                    help="samples per USB buffer, rounded up to a whole number "
                         "of packets (2048 on SuperSpeed). Larger buffers mean "
                         "fewer USB transfers per second; num_buffers * "
                         "buffer_size / sample_rate is how long the reader may "
                         "stall before the capture has to restart")
    ap.add_argument("--max-tries", type=int, default=20, metavar="N",
                    help="restart the capture at most N times when a "
                         "discontinuity truncates it")
    ap.add_argument("--rssi", metavar="LO:HI",
                    help="also report the RSSI between two baseband offsets in "
                         "MHz. Use = for negative values, which argparse would "
                         "otherwise read as a flag: --rssi=-1.5:1.5")
    ap.add_argument("--plot", metavar="PATH", help="write a PSD plot")
    ap.add_argument("--save-npz", metavar="PATH",
                    help="save freqs, psd_dbm and the corrected IQ")
    args = ap.parse_args()

    want = 2 ** args.log2n
    bandwidth = args.bandwidth or 0.8 * args.sample_rate
    ch = _bladerf.CHANNEL_RX(args.channel)

    dev = _bladerf.BladeRF()
    fpga = dev.get_fpga_version()
    if (fpga.major, fpga.minor) < (0, 17):
        print(f"FPGA {fpga} does not tag RX packets with the RFIC gain; "
              f"v0.17.0 or later required.", file=sys.stderr)
        return 2

    calibrated = False
    if args.gain_cal:
        path = args.gain_cal
        if path == "auto":
            path = f"{dev.get_serial()}_rx_gain_cal.tbl"
        else:
            m = re.search(r"([0-9a-fA-F]{32})", os.path.basename(path))
            if m and m.group(1).lower() != dev.get_serial().lower():
                print(f"WARNING: {os.path.basename(path)} was swept on "
                      f"{m.group(1)}, this device is {dev.get_serial()}",
                      file=sys.stderr)
        try:
            dev.set_gain_calibration(ch, path)
            calibrated = True
        except Exception as exc:
            print(f"failed to load gain calibration {path}: {exc}",
                  file=sys.stderr)
            return 2

    dev.set_sample_rate(ch, int(args.sample_rate))
    dev.set_bandwidth(ch, int(bandwidth))
    dev.set_frequency(ch, int(args.freq))
    dev.set_gain_mode(ch, GAIN_MODES[args.gain_mode])
    if args.gain_mode == "manual" and args.manual_gain is not None:
        dev.set_gain(ch, args.manual_gain)

    speed = dev.get_device_speed()
    nsamples = 2044 if "super" in str(speed).lower() else 1020

    if args.num_transfers >= args.num_buffers:
        print("--num-transfers must be less than --num-buffers", file=sys.stderr)
        return 2

    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11_META,
                    num_buffers=args.num_buffers, buffer_size=args.buffer_size,
                    num_transfers=args.num_transfers, stream_timeout=3500)
    dev.enable_module(ch, True)
    try:
        iq, gain_db, info = capture(dev, ch, nsamples, want, args)
    finally:
        dev.enable_module(ch, False)

    # dBFS of the raw capture, for reference
    dbfs = 10 * np.log10(float(np.mean(np.abs(iq) ** 2)) + 1e-300)

    # per-sample correction: after this, mean(|y|^2) is power in mW
    y = iq * (10.0 ** (-gain_db / 20.0)).astype(np.float32)

    (freqs, p_lin), nseg = welch_dbm(y, args.nperseg, args.overlap,
                                     args.sample_rate)
    total_dbm = 10 * np.log10(float(np.sum(p_lin)))
    direct_dbm = 10 * np.log10(float(np.mean(np.abs(y) ** 2)))

    print(f"FPGA {fpga}, {args.freq/1e6:.3f} MHz, "
          f"{args.sample_rate/1e6:.3f} Msps, {args.gain_mode} AGC")
    slack_ms = args.num_buffers * args.buffer_size / args.sample_rate * 1e3
    print(f"capture     : 2^{args.log2n} = {want} samples "
          f"({want/args.sample_rate*1e3:.2f} ms) in one sync_rx(), "
          f"{info['packets']} packets, contiguous "
          f"(restarts: {info['restarts']})")
    print(f"buffers     : {args.num_buffers} x {args.buffer_size} samples "
          f"= {slack_ms:.1f} ms of slack; {info['packets']} gain tags, "
          f"{info['num_messages']} headers read"
          f"{', 1 carried from the previous call' if info['carried'] else ''}")
    print(f"gain        : {', '.join(f'{g:.2f}' for g in info['distinct_gains'])} dB"
          f" (index {info['index_min']}..{info['index_max']})"
          f"{'' if calibrated else '  [UNCALIBRATED -- values carry an offset]'}")
    amb = info["ambiguous_samples"]
    print(f"              {amb} samples ({100*amb/want:.2f}%) sit in a chunk "
          f"where the gain changed; those chunks are split at their midpoint, "
          f"so about a quarter of them carry a one-step gain error")
    print(f"welch       : {nseg} segments of {args.nperseg}, hann, "
          f"{args.overlap*100:.0f}% overlap, "
          f"bin = {args.sample_rate/args.nperseg/1e3:.3f} kHz")
    print()
    print(f"mean power (dBFS, uncorrected) : {dbfs:+9.3f} dBFS")
    print(f"sum of PSD bins               : {total_dbm:+9.3f} dBm")
    print(f"time-domain mean(|y|^2)       : {direct_dbm:+9.3f} dBm   "
          f"(consistency check, delta {total_dbm-direct_dbm:+.4f} dB)")

    if args.rssi:
        lo, hi = (float(v) * 1e6 for v in args.rssi.split(":"))
        mask = (freqs >= lo) & (freqs <= hi)
        print(f"\nRSSI over {lo/1e6:+.3f}..{hi/1e6:+.3f} MHz "
              f"({mask.sum()} bins)   : {rssi_dbm(p_lin, mask):+9.3f} dBm")

    psd_dbm = 10 * np.log10(p_lin + 1e-300)
    if args.save_npz:
        np.savez(args.save_npz, freqs=freqs, psd_dbm=psd_dbm, p_lin=p_lin,
                 iq_corrected=y, gain_db=gain_db)
        print(f"\nsaved {args.save_npz}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(freqs / 1e6, psd_dbm, lw=0.6)
        ax.set_xlabel("baseband offset (MHz)")
        ax.set_ylabel("dBm per bin" + ("" if calibrated else " (uncalibrated)"))
        ax.set_title(f"{args.freq/1e6:.3f} MHz, {nseg}x{args.nperseg} Welch, "
                     f"total {total_dbm:+.2f} dBm")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"wrote {args.plot}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
