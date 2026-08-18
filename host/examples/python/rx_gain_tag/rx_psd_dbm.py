#!/usr/bin/env python3
"""Capture contiguous IQ and produce a Welch PSD calibrated in dBm.

Captures N contiguous samples, applies the per-chunk RFIC gain from each packet's
metadata header, and computes a Welch PSD normalised so that the linear sum of any
set of bins is the absolute power in those bins. That makes RSSI a sum:

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


def capture(dev, ch, nsamples, want, args):
    """Contiguous capture. Returns (iq_normalised, gain_db_per_sample, info)."""
    ffi = _bladerf.ffi
    bufsize = nsamples + 4
    meta = ffi.new("struct bladerf_metadata *")
    buf = bytearray(4 * bufsize)

    npkt = -(-want // nsamples)
    iq = np.empty(npkt * nsamples, dtype=np.complex64)
    gain = np.empty(npkt * nsamples, dtype=np.float32)

    cache = {}
    next_ts = None
    got = 0
    gaps = 0
    ambiguous = 0
    prev_ts = None
    tries = 0

    while got < npkt:
        if next_ts is None:
            meta.flags = 0x80000000
            meta.timestamp = 0
        else:
            meta.flags = 0
            meta.timestamp = next_ts
        meta.status = 0
        try:
            dev.sync_rx(buf, nsamples, 3500, meta)
        except _bladerf.TimePastError:
            # fell behind; restart the capture so it stays contiguous
            tries += 1
            if tries > 20:
                raise RuntimeError("could not obtain a contiguous capture")
            next_ts, got, gaps, ambiguous, prev_ts = None, 0, 0, 0, None
            continue
        ts = int(meta.timestamp)
        if prev_ts is not None and ts != prev_ts + nsamples:
            # discontinuity: the capture would be invalid, start over
            gaps += 1
            next_ts, got, ambiguous, prev_ts = None, 0, 0, None
            continue
        prev_ts = ts
        next_ts = ts + nsamples

        tag = _bladerf.rx_gain_tag(meta)
        if tag is None:
            raise RuntimeError("no gain tag: needs FPGA v0.17.0+ and a metadata "
                               "RX format")

        v = np.frombuffer(buf, dtype=np.int16, count=2 * nsamples)
        sl = slice(got * nsamples, (got + 1) * nsamples)
        iq[sl] = (v[0::2].astype(np.float32)
                  + 1j * v[1::2].astype(np.float32)) / FULL_SCALE

        # Per-sample gain, piecewise constant over the packet's chunks. base is
        # the index at sample 0; chunk_gain_index[i] is the index at the END of
        # chunk i, so a chunk whose end differs from its start contains the
        # transition somewhere inside and its exact position is unknown.
        clen = nsamples // CHUNKS
        pts = [tag.gain_index] + list(tag.chunk_gain_index[:CHUNKS])
        for i in range(CHUNKS):
            lo = got * nsamples + i * clen
            hi = lo + clen if i < CHUNKS - 1 else (got + 1) * nsamples
            idx = pts[i + 1]
            if idx not in cache:
                g = dev.rx_gain_tag_to_gain_db(ch, idx)
                if g is None:
                    raise RuntimeError(f"gain index {idx} outside the table")
                cache[idx] = g
            gain[lo:hi] = cache[idx]
            if pts[i + 1] != pts[i]:
                ambiguous += clen
        got += 1

    info = {"packets": npkt, "restarts": gaps + tries,
            "ambiguous_samples": ambiguous,
            "distinct_gains": sorted(set(cache.values()))}
    return iq[:want], gain[:want], info


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
    ap.add_argument("--num-buffers", type=int, default=1024)
    ap.add_argument("--num-transfers", type=int, default=32)
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

    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11_META,
                    num_buffers=args.num_buffers, buffer_size=nsamples + 4,
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
    print(f"capture     : 2^{args.log2n} = {want} samples "
          f"({want/args.sample_rate*1e3:.2f} ms), {info['packets']} packets, "
          f"contiguous (restarts: {info['restarts']})")
    print(f"gain        : {', '.join(f'{g:.2f}' for g in info['distinct_gains'])} dB"
          f"{'' if calibrated else '  [UNCALIBRATED -- values carry an offset]'}")
    amb = info["ambiguous_samples"]
    print(f"              {amb} samples ({100*amb/want:.2f}%) sit in a chunk "
          f"where the gain changed, so their gain is known only to that chunk")
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
