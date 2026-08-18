#!/usr/bin/env python3
"""Measure the RSSI of LTE cells, tuning each to its own center frequency.

For every cell given as CENTER:CHANNEL_BW (MHz), retunes to that center, captures
contiguous IQ, applies the per-chunk RFIC gain from the packet headers, and
reports the absolute power in the channel bandwidth and in the occupied
bandwidth.

Tuning per cell rather than capturing several at once matters for accuracy:

  * the cell sits at baseband DC, in the flattest part of the RX response, rather
    than out near the filter edge;
  * the gain calibration table is one scalar per frequency, so evaluating it at
    the cell's own center is the correct thing to do;
  * a 10 MHz cell needs only +-5 MHz of the +-10 MHz Nyquist band at 20 Msps.

The noise floor is estimated as a low percentile of the per-bin power outside the
channel, which avoids being biased by adjacent carriers -- a plain mean over the
out-of-channel region would include them. The noise-subtracted column removes
n_bins * that floor.

Occupied bandwidth follows the LTE resource-block count (RB * 180 kHz).

Example:
    ./rx_lte_rssi.py --cells 723:10,731.5:5,739:10,751:10,763:10 \
        --gain-cal auto -s 20e6 -b 20e6
"""

import argparse
import os
import re
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rx_psd_dbm as psd                                   # noqa: E402
from bladerf import _bladerf                               # noqa: E402

# LTE channel bandwidth (MHz) -> occupied bandwidth (MHz), from the RB count
OCCUPIED = {1.4: 1.08, 3.0: 2.7, 5.0: 4.5, 10.0: 9.0, 15.0: 13.5, 20.0: 18.0}


def parse_cells(text):
    out = []
    for tok in text.replace(" ", "").split(","):
        if not tok:
            continue
        c, b = tok.split(":")
        c, b = float(c), float(b)
        if b not in OCCUPIED:
            raise SystemExit(f"unknown LTE channel bandwidth {b} MHz; "
                             f"known: {sorted(OCCUPIED)}")
        out.append((c * 1e6, b))
    if not out:
        raise SystemExit("no cells given")
    return out


def measure(dev, ch, fc_hz, chan_bw_mhz, args, nsamples):
    dev.set_frequency(ch, int(fc_hz))
    dev.set_gain_mode(ch, psd.GAIN_MODES[args.gain_mode])
    want = 2 ** args.log2n
    iq, gain_db, info = psd.capture(dev, ch, nsamples, want, args)

    y = iq * (10.0 ** (-gain_db / 20.0)).astype(np.float32)
    (freqs, p_lin), nseg = psd.welch_dbm(y, args.nperseg, args.overlap,
                                         args.sample_rate)
    fm = freqs / 1e6
    occ_bw = OCCUPIED[chan_bw_mhz]

    def power(bw):
        m = (fm >= -bw / 2.0) & (fm <= bw / 2.0)
        return float(np.sum(p_lin[m])), int(m.sum())

    # Noise floor: low percentile of out-of-channel bins, so an adjacent carrier
    # in the guard region does not inflate it.
    guard = (np.abs(fm) > chan_bw_mhz / 2.0 + 0.5) & (np.abs(fm) < 9.0)
    noise_per_bin = (float(np.percentile(p_lin[guard], 10))
                     if guard.sum() > 20 else float("nan"))

    res = {"fc": fc_hz, "chan_bw": chan_bw_mhz, "occ_bw": occ_bw,
           "nseg": nseg, "gains": info["distinct_gains"],
           "ambiguous": info["ambiguous_samples"], "want": want,
           "restarts": info["restarts"],
           "total": float(np.sum(p_lin)), "noise": noise_per_bin,
           "freqs": freqs, "p_lin": p_lin}
    for label, bw in (("chan", chan_bw_mhz), ("occ", occ_bw)):
        tot, nb = power(bw)
        res[label] = {"bw": bw, "total": tot, "bins": nb,
                      "per_bin": tot / nb,
                      "sub": max(tot - noise_per_bin * nb, 1e-300)}
    return res


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cells", required=True,
                    help="comma-separated CENTER_MHz:CHANNEL_BW_MHz, e.g. "
                         "723:10,731.5:5,739:10")
    ap.add_argument("-n", "--log2n", type=int, default=18)
    ap.add_argument("-s", "--sample-rate", type=float, default=20e6)
    ap.add_argument("-b", "--bandwidth", type=float, default=20e6)
    ap.add_argument("-c", "--channel", type=int, default=0)
    ap.add_argument("-g", "--gain-mode", choices=sorted(psd.GAIN_MODES),
                    default="slow")
    ap.add_argument("--nperseg", type=int, default=2048)
    ap.add_argument("--overlap", type=float, default=0.5)
    ap.add_argument("--gain-cal", metavar="PATH")
    ap.add_argument("--settle", type=float, default=0.3)
    ap.add_argument("--num-buffers", type=int, default=1024)
    ap.add_argument("--num-transfers", type=int, default=64)
    ap.add_argument("--repeat", type=int, default=1,
                    help="measure each cell this many times (default 1)")
    ap.add_argument("--plot", metavar="PATH",
                    help="write a figure with one PSD panel per cell")
    args = ap.parse_args()

    cells = parse_cells(args.cells)
    ch = _bladerf.CHANNEL_RX(args.channel)
    dev = _bladerf.BladeRF()

    fpga = dev.get_fpga_version()
    if (fpga.major, fpga.minor) < (0, 17):
        print(f"FPGA {fpga} lacks the RX gain tag; v0.17.0+ required.",
              file=sys.stderr)
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
    dev.set_bandwidth(ch, int(args.bandwidth))
    speed = dev.get_device_speed()
    nsamples = 2044 if "super" in str(speed).lower() else 1020

    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11_META,
                    num_buffers=args.num_buffers, buffer_size=nsamples + 4,
                    num_transfers=args.num_transfers, stream_timeout=3500)
    dev.enable_module(ch, True)

    print(f"# FPGA {fpga}, {args.sample_rate/1e6:.3f} Msps, "
          f"{args.bandwidth/1e6:.1f} MHz BW, {args.gain_mode} AGC, "
          f"2^{args.log2n} samples/measurement")
    print(f"# bin = {args.sample_rate/args.nperseg/1e3:.3f} kHz"
          f"{'' if calibrated else '   [UNCALIBRATED: values carry an offset]'}")
    print()
    hdr = (f"{'cell':>10} {'BW':>6} {'span MHz':>17} {'bins':>5} {'RSSI dBm':>9} "
           f"{'dBm/bin':>9} {'SNR/bin':>8} {'noise-sub':>10} {'gain dB':>16}")
    print(hdr)
    print("-" * len(hdr))

    results = []
    try:
        for fc, bw in cells:
            for rep in range(args.repeat):
                r = measure(dev, ch, fc, bw, args, nsamples)
                results.append(r)
                for key in ("chan", "occ"):
                    e = r[key]
                    lo, hi = fc / 1e6 - e["bw"] / 2, fc / 1e6 + e["bw"] / 2
                    snr = 10 * np.log10(e["per_bin"] / r["noise"])
                    gtxt = (f"{min(r['gains']):.2f}"
                            if len(r["gains"]) == 1
                            else f"{min(r['gains']):.2f}-{max(r['gains']):.2f}")
                    print(f"{fc/1e6:>10.2f} {e['bw']:>5.1f}M "
                          f"{lo:>8.2f}..{hi:<7.2f} {e['bins']:>5d} "
                          f"{10*np.log10(e['total']):>+9.2f} "
                          f"{10*np.log10(e['per_bin']):>+9.2f} {snr:>+8.2f} "
                          f"{10*np.log10(e['sub']):>+10.2f} "
                          f"{gtxt:>16}")
                print()
    finally:
        dev.enable_module(ch, False)

    print("RSSI = 10*log10(sum of linear bin powers) over the span.")
    print("noise floor = 10th percentile of out-of-channel bins; noise-sub "
          "removes bins * that floor.")

    if args.plot and results:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        n = len(results)
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.6 * n), squeeze=False)
        for ax, r in zip(axes[:, 0], results):
            fm = r["freqs"] / 1e6
            ax.plot(fm, 10 * np.log10(r["p_lin"] + 1e-300), lw=0.5)
            for bw, col in ((r["chan_bw"], "red"), (r["occ_bw"], "green")):
                ax.axvspan(-bw / 2, bw / 2, color=col, alpha=0.08)
                for e in (-bw / 2, bw / 2):
                    ax.axvline(e, color=col, ls="--", lw=0.7)
            ax.axhline(10 * np.log10(r["noise"]), color="grey", ls=":", lw=0.8)
            ax.set_title(f"{r['fc']/1e6:.2f} MHz, {r['chan_bw']:.0f} MHz cell: "
                         f"chan {10*np.log10(r['chan']['total']):+.2f} dBm, "
                         f"occ {10*np.log10(r['occ']['total']):+.2f} dBm",
                         fontsize=9)
            ax.set_ylabel("dBm/bin", fontsize=8)
            ax.grid(alpha=0.3)
        axes[-1, 0].set_xlabel("baseband offset (MHz)")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"\nwrote {args.plot}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
