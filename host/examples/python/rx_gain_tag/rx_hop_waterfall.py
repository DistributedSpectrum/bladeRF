#!/usr/bin/env python3
"""Capture IQ per frequency step and render a per-packet waterfall.

Hops through a list of center frequencies, writes the IQ of each step to its own
file, and produces a waterfall with one PSD row per USB packet so the spectrum
can be watched changing across a retune.

Reads cover several packets at a time (--read-packets), and
bladerf_get_rx_gain_tags() then hands back one entry per packet -- its timestamp,
the slice of the read it occupies, and its own four-chunk gain profile. So the
rows, the PSD and the .iq files stay per packet while the loop makes far fewer
calls, which is what lets it keep up at 20 Msps and above.

The reason this exists as a separate tool from rx_gain_tag_sweep.py is a trap
worth understanding. Reading contiguously means asking for the samples that
follow the previous read, so the reader necessarily runs *behind* real time --
by up to num_buffers * buffer_size / sample_rate, which is 105 ms at the
defaults. bladerf_set_frequency() takes effect in real time. So for a while after
a retune, the packets arriving still contain samples captured at the *previous*
frequency, and anything that labels them by "the frequency I most recently asked
for" is simply wrong. Measured on a 951 -> 731 MHz hop at 20 Msps, the first 119
packets after the retune were still 951 MHz data.

This tool therefore records the RX sample counter at each retune and attributes
every packet by comparing its timestamp against those boundaries, so a step's
file holds only samples actually captured at that frequency. Packets that
straddle a boundary contain both and are flagged rather than filed.

Outputs, under --outdir:
    step<NN>_lap<L>_<freq>MHz.iq   interleaved int16 SC16 Q11, little-endian
    packets.csv                    one row per packet, with both the naive label
                                   and the timestamp-derived truth
    waterfall.png                  per-packet PSD, retune boundaries marked

Example:
    ./rx_hop_waterfall.py -f 951,731 --mhz -d 0.15 -s 20e6 \
        --gain-cal auto -o /tmp/hop
"""

import argparse
import bisect
import csv
import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gain_profile as gp                # noqa: E402
from bladerf import _bladerf             # noqa: E402

FULL_SCALE = gp.FULL_SCALE               # SC16 Q11

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
        if tok:
            v = float(tok)
            out.append(int(v * 1e6) if mhz else int(v))
    if not out:
        raise SystemExit("no frequencies given")
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-f", "--freqs", required=True)
    ap.add_argument("--mhz", action="store_true")
    ap.add_argument("-d", "--dwell", type=float, default=0.15)
    ap.add_argument("-r", "--repeat", type=int, default=1)
    ap.add_argument("-s", "--sample-rate", type=float, default=20e6)
    ap.add_argument("-b", "--bandwidth", type=float, default=None)
    ap.add_argument("-c", "--channel", type=int, default=0)
    ap.add_argument("-g", "--gain-mode", choices=sorted(GAIN_MODES),
                    default="slow")
    ap.add_argument("--gain-cal", metavar="PATH",
                    help="RX gain calibration table (.tbl or .csv), or 'auto'")
    ap.add_argument("--read-packets", type=int, default=2, metavar="N",
                    help="packets per sync_rx() call (default 2). Whole-packet "
                         "reads keep every gain tag entry a complete packet, so "
                         "rows, PSDs and .iq files stay per packet")
    ap.add_argument("--buffer-size", type=int, default=8192, metavar="SAMPLES",
                    help="samples per USB buffer, rounded up to whole packets "
                         "(default 8192 = 4 packets on SuperSpeed)")
    ap.add_argument("--num-buffers", type=int, default=256)
    ap.add_argument("--num-transfers", type=int, default=32)
    ap.add_argument("-o", "--outdir", default="hop_capture")
    ap.add_argument("--fft", type=int, default=2048,
                    help="FFT size per packet (default 2048, matching the USB "
                         "buffer; a packet carries 4 fewer samples than that "
                         "because the header replaces them, so the tail is "
                         "zero-padded)")
    ap.add_argument("--max-packets", type=int, default=400,
                    help="cap packets stored per step (default 400) to bound "
                         "memory and disk")
    ap.add_argument("--psd-every", type=int, default=1, metavar="N",
                    help="keep only every Nth packet's PSD in the waterfall "
                         "(default 1). Each row is fft*4 bytes, so a long hop "
                         "list needs this: 46 steps at 20 Msps is ~58k rows, "
                         "about 475 MB at the default FFT size")
    ap.add_argument("--no-iq", action="store_true",
                    help="compute the waterfall but do not write .iq files")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    freqs = parse_freqs(args.freqs, args.mhz)
    bandwidth = args.bandwidth or 0.8 * args.sample_rate
    ch = _bladerf.CHANNEL_RX(args.channel)
    os.makedirs(args.outdir, exist_ok=True)

    dev = _bladerf.BladeRF()

    fpga = dev.get_fpga_version()
    if (fpga.major, fpga.minor) < (0, 17):
        print(f"FPGA {fpga} does not tag RX packets with the RFIC gain; "
              f"v0.17.0 or later required.", file=sys.stderr)
        return 2

    if args.gain_cal:
        path = args.gain_cal
        if path == "auto":
            path = f"{dev.get_serial()}_rx_gain_cal.tbl"
        else:
            m = re.search(r"([0-9a-fA-F]{32})", os.path.basename(path))
            if m and m.group(1).lower() != dev.get_serial().lower():
                print(f"WARNING: {os.path.basename(path)} was swept on "
                      f"{m.group(1)}, this device is {dev.get_serial()}. "
                      f"Absolute power will be wrong.", file=sys.stderr)
        try:
            dev.set_gain_calibration(ch, path)
        except Exception as exc:
            print(f"failed to load gain calibration {path}: {exc}",
                  file=sys.stderr)
            return 2

    speed_name = gp.speed_name(dev)
    nsamples = gp.MESSAGE_SAMPLES[speed_name]
    read_n = gp.read_samples(nsamples, args.read_packets)

    dev.set_sample_rate(ch, int(args.sample_rate))
    dev.set_bandwidth(ch, int(bandwidth))
    dev.set_frequency(ch, freqs[0])
    dev.set_gain_mode(ch, GAIN_MODES[args.gain_mode])

    lag_ms = args.num_buffers * args.buffer_size / args.sample_rate * 1e3
    print(f"# FPGA {fpga}, {speed_name}: {nsamples} samples/packet, "
          f"{args.sample_rate/1e6:.3f} Msps, gain mode {args.gain_mode}")
    print(f"# reading {args.read_packets} packet(s) = {read_n} samples per call, "
          f"one gain profile per packet")
    print(f"# {args.num_buffers} buffers x {args.buffer_size} samples => the "
          f"reader may lag real time by up to {lag_ms:.1f} ms, so that much "
          f"post-retune data can still be from the previous step")

    if args.num_transfers >= args.num_buffers:
        print("--num-transfers must be less than --num-buffers", file=sys.stderr)
        return 2

    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11_META,
                    num_buffers=args.num_buffers, buffer_size=args.buffer_size,
                    num_transfers=args.num_transfers, stream_timeout=3500)
    # Everything captured before the first in-loop retune was taken at freqs[0],
    # which was tuned above. Seed a boundary at sample 0 so those packets are
    # attributed correctly instead of looking mislabeled.
    dev.enable_module(ch, True)

    ffi = _bladerf.ffi
    meta = ffi.new("struct bladerf_metadata *")
    buf = bytearray(4 * read_n)

    # One entry per retune: (ts_before, ts_after, freq, lap, seg).
    #
    # Bracketing matters. bladerf_set_frequency() moves the AD9361 LO early in
    # its sequence and then spends milliseconds on band selection, port switching
    # and recalibration, so a timestamp read *after* it returns lands well past
    # the actual RF change -- measured 8 ms late at 20 Msps. Reading before and
    # after gives a window that certainly contains the change, and packets
    # overlapping that window are reported as uncertain rather than guessed at.
    bounds = [(0, 0, freqs[0], 0, 0)]
    files = {}
    rows = []
    psd_rows = []          # one PSD per kept packet, in arrival order
    psd_index = []         # packet seq for each kept PSD row
    psd_marks = []         # (psd row, retune sample, freq, lap, seg)
    win = np.hanning(nsamples).astype(np.float32)
    win_norm = float(np.sum(win ** 2))
    # index -> dB is band dependent and costs a USB round trip per miss in FPGA
    # tuning mode, so it is memoised per (frequency, index).
    gains = gp.GainDb(dev, ch)
    next_ts = None
    stored = {}
    partial = 0

    def attribute(ts, n):
        """What was tuned while samples [ts, ts+n) were captured.

        Returns (freq, lap, seg, certain). `certain` is False when the packet
        overlaps a retune window, so it contains a mix of two frequencies or was
        captured while the RFIC was mid-retune.
        """
        # last retune whose window starts before this packet ends
        j = 0
        for k in range(len(bounds) - 1, -1, -1):
            if bounds[k][0] < ts + n:
                j = k
                break
        b_before, b_after, freq, lap, seg = bounds[j]
        if ts >= b_after:
            return freq, lap, seg, True          # entirely after the retune
        if ts + n <= b_before:
            prev = bounds[j - 1] if j > 0 else bounds[0]
            return prev[2], prev[3], prev[4], True   # entirely before it
        return freq, lap, seg, False             # overlaps the retune window

    try:
        for lap in range(args.repeat):
            for seg, freq in enumerate(freqs):
                ts_before = dev.get_timestamp(_bladerf.Direction.RX)
                dev.set_frequency(ch, freq)
                ts_after = dev.get_timestamp(_bladerf.Direction.RX)
                bounds.append((ts_before, ts_after, freq, lap, seg))
                psd_marks.append((len(psd_rows), len(rows), ts_before, ts_after,
                                  freq, lap, seg))
                t0 = time.monotonic()

                while time.monotonic() - t0 < args.dwell:
                    if next_ts is None:
                        meta.flags = 0x80000000
                        meta.timestamp = 0
                    else:
                        meta.flags = 0
                        meta.timestamp = next_ts
                    meta.status = 0
                    try:
                        dev.sync_rx(buf, read_n, 3500, meta)
                    except _bladerf.TimePastError:
                        next_ts = None
                        continue
                    except Exception as exc:
                        print(f"  sync_rx: {exc}", file=sys.stderr)
                        break
                    # actual_count, not read_n: a discontinuity ends the read
                    # early and the next request has to follow what arrived.
                    next_ts = int(meta.timestamp) + int(meta.actual_count)

                    tags = dev.rx_gain_tags()
                    if tags is None:
                        print("  no gain tag: needs FPGA v0.17.0+ and a "
                              "metadata RX format", file=sys.stderr)
                        break
                    v = np.frombuffer(buf, dtype=np.int16,
                                      count=2 * int(meta.actual_count))

                    for t in tags:
                        n = int(t.sample_count)
                        ts = int(t.timestamp) + t.msg_sample_offset
                        seg = v[2 * t.sample_offset:2 * (t.sample_offset + n)]

                        f32 = seg.astype(np.float32)
                        iq = (f32[0::2] + 1j * f32[1::2]) / FULL_SCALE

                        true_freq, true_lap, true_seg, certain = attribute(ts, n)
                        gdb = gains.at(true_freq)
                        gain_db = gdb(t.gain_index)
                        # Per-chunk correction, so a packet the AGC moved inside
                        # is still right; a single gain would not be.
                        dbfs, dbm = gp.packet_power(v, t, nsamples, gdb)

                        # PSD, one row per packet, FFT sized to the USB buffer.
                        # A short entry cannot use the packet-length window, and
                        # only appears after a truncated read, so skip it.
                        if len(rows) % args.psd_every == 0:
                            if n == nsamples:
                                spec = np.fft.fftshift(
                                    np.fft.fft(iq * win, n=args.fft))
                                psd = 10.0 * np.log10(
                                    np.abs(spec) ** 2 / win_norm + 1e-20)
                                psd_rows.append(psd.astype(np.float32))
                                psd_index.append(len(rows))
                            else:
                                partial += 1

                        rows.append({
                            "seq": len(rows),
                            "timestamp": ts,
                            "label_freq_hz": freq,     # naive: last retune asked for
                            "true_freq_hz": true_freq, # from the timestamp
                            "mislabeled": int(true_freq != freq),
                            "certain": int(certain),
                            "lap": true_lap,
                            "seg": true_seg,
                            "count": n,
                            "gain_index": t.gain_index,
                            "gain_db": round(gain_db, 2),
                            "chunks": ",".join(str(c) for c in
                                               t.chunk_gain_index[:4]),
                            "dbfs": round(dbfs, 2),
                            "dbm": round(dbm, 2),
                            "changed": int(gp.profile(t)[2]),
                            "carried": int(t.carried),
                        })

                        # file the IQ under the frequency it was really captured at
                        if not args.no_iq and certain:
                            k = (true_lap, true_seg)
                            if stored.get(k, 0) < args.max_packets:
                                if k not in files:
                                    name = (f"step{true_seg:02d}_lap{true_lap}_"
                                            f"{true_freq/1e6:.3f}MHz.iq")
                                    files[k] = open(
                                        os.path.join(args.outdir, name), "wb")
                                files[k].write(seg.tobytes())
                                stored[k] = stored.get(k, 0) + 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        dev.enable_module(ch, False)
        for fh in files.values():
            fh.close()

    if not rows:
        print("no packets captured", file=sys.stderr)
        return 1

    csv_path = os.path.join(args.outdir, "packets.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    mis = sum(r["mislabeled"] for r in rows)
    strad = sum(1 for r in rows if not r["certain"])
    print(f"\n{len(rows)} packets -> {csv_path}")
    print(f"{len(files)} IQ files in {args.outdir}")
    print(f"{mis} packets ({100*mis/len(rows):.1f}%) carried samples from a "
          f"different frequency than the naive label")
    print(f"{strad} packets overlapped a retune window (frequency uncertain, "
          f"not written to any .iq file)")
    if partial:
        print(f"{partial} packets were truncated by a discontinuity and have no "
              f"PSD row")

    # How long each retune kept delivering the *previous* step's samples. Keyed
    # by the boundary itself so laps stay separate.
    print(f"\n{'lap':>4} {'seg':>4} {'asked for':>11} {'stale pkts':>11} "
          f"{'stale ms':>9} {'really was':>11}")
    for (row_i, seq_i, tsb, tsa, freq, lap, seg) in psd_marks:
        stale = [r for r in rows[seq_i:]
                 if r["label_freq_hz"] == freq and r["mislabeled"] == 1]
        if not stale:
            continue
        # contiguous run of stale packets immediately after this retune
        run = 0
        for r in rows[seq_i:]:
            if r["mislabeled"] and r["label_freq_hz"] == freq:
                run += 1
            else:
                break
        was = stale[0]["true_freq_hz"] / 1e6 if stale else 0
        print(f"{lap:>4} {seg:>4} {freq/1e6:>11.3f} {run:>11} "
              f"{run*nsamples/args.sample_rate*1e3:>9.1f} {was:>11.3f}")

    if not args.no_plot:
        make_plot(args, rows, psd_rows, psd_marks, nsamples)
    return 0


def make_plot(args, rows, psd_rows, psd_marks, nsamples):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    wf = np.vstack(psd_rows)
    nrow, nbin = wf.shape
    half = args.sample_rate / 2e6

    fig, (ax0, ax1) = plt.subplots(
        2, 1, figsize=(11, 12), sharex=False,
        gridspec_kw={"height_ratios": [3, 1]})

    vmax = float(np.percentile(wf, 99.5))
    vmin = vmax - 70
    im = ax0.imshow(wf, aspect="auto", origin="lower", cmap="viridis",
                    vmin=vmin, vmax=vmax,
                    extent=[-half, half, 0, nrow])
    ax0.set_ylabel(f"PSD row (time ->)"
                   f"{'' if args.psd_every == 1 else f', every {args.psd_every} pkts'}")
    ax0.set_xlabel("baseband offset (MHz)")
    ax0.set_title(f"per-packet PSD, {args.fft}-point FFT of each "
                  f"{nsamples}-sample USB packet\n"
                  f"dashed = retune issued; the spectrum changes later, by the "
                  f"reader's lag")
    fig.colorbar(im, ax=ax0, label="dB (uncal)")

    for (row_i, seq_i, tsb, tsa, freq, lap, seg) in psd_marks:
        ax0.axhline(row_i, color="red", ls="--", lw=0.8)
        ax0.text(-half * 0.98, row_i, f" {freq/1e6:.0f}", color="red",
                 va="bottom", fontsize=7)

    seq = [r["seq"] for r in rows]
    ax1.plot(seq, [r["dbfs"] for r in rows], lw=0.8, label="dBFS")
    dbm = [r["dbm"] if r["dbm"] is not None else np.nan for r in rows]
    ax1.plot(seq, dbm, lw=0.8, label="dBm (gain corrected)")
    ax1.set_xlabel("packet")
    ax1.set_ylabel("power")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8)

    mis = [r["seq"] for r in rows if r["mislabeled"]]
    if mis:
        ax1.plot(mis, [min(dbm) if dbm else 0] * len(mis), "|", color="red",
                 ms=4, label="mislabeled")
        ax1.legend(loc="upper right", fontsize=8)
    for (row_i, seq_i, tsb, tsa, freq, lap, seg) in psd_marks:
        ax1.axvline(seq_i, color="red", ls="--", lw=0.8)

    out = os.path.join(args.outdir, "waterfall.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nwrote {out}  ({nrow} packets x {nbin} bins)")


if __name__ == "__main__":
    sys.exit(main())
