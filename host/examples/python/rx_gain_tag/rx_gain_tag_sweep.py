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

Requires FPGA v0.17.0 or later. Buffers are sized to one message so that each
receive maps to exactly one packet and one gain profile.

Examples:

  # three frequencies, 200 ms each, two passes
  ./rx_gain_tag_sweep.py -f 731e6,915e6,2400e6 -d 0.2 -r 2

  # log to CSV, include measured IQ power so the gain correction is visible
  ./rx_gain_tag_sweep.py -f 731,915,2400 --mhz -d 0.5 --power --csv sweep.csv
"""

import argparse
import csv
import sys
import time

from bladerf import _bladerf

# The FPGA reserves 4 samples of every DMA buffer for the header, so a buffer
# sized to one message returns this many samples.
MESSAGE_SAMPLES = {"SuperSpeed": 2044, "Hi-Speed": 1020}
BUFFER_SAMPLES = {"SuperSpeed": 2048, "Hi-Speed": 1024}

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


def iq_power_dbfs(raw, count):
    """Mean power of interleaved SC16 Q11 samples, in dB relative to full scale."""
    import math
    acc = 0
    for i in range(count):
        s = raw[2 * i]
        q = raw[2 * i + 1]
        acc += s * s + q * q
    if acc == 0:
        return float("-inf")
    # 2048 is full scale for SC16 Q11
    return 10.0 * math.log10(acc / count / (2048.0 * 2048.0))


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
    ap.add_argument("--power", action="store_true",
                    help="also compute IQ power per packet and the gain-"
                         "corrected absolute power (slower, pure Python)")
    ap.add_argument("--csv", metavar="PATH", help="write a row per packet")
    ap.add_argument("--now", action="store_true",
                    help="request each packet with RX_NOW instead of reading "
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

    speed = dev.get_device_speed()
    speed_name = "SuperSpeed" if "super" in str(speed).lower() else "Hi-Speed"
    nsamples = MESSAGE_SAMPLES[speed_name]
    bufsize = BUFFER_SAMPLES[speed_name]

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
    print(f"# {len(freqs)} frequencies x {args.dwell:g} s x {args.repeat} pass"
          f"{'es' if args.repeat != 1 else ''}")
    print(f"# packet timestamps are in samples at the RX sample rate")

    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11_META,
                    num_buffers=16, buffer_size=bufsize,
                    num_transfers=8, stream_timeout=3500)
    dev.enable_module(ch, True)

    ffi = _bladerf.ffi
    meta = ffi.new("struct bladerf_metadata *")
    buf = bytearray(4 * bufsize)
    raw = ffi.cast("int16_t *", ffi.from_buffer(buf)) if args.power else None

    rows = []
    header_printed = False
    # Contiguous reads: after the first packet, ask for exactly the samples that
    # follow the previous one. RX_NOW instead returns whatever is current and
    # silently drops the rest, which leaves gaps between packets.
    next_ts = None
    resyncs = 0
    # index -> dB depends only on the index and the tuned band, and the call
    # costs a USB round trip in FPGA tuning mode, so memoise it per frequency
    # rather than paying it on every packet.
    gain_db_cache = {}
    try:
        for lap in range(args.repeat):
            for seg, freq in enumerate(freqs):
                dev.set_frequency(ch, freq)
                retune_wall = time.monotonic()
                retune_ts = None
                prev_ts = None
                pkt = 0

                while time.monotonic() - retune_wall < args.dwell:
                    meta.status = 0
                    if args.now or next_ts is None:
                        meta.flags = 0x80000000  # BLADERF_META_FLAG_RX_NOW
                        meta.timestamp = 0
                    else:
                        meta.flags = 0
                        meta.timestamp = next_ts
                    try:
                        dev.sync_rx(buf, nsamples, 3500, meta)
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
                    next_ts = int(meta.timestamp) + nsamples

                    tag = _bladerf.rx_gain_tag(meta)
                    if retune_ts is None:
                        retune_ts = meta.timestamp

                    # Samples missing between this packet and the previous one.
                    # Non-zero means the consumer fell behind, and a gain change
                    # spanning the gap cannot be pinned to a single moment.
                    gap = 0
                    if prev_ts is not None:
                        gap = int(meta.timestamp - prev_ts) - nsamples
                    prev_ts = meta.timestamp

                    gain_db = None
                    if tag is not None:
                        key = (freq, tag.gain_index)
                        if key not in gain_db_cache:
                            gain_db_cache[key] = dev.rx_gain_tag_to_gain_db(
                                ch, tag.gain_index)
                        gain_db = gain_db_cache[key]

                    dbfs = dbm = None
                    if args.power:
                        dbfs = iq_power_dbfs(raw, meta.actual_count)
                        if gain_db is not None:
                            dbm = dbfs - gain_db

                    row = {
                        "lap": lap,
                        "seg": seg,
                        "freq_hz": freq,
                        "pkt": pkt,
                        "timestamp": int(meta.timestamp),
                        "ts_since_retune": int(meta.timestamp - retune_ts),
                        "wall_s": round(time.monotonic() - retune_wall, 6),
                        "count": int(meta.actual_count),
                        "gap_samples": gap,
                        "status": int(meta.status),
                        "gain_index": tag.gain_index if tag else None,
                        "gain_db": round(gain_db, 2) if gain_db is not None else None,
                        "idx_min": tag.gain_index_min if tag else None,
                        "idx_max": tag.gain_index_max if tag else None,
                        "chunks": ",".join(str(c) for c in tag.chunk_gain_index)
                                  if tag else None,
                        "changed": int(tag.changed) if tag else None,
                        "locked": int(tag.locked) if tag else None,
                        "msgs": tag.num_messages if tag else None,
                        "dbfs": round(dbfs, 2) if dbfs is not None else None,
                        "dbm": round(dbm, 2) if dbm is not None else None,
                    }
                    rows.append(row)

                    if not args.quiet:
                        if not header_printed:
                            hdr = (f"{'freq_MHz':>9} {'pkt':>4} {'timestamp':>12} "
                                   f"{'dts':>8} {'gap':>8} {'idx':>4} "
                                   f"{'gain_dB':>8} {'chunk profile':>17} "
                                   f"{'flg':>4}")
                            if args.power:
                                hdr += f" {'dBFS':>7} {'dBm':>8}"
                            print(hdr)
                            header_printed = True
                        flg = ("C" if tag and tag.changed else "-") + \
                              ("L" if tag and tag.locked else "-")
                        line = (f"{freq/1e6:9.3f} {pkt:4d} {row['timestamp']:12d} "
                                f"{row['ts_since_retune']:8d} {gap:8d} "
                                f"{row['gain_index'] if tag else -1:4d} "
                                f"{row['gain_db'] if gain_db is not None else float('nan'):8.2f} "
                                f"{row['chunks'] or '':>17} {flg:>4}")
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
            sub = [r for r in rows if r["lap"] == lap and r["seg"] == seg
                   and r["gain_index"] is not None]
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
    print(f"\n{len(rows)} packets, {overruns} with overrun status, "
          f"{untagged} without a gain tag, {dropped} samples dropped, "
          f"{resyncs} resyncs")
    if dropped:
        print("note: samples were dropped, so a gain change spanning a gap "
              "cannot be pinned to one moment. Lower --sample-rate to keep up.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
