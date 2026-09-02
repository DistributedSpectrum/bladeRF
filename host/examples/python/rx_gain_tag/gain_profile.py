#!/usr/bin/env python3
"""Reconstruct a per-sample RFIC gain from the RX gain tags of a multi-packet read.

Every script in this directory needs the same thing: a read of arbitrary size, and
then the gain that applied to each sample of it. bladerf_get_rx_gain_tags()
(dev.rx_gain_tags()) supplies one entry per packet the read consumed, and this
module turns those entries into spans of constant gain in *buffer* coordinates,
which is what the callers actually want to index with.

Two coordinate systems meet here, which is the only subtle part:

  * Chunk boundaries are fixed in the packet payload -- chunk c covers payload
    samples [c*L, (c+1)*L) where L = samples_per_packet / CHUNKS.
  * sample_offset / sample_count are positions in the caller's buffer.

They differ by `msg_sample_offset`, the payload offset of the first sample the
entry describes, which is nonzero only when a read began part way into a packet.
Requesting a whole multiple of samples_per_packet avoids that case entirely, and
read_samples() below is the way to ask for one.

chunk_spans() also applies the midpoint split: a chunk whose end index differs
from its start contains a transition at an unknown position, so the first half
takes the gain the chunk started at and the second half the gain it ended at. For
a transition uniformly distributed in the chunk that halves the expected
mis-assigned fraction, from 0.50 of the chunk to 0.25.
"""

import collections
import math

# A packet's payload is the DMA buffer minus the 16-byte (4-sample) header.
MESSAGE_SAMPLES = {"SuperSpeed": 2044, "Hi-Speed": 1020}
CHUNKS = 4
FULL_SCALE = 2048.0                     # SC16 Q11: 12 significant bits

try:
    import numpy as _np
except ImportError:                     # every consumer but the sweep needs it
    _np = None

Span = collections.namedtuple("Span", "lo hi index split")


def speed_name(dev):
    """'SuperSpeed' or 'Hi-Speed' for a connected device."""
    return ("SuperSpeed" if "super" in str(dev.get_device_speed()).lower()
            else "Hi-Speed")


def message_samples(dev):
    """Samples of payload in one RX packet, for this device's USB speed."""
    return MESSAGE_SAMPLES[speed_name(dev)]


def read_samples(msg_samples, packets):
    """Request size covering `packets` whole packets.

    Reading a whole multiple of the payload keeps every read aligned to a packet
    boundary, so each gain tag entry describes a complete packet and no entry is
    ever flagged `carried`. Reading a non-multiple works too -- chunk_spans()
    handles the partial entries -- but the per-packet bookkeeping in these
    examples is simpler when packets and reads line up.
    """
    return max(1, int(packets)) * msg_samples


def chunk_spans(tag, msg_samples, chunks=CHUNKS):
    """Spans of constant gain covering the samples one tag entry describes.

    Yields Span(lo, hi, index, split) in buffer coordinates, in order, with the
    midpoint split already applied. `split` marks a span that came from a chunk
    containing a transition, i.e. one whose gain is known only to chunk
    resolution.
    """
    clen = msg_samples // chunks
    pts = [tag.gain_index] + list(tag.chunk_gain_index[:chunks])
    end = tag.msg_sample_offset + tag.sample_count

    for c in range(chunks):
        # chunk c in payload coordinates, clipped to what this entry covers
        lo = max(c * clen, tag.msg_sample_offset)
        hi = min((c + 1) * clen if c < chunks - 1 else msg_samples, end)
        if hi <= lo:
            continue

        # payload -> buffer
        blo = tag.sample_offset + (lo - tag.msg_sample_offset)
        bhi = tag.sample_offset + (hi - tag.msg_sample_offset)

        if pts[c] == pts[c + 1]:
            yield Span(blo, bhi, pts[c + 1], False)
        else:
            mid = blo + (bhi - blo) // 2
            yield Span(blo, mid, pts[c], True)
            yield Span(mid, bhi, pts[c + 1], True)


def gain_db_array(tags, msg_samples, count, gdb, chunks=CHUNKS):
    """Per-sample gain in dB over a whole read.

    `gdb` maps a gain index to dB (see GainDb). Returns
    (array, samples_filled, ambiguous_samples); entries tile the returned
    samples, so samples_filled == count unless the tag array was truncated.
    """
    if _np is None:
        raise RuntimeError("numpy is required for gain_db_array()")

    out = _np.zeros(count, dtype=_np.float32)
    filled = 0
    ambiguous = 0
    for t in tags:
        for s in chunk_spans(t, msg_samples, chunks):
            out[s.lo:s.hi] = gdb(s.index)
            filled += s.hi - s.lo
            if s.split:
                ambiguous += s.hi - s.lo
    return out, filled, ambiguous


def samples_view(buf, count):
    """int16 view of one read's interleaved samples, for packet_power().

    Matched to whichever path packet_power() will take, so the two agree about
    what `v` is: a numpy array when numpy is present, a cffi int16_t* otherwise.
    """
    if _np is not None:
        return _np.frombuffer(buf, dtype=_np.int16, count=2 * count)
    from bladerf import _bladerf
    return _bladerf.ffi.cast("int16_t *", _bladerf.ffi.from_buffer(buf))


def packet_power(v, tag, msg_samples, gdb=None, chunks=CHUNKS):
    """Power of the samples one tag entry describes: (dbfs, dbm).

    `v` is the read's interleaved int16 samples. dbm is None when `gdb` is None;
    otherwise each span is divided by its own linear power gain before averaging,
    which is the whole point of the per-chunk profile -- a single gain for the
    packet would be wrong exactly when the AGC moved inside it.
    """
    acc_fs = 0.0        # sum of |iq/2048|^2
    acc_mw = 0.0        # sum of |iq/2048|^2 / 10**(g/10)
    n = 0

    for s in chunk_spans(tag, msg_samples, chunks):
        if _np is not None:
            seg = v[2 * s.lo:2 * s.hi].astype(_np.int64)
            ssq = float(_np.dot(seg, seg))
        else:
            ssq = 0.0
            for i in range(2 * s.lo, 2 * s.hi):
                ssq += float(v[i]) * float(v[i])
        ssq /= FULL_SCALE * FULL_SCALE
        acc_fs += ssq
        if gdb is not None:
            acc_mw += ssq / (10.0 ** (gdb(s.index) / 10.0))
        n += s.hi - s.lo

    if n == 0 or acc_fs == 0.0:
        return float("-inf"), (float("-inf") if gdb is not None else None)

    dbfs = 10.0 * math.log10(acc_fs / n)
    dbm = 10.0 * math.log10(acc_mw / n) if gdb is not None else None
    return dbfs, dbm


def profile(tag, chunks=CHUNKS):
    """Per-packet equivalents of the summary fields, for one tag entry.

    Returns (idx_min, idx_max, changed). The metadata summary reports these over
    a whole call; per packet they come straight from the profile, with `changed`
    True when any chunk differs from the packet's base index.
    """
    pts = [tag.gain_index] + list(tag.chunk_gain_index[:chunks])
    return min(pts), max(pts), any(p != pts[0] for p in pts[1:])


class GainDb:
    """Memoised gain index -> dB.

    bladerf_rx_gain_tag_to_gain_db() depends on the tuned LO through the gain
    table band, the front-end offset and the calibration table, so a cache must be
    keyed on (frequency, index) and not on the index alone. In
    BLADERF_TUNING_MODE_FPGA each miss also costs a USB round trip to read the LO,
    while a capture only ever contains a handful of distinct indices -- so caching
    turns one call per chunk per packet into a few calls per frequency.
    """

    def __init__(self, dev, ch):
        self.dev = dev
        self.ch = ch
        self.freq = None
        self.cache = {}

    def at(self, freq_hz):
        """A gdb(index) callable for samples captured at `freq_hz`."""
        self.freq = freq_hz
        return self.__call__

    def __call__(self, index):
        key = (self.freq, index)
        if key not in self.cache:
            g = self.dev.rx_gain_tag_to_gain_db(self.ch, index)
            if g is None:
                raise RuntimeError(
                    f"gain index {index} is outside the RX gain table. Is RFIC "
                    f"register 0x035 still 0x16?")
            self.cache[key] = g
        return self.cache[key]

    def values(self):
        """Every dB value resolved so far."""
        return sorted(set(self.cache.values()))
