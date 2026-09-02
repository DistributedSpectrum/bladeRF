# RX gain tagging

FPGA v0.17.0 and later stamp the AD9361's live RX gain into every RX metadata
packet, so the gain that was applied to a block of IQ arrives *with* that IQ
instead of having to be polled asynchronously with `bladerf_get_gain()`. That is
what makes an absolute power measurement possible while the AGC is running:
subtract the tagged gain from a measured IQ power and you have dBm.

Requires an AGC gain mode and RX1. The RFIC's `CTRL_OUT` pins are pointed at row
`0x16` (UG-570 Table 44), which presents the CH1 full gain-table index in real
time; the FPGA samples them and writes them into the header.

---

## 1. The header field

RX metadata headers are 16 bytes. The first four are a reserved word that the
host never read in `SC16_Q11_META` mode — the FPGA used to fill it with the
constant `0x12344321`. It now carries the gain profile:

```
 31       25 24  23        18 17        12 11         6 5          0
+-----------+---+------------+------------+------------+------------+
| base      | L |  delta 0   |  delta 1   |  delta 2   |  delta 3   |
| 7 bits    |1b |  6b signed |  6b signed |  6b signed |  6b signed |
+-----------+---+------------+------------+------------+------------+
```

| field | meaning |
|---|---|
| `base` | full gain-table index at the **first sample** of the packet, 0…76 |
| `L` | AGC gain lock. **Fast-attack AGC only** — the AD9361 drives it from the fast-attack state machine, so it always reads 0 in slow-attack and hybrid |
| `delta i` | index at the **end of chunk i**, minus `base`. Clamps at −32/+31 rather than wrapping |

`base` needs 7 bits, not 6: the full gain table runs to index 76, so 6 bits would
alias 64…76 down over 0…12.

The packet payload is split into four equal chunks, so the header describes
**five points** across the packet:

```
sample:   0                511              1022             1533            2043
          |----- chunk 0 ---|--- chunk 1 ----|--- chunk 2 ----|--- chunk 3 ----|
value:  base              d0               d1               d2               d3
```

A packet payload is `GPIF_BUF_SIZE − 4` dwords — 2044 on SuperSpeed, 1020 on
Hi-Speed — because the header replaces four. Chunks are therefore **511** dwords
(SuperSpeed) or 255 (Hi-Speed). Boundaries are counted in *dwords*, which equal
samples only in 16-bit single-channel mode; in 2×2 MIMO 2044 dwords is 1022
samples per channel, and in 8-bit mode a dword holds two samples.

Since the whole 32-bit word is used there is no in-band magic. An older image's
`0x12344321` decodes as a plausible-looking profile (base 9, chunks 22/13/21/0),
so `BLADERF_CAP_FPGA_RX_GAIN_TAG` is the only safe discriminator and the host
checks it before decoding.

---

## 2. The data path: a 12-bit sample to a USB transfer

Four sizes govern everything between the ADC and `bladerf_sync_rx()`. The first
two are fixed by the hardware and the FX3 firmware; the last two are the
`bladerf_sync_config()` parameters:

| unit | size | set by |
|---|---|---|
| sample | 4 bytes (`SC16_Q11`), 2 (`SC8_Q7`), 3 (`SC16_Q11_PACKED`) | `format` |
| message, i.e. packet | 8192 B SuperSpeed, 4096 B Hi-Speed | FX3 firmware ≥ v2.5.0 with FPGA ≥ v0.16.0; 2048/1024 B before that |
| buffer = one USB transfer | `buffer_size` samples, rounded up to whole messages | you |
| pipeline and pool depth | `num_transfers`, `num_buffers` | you |

### One sample

The AD9361 delivers 12 bits of I and 12 bits of Q. In `SC16_Q11` the FPGA
sign-extends each into an `int16_t`, so one sample is one 32-bit dword:

```
   AD9361 RX                            one sample, SC16_Q11
   +--------------+                     31            16 15             0
   | ADC   I[11:0]|---+                +---------------+---------------+
   |       Q[11:0]|---+--sign-extend-->|    Q int16    |    I int16    |
   +--------------+      12 -> 16      +---------------+---------------+
                                        little-endian, I first, and
                                        [-2048, 2047] means [-1.0, +1.0)
```

Those 12 significant bits are why every example divides by 2048: `|IQ| / 2048` is
a fraction of full scale, which is what `dBFS` in
[§5](#5-removing-the-front-end-gain-to-get-dbm-and-rssi) is relative to. `SC8_Q7`
truncates to 8 bits per component (2 bytes per sample) and `SC16_Q11_PACKED` packs
the 12-bit pairs with no padding (3 bytes), but only the two `_META` formats carry
a gain tag at all.

### One message

`fifo_writer` in the FPGA fills one fixed-size message per FX3 DMA buffer: a
16-byte header, then payload. The header is what the gain profile travels in, and
it costs four dwords of every message — which is why the payload is 2044 samples
and not 2048:

```
   message = one FX3 GPIF buffer = 8192 B SuperSpeed (4096 B Hi-Speed)

   byte  0        4                12        16                        8192
         +--------+----------------+---------+----------------------------+
         |  gain  |    timestamp   |  flags  |      2044 dwords of IQ     |
         | 32 bit |     64 bit     | 32 bit  |      (1020 on Hi-Speed)    |
         +--------+----------------+---------+----------------------------+
           |          |                 |
           |          |                 `- RX_HW_UNDERFLOW, MINIEXP1/2
           |          `- sample counter of the FIRST payload sample
           `- base | L | d0 | d1 | d2 | d3      (FPGA v0.17.0 and later;
                                                 previously a constant)
```

Both header fields address the same payload, one as an instant and one as a
profile across it:

```
   payload  |<--- 511 --->|<--- 511 --->|<--- 511 --->|<--- 511 --->|
   dword    0            511          1022          1533          2043
   gain     base -------> d0 --------> d1 --------> d2 --------> d3
   time     t ------------------------------------------------------> t+2043
```

The gain word is committed at the *tail* of its message, which is what lets the
profile describe that message's own samples; the timestamp still refers to the
first payload sample. One header per message means gain profiles arrive at
`fs / 2044` — 9786 per second at 20 Msps, each covering 102.2 µs.

### One buffer, one USB transfer

A buffer is a whole number of messages, and one buffer is exactly one USB bulk
transfer:

```
   buffer_size = 8192 samples  ->  4 messages, 32768 bytes

   buffer[k]  +-----------+-----------+-----------+-----------+
              |h|  msg 0  |h|  msg 1  |h|  msg 2  |h|  msg 3  |
              +-----------+-----------+-----------+-----------+
               ^ 16-byte header, one per message, never split across buffers
              \______________ one libusb bulk transfer, EP 0x81 ____________/
```

`bladerf_sync_config()` rounds `buffer_size` up to a whole number of messages and
warns; `bladerf_init_stream()` rejects it instead. (Both compare against the
SuperSpeed 8192-byte figure regardless of the negotiated speed, `sync.c:146`, so
on Hi-Speed the smallest buffer is two messages rather than one.)

Below the transfer, sizes the host does not choose: the FX3 hands the GPIF data to
the USB endpoint through an AUTO DMA channel of **11 buffers of one message each**
(`fx3_firmware/src/rf.c:293`), and SuperSpeed bulk moves them as 1024-byte packets
in bursts of up to 16 — so a message is 8 packets and the buffer above is 32.

Above the transfer, the two depths the host does choose:

```
        in flight (num_transfers = 32)          full, waiting for sync_rx()
   +----+----+----+ ... +----+   +----+----+----+----+ ... +----+
   |  0 |  1 |  2 |     | 31 |   | 32 | 33 | 34 | 35 |     |255 |
   +----+----+----+ ... +----+   +----+----+----+----+ ... +----+
    \________________ num_buffers = 256, buffer_size samples each _______/
        submitted to libusb              refilled by the RX worker as
        (must be < num_buffers)          transfers complete
```

At 20 Msps with those numbers: 2441 transfers per second of 32 KB each (80 MB/s),
and `num_buffers * buffer_size / fs` = **105 ms** of slack before a stalled reader
loses data. When the pool does fill, the worker discards `num_transfers` whole
buffers to recover — headers and all — which is the one way a gain value is truly
lost rather than merely summarised.

### One read

`bladerf_sync_rx()` walks messages inside the pool: strip the 16-byte header, fold
its gain word into the tag state, copy payload, repeat until the request is
satisfied. So the request size — not `buffer_size` — decides how many profiles one
`bladerf_metadata` has to describe:

```
   n = 2044   [h|<------------- 2044 ------------->]
              meta.timestamp = t, reserved: base/d0..d3 exact for these samples
              rx_gain_tags(): 1 entry

   n = 8176   [h|<-2044->][h|<-2044->][h|<-2044->][h|<-2044->]
              meta.timestamp = t, reserved: chunk_gain_index of the FIRST
                message only, plus min/max/CHANGED across all four
              rx_gain_tags(): 4 entries, one full profile each, tiling the
                returned samples
```

`meta.timestamp` is the timestamp of the first sample returned, so a contiguous
read asks for `previous timestamp + n`; the per-message entries carry their own
timestamps, which is how a discontinuity inside a large read stays visible.

---

## 3. Sample rate versus the 1000 µs slow-attack AGC interval

Slow-attack AGC changes gain at most once per gain update interval, 1000 µs by
default (`agc_gain_update_interval_us` in `fpga_common/src/ad936x_params.c`), and
by at most 2 indices per decision. Whether the four-chunk profile can represent
what happened is therefore purely a question of how much *time* a packet and a
chunk span, which is set by the sample rate:

| fs (Msps) | packet | chunk | packets per AGC interval | AGC decisions per packet | per chunk |
|---|---|---|---|---|---|
| 61.44 | 33.3 µs | 8.3 µs | 30.1 | 0.03 | 0.01 |
| 30.72 | 66.5 µs | 16.6 µs | 15.0 | 0.07 | 0.02 |
| **20** | **102.2 µs** | **25.6 µs** | **9.8** | **0.10** | **0.03** |
| 10 | 204.4 µs | 51.1 µs | 4.9 | 0.20 | 0.05 |
| 5 | 408.8 µs | 102.2 µs | 2.45 | 0.41 | 0.10 |
| 2.044 | 1000 µs | 250 µs | 1.00 | 1.00 | 0.25 |
| 1 | 2044 µs | 511 µs | 0.49 | 2.04 | 0.51 |
| 0.511 | 4000 µs | 1000 µs | 0.25 | 4.00 | 1.00 |

Two crossovers matter:

* **fs > 2.044 Msps** — a packet is shorter than one AGC interval, so it can
  contain **at most one** gain change. The five-point profile is then an exact
  description of the packet. Measured at 20 Msps over 58k packets: 95.7% of
  packets were flat (`base == d0 == d1 == d2 == d3`, one gain for all 2044
  samples), 4.3% held exactly one change, and **not one packet ever held two**.
* **fs < 0.511 Msps** — a *chunk* is longer than one AGC interval, so a chunk can
  contain more than one change. Only the value at the chunk's end is recorded, so
  intermediate gains are lost and the delta clamp becomes reachable. Below this
  rate treat the profile as a bound, not a description.

Between those, a packet may hold a few changes but no chunk holds more than one,
so the profile stays faithful.

At any normal rate the practical consequence is that **nothing needs to be
discarded**. 95.7% of packets carry one exact gain. For the 4.3% that contain a
change, the ambiguous chunk is split at its midpoint — first half the gain the
chunk started at, second half the gain it ended at — which is the midpoint
estimator for a transition of unknown position. See
[§5](#build-the-per-chunk-gain-array) for the residual error, which is under
0.01 dB.

Fast-attack is a different regime — 1 µs interval, so ~25 decisions per chunk at
20 Msps. Measured over 292k packets: 85.8% flat, 8.6% with more than one chunk
transition, and a largest excursion of 29 indices in one packet (21 of them inside
a single chunk), three short of the ±32 clamp. Use slow-attack for calibrated
power work.

---

## 4. The API

### C

```c
/* Overlaid on bladerf_metadata.reserved[32] by bladerf_sync_rx() in a metadata
 * RX format. Check `version` first. */
struct bladerf_rx_gain_tag {
    uint8_t  version;              /* BLADERF_RX_GAIN_TAG_VERSION_1, or _NONE */
    uint8_t  flags;                /* _CHANGED | _LOCKED                      */
    uint8_t  gain_index;           /* base index, first sample returned       */
    uint8_t  gain_index_min;       /* min over every chunk of every message    */
    uint8_t  gain_index_max;       /* max                                     */
    uint8_t  chunks;               /* 4                                       */
    uint16_t num_messages;         /* messages summarised; 0 = header consumed
                                    * by an earlier call                      */
    uint8_t  chunk_gain_index[8];  /* absolute index at the end of each chunk  */
};

/* One entry per message a sync_rx() call consumed, retrieved after the call.
 * Entries tile the returned samples in order. */
struct bladerf_rx_gain_tag_msg {
    bladerf_timestamp timestamp;   /* first sample of the message             */
    uint32_t sample_offset;        /* where it starts in the caller's buffer   */
    uint32_t sample_count;         /* how many samples of it were returned     */
    uint16_t msg_sample_offset;    /* payload offset of the first of those     */
    uint8_t  gain_index;           /* base index, message's first sample       */
    uint8_t  flags;                /* _LOCKED | _CARRIED                       */
    uint8_t  chunk_gain_index[8];  /* this message's own chunk profile         */
    uint8_t  reserved[4];
};

/* *num_tags is what was AVAILABLE, which may exceed max_tags; at most max_tags
 * are written. Call after sync_rx(), before the next one. */
int bladerf_get_rx_gain_tags(struct bladerf *dev,
                             struct bladerf_rx_gain_tag_msg *tags,
                             unsigned int max_tags, unsigned int *num_tags);

/* Index -> the conversion gain the chain actually achieved, in dB. Includes the
 * per-band offset AND the loaded gain calibration table, so the result is on the
 * same scale bladerf_get_gain() reports. */
int bladerf_rx_gain_tag_to_gain_db(struct bladerf *dev, bladerf_channel ch,
                                   uint8_t gain_index, float *gain_db);
```

`BLADERF_CAP_FPGA_RX_GAIN_TAG` gates availability (FPGA ≥ 0.17.0); `sync_rx()`
resolves it once at `sync_init()` and leaves `version` at `_NONE` otherwise.

One `bladerf_sync_rx()` call can span many packets but returns a single
`bladerf_metadata`, so `gain_index_min`/`max`/`CHANGED`/`num_messages` summarise
all of them while `chunk_gain_index` describes only the first. Two ways to keep
every profile:

* **Read one packet per call** — request exactly 2044 samples (1020 on
  Hi-Speed). Each call then consumes exactly one header, `num_messages` is 1, and
  `chunk_gain_index` describes precisely the samples returned. This depends on
  the *request size*, not on `buffer_size`: a buffer of many packets works
  identically and costs far fewer USB transfers, which is what matters at high
  sample rates. `buffer_size` only has to be a whole number of packets, and
  `bladerf_sync_config()` rounds it up to one.
* **Read as much as you like and call `bladerf_get_rx_gain_tags()`** — returns
  one entry per packet consumed, each with its own chunk profile and the slice of
  the buffer it applies to. Nothing is summarised away.

Reading several packets per call *without* the second is what loses resolution.

`num_messages` counts headers *read*, which is normally the number of entries but
can be one higher: a header can be read and still contribute no samples — the last
one before a discontinuity ended the call, or one walked past while seeking to a
requested timestamp. Those are counted in `num_messages` and dropped from the
array, which only ever holds entries that describe samples. Size an array from the
request (`num_samples / 2044 + 2`), and trust `*num_tags`, not `num_messages`, for
how many entries there are.

### Python

```python
from bladerf import _bladerf

tag = _bladerf.rx_gain_tag(meta)       # None if the FPGA supplied no tag
tag.gain_index, tag.chunk_gain_index, tag.changed, tag.locked
tags = dev.rx_gain_tags()              # one entry per packet in the last read
tags[0].sample_offset, tags[0].sample_count, tags[0].chunk_gain_index
tags[0].timestamp, tags[0].msg_sample_offset, tags[0].carried
dev.rx_gain_tag_to_gain_db(ch, idx)    # dB, or None if idx is outside the table
dev.get_timestamp(_bladerf.Direction.RX)
dev.set_gain_calibration(ch, path)     # bladerf_load_gain_calibration
```

`gain_profile.py` in this directory carries the reconstruction every example needs,
so none of them hand-roll it:

```python
import gain_profile as gp

msg = gp.message_samples(dev)              # 2044 or 1020
read_n = gp.read_samples(msg, packets=2)   # request whole packets: 4088

gains = gp.GainDb(dev, ch)                 # memoised per (frequency, index)
gdb = gains.at(freq_hz)

for span in gp.chunk_spans(tag, msg):      # constant-gain spans, buffer coords
    span.lo, span.hi, span.index, span.split
gain_db, filled, ambiguous = gp.gain_db_array(tags, msg, count, gdb)
dbfs, dbm = gp.packet_power(v, tag, msg, gdb)   # one packet, per-chunk corrected
idx_min, idx_max, changed = gp.profile(tag)     # the summary fields, per packet
```

---

## 5. Removing the front-end gain to get dBm and RSSI

### Load a calibration table first

Without one, `rx_gain_tag_to_gain_db()` returns the nominal gain and every
absolute figure carries a fixed offset. With one, its frequency-interpolated
correction is folded in and the numbers are absolute.

```python
dev.set_gain_calibration(ch, f"{dev.get_serial()}_rx_gain_cal.tbl")
```

The table's `gain_corr` entries are `dBFS_measured − dBm_in`, swept at an RX gain
of 0, so the correction is frequency dependent — measured +4.32 dB at 731 MHz and
+3.40 dB at 915 MHz on one board. That fractional part is why the conversion
returns `float`.

### Build the per-chunk gain array

`chunk_gain_index[i]` is the index at the **end** of chunk i, and `base` is the
index at sample 0, so chunk i runs from `pts[i]` to `pts[i+1]`. When those differ
the chunk contains a transition at an unknown position, so split the chunk at its
midpoint:

```python
CHUNKS = 4
clen = nsamples // CHUNKS                     # 511 on SuperSpeed
pts = [tag.gain_index] + list(tag.chunk_gain_index[:CHUNKS])

def gdb(idx):                                 # USB round trip in FPGA tuning
    if idx not in cache:                      # mode, so memoise per (freq, idx)
        cache[idx] = dev.rx_gain_tag_to_gain_db(ch, idx)
    return cache[idx]

gain_db = np.empty(nsamples, dtype=np.float32)
for i in range(CHUNKS):
    lo = i * clen
    hi = lo + clen if i < CHUNKS - 1 else nsamples   # last chunk takes the remainder
    if pts[i] == pts[i + 1]:                        # flat: exact
        gain_db[lo:hi] = gdb(pts[i + 1])
    else:                                           # transition inside: split
        mid = lo + (hi - lo) // 2
        gain_db[lo:mid] = gdb(pts[i])
        gain_db[mid:hi] = gdb(pts[i + 1])
```

**Why the midpoint, and what it costs.** If the transition is uniformly
distributed within the chunk, assigning the whole chunk one value mis-assigns an
expected 0.50 of it (worst case 1.00); splitting at the midpoint mis-assigns an
expected 0.25 (worst case 0.50). So the split halves the error and removes any
need to discard data.

The residual is negligible in slow-attack, where a decision moves the gain by at
most 2 dB and 4.3% of packets contain one. That leaves ~0.27% of samples carrying
a one-step error, bounding total power to **under 0.01 dB**. Measured on hardware
over 2¹⁸ samples at 731 MHz, the two strategies differed by **0.0035 dB** in total
power and **0.0046 dB** in a 3 MHz RSSI band.

Fast-attack is not covered by this reasoning: a chunk there can hold ~25
decisions, so a midpoint split is not meaningful and the profile should be treated
as a bound.

### Reads larger than one packet

`rx_gain_tags()` removes the one-packet-per-call constraint: request whatever
size suits the processing, then build the same per-sample array over the whole
read. Each entry carries its own profile, so the loop above becomes the loop
below with no change in resolution. That loop is `gain_profile.chunk_spans()` and
`gain_profile.gain_db_array()`, which is what the examples call — it is spelled out
here because the coordinate handling is the part worth understanding:

```python
dev.sync_rx(buf, nsamples, 3500, meta)          # nsamples >> 2044 is fine
iq = np.frombuffer(buf, np.int16, 2 * meta.actual_count).astype(np.float32)
iq = (iq[0::2] + 1j * iq[1::2]) / 2048.0

gain_db = np.empty(meta.actual_count, dtype=np.float32)
for t in dev.rx_gain_tags():
    clen = MESSAGE_SAMPLES // CHUNKS               # 511 on SuperSpeed
    pts = [t.gain_index] + list(t.chunk_gain_index[:CHUNKS])
    for c in range(CHUNKS):
        # chunk c spans payload samples [c*clen, (c+1)*clen); intersect that
        # with the part of the message this entry actually returned
        lo = max(c * clen, t.msg_sample_offset)
        hi = min((c + 1) * clen if c < CHUNKS - 1 else MESSAGE_SAMPLES,
                 t.msg_sample_offset + t.sample_count)
        if hi <= lo:
            continue
        # payload coordinates -> buffer coordinates
        blo = t.sample_offset + (lo - t.msg_sample_offset)
        bhi = t.sample_offset + (hi - t.msg_sample_offset)
        if pts[c] == pts[c + 1]:                   # flat: exact
            gain_db[blo:bhi] = gdb(pts[c + 1])
        else:                                      # transition: midpoint split
            mid = blo + (bhi - blo) // 2
            gain_db[blo:mid] = gdb(pts[c])
            gain_db[mid:bhi] = gdb(pts[c + 1])
```

Entries tile the returned samples — `sample_offset + sample_count` of one is the
`sample_offset` of the next, and the last ends at `meta.actual_count` — so this
fills `gain_db` completely. Two details worth knowing:

* A call that begins part way into a packet (because the previous call ended part
  way into it) gets an entry flagged `carried`, with `msg_sample_offset` saying
  how far in. Its profile came from the header an earlier call read, which is why
  the chunk arithmetic above works in payload coordinates rather than buffer
  coordinates.
* Packets the call skipped over — seeking to a requested timestamp, or discarding
  after an overrun — produce no entry, so a jump in `timestamp` between adjacent
  entries marks a discontinuity. `meta.status & BLADERF_META_STATUS_OVERRUN`
  reports the same thing.

Sizing a C array for it: at most `num_samples / 2044 + 2` entries on SuperSpeed.

**Request whole packets** and both wrinkles disappear: with a read of
`k * 2044` samples starting on a packet boundary, every entry is a complete
packet, `msg_sample_offset` is 0 and `carried` is never set, so per-packet
bookkeeping needs no special cases. That is what `gain_profile.read_samples()` is
for, and why `--read-packets` rather than `--read-samples` is the knob the hop
examples expose. Contiguous reads keep the alignment indefinitely; a resync starts
a new one at the next packet boundary.

`rx_psd_dbm.py` is written this way — one `sync_rx()` for the whole capture, then
one pass over `rx_gain_tags()` — and asserts the coverage rather than assuming it:

```python
gain_db, filled, ambiguous = gain_per_sample(tags, MESSAGE_SAMPLES, want, gdb)
if filled != want:
    raise RuntimeError(f"gain tags covered {filled} of {want} samples")
```

### Turning a tagged index into a gain-table entry

Every index in a tag — `gain_index` and each `chunk_gain_index[i]` — is a row of
the AD9361's **RX full gain table**, which is the same table the RFIC's AGC walks.
`bladerf_rx_gain_tag_to_gain_db()` is what resolves one into dB, and it composes
three things:

1. **The table row.** Gain is affine in the index above a per-band offset:
   `starting_gain_db + (index - idx_step_offset) * gain_step_db`, clamped at the
   band's maximum. The three bands are in
   `fpga_common/src/ad936x_helpers.c:196`, mirroring `ad9361_init_gain_tables()`
   in the RFIC driver:

   | LO | starting dB | step | idx offset | max dB | usable indices |
   |---|---|---|---|---|---|
   | ≤ 1300 MHz | +1 | 1 | 0 | 77 | 0…76 |
   | ≤ 4000 MHz | −4 | 1 | 1 | 71 | 0…76 |
   | > 4000 MHz | −10 | 1 | 4 | 62 | 0…76 |

   So the *same index means a different gain in a different band* — the mapping is
   not a property of the index alone.

2. **The per-band front-end offset**, the same one `bladerf_get_gain()` adds, so
   the result is on the scale that call reports rather than a bare RFIC figure.

3. **The gain calibration table**, if one is loaded and enabled for the channel.
   Its `gain_corr` entry is interpolated at the tuned frequency and folded in,
   which is what makes the number absolute instead of nominal. This is also why
   the return type is `float`: the correction has a fractional part (+4.32 dB at
   731 MHz, +3.40 dB at 915 MHz on one board).

Two consequences for how to call it:

* **Memoise per (frequency, index), not per index.** The conversion depends on the
  tuned LO through all three terms, so a cache has to be invalidated on retune. In
  `BLADERF_TUNING_MODE_FPGA` each call also costs a USB round trip to read the LO,
  which is why the examples cache: a capture only ever contains a handful of
  distinct indices (measured 4 to 6 over 2¹⁸ samples), so the miss count is tiny
  even though the call count is one per chunk per packet.
* **A `None` / `BLADERF_ERR_INVAL` return is a configuration error, not a bad
  sample.** It means the index fell outside the table, which almost always means
  `CTRL_OUT` is not pointed at row `0x16` — check RFIC register `0x035`.

### Apply it in the time domain, before any FFT

The validated relation is

```
dBm_in = dBFS − gain_db        with dBFS = 10·log10(mean(|raw/2048|²))
```

so dividing by the linear voltage gain leaves samples whose mean square *is* power
in mW:

```python
y = iq * 10.0 ** (-gain_db / 20.0)     # iq = (I + jQ)/2048
power_dbm = 10 * np.log10(np.mean(np.abs(y) ** 2))
```

**It must be per sample, not per FFT segment.** A 2048-point Welch segment is
larger than the 2044-sample payload, so a segment can never sit inside one packet
— it always straddles a boundary and generally spans four gain chunks. "The gain
for this segment" does not exist.

### PSD normalised so bins sum to power

```python
w = np.hanning(nperseg)
acc = sum(np.abs(np.fft.fft(y[i:i+nperseg] * w))**2
          for i in range(0, len(y)-nperseg+1, nperseg//2))
p_lin = np.fft.fftshift(acc / nseg / (nperseg * np.sum(w**2)))
```

Then `sum(p_lin) == mean(|y|²)`, so any subset of bins sums to the power in those
bins:

```python
rssi_dbm = 10 * np.log10(np.sum(p_lin[mask]))
```

This is scipy's `scaling='density'` **times the bin width**, not
`scaling='spectrum'` (which normalises by `sum(w)²` so a tone's peak bin reads its
amplitude, but summing bins does not give total power). The scipy equivalent:

```python
f, Pxx = welch(y, fs, 'hann', nperseg=2048, noverlap=1024,
               return_onesided=False, scaling='density', detrend=False)
rssi_mW = np.sum(Pxx[mask]) * (fs / 2048)
```

### Validation

Synthetic, a −40 dBm tone in −70 dBm noise with the gain stepping 60/54/48 dB
mid-capture: recovered total power and recovered tone RSSI both correct to
**0.000 dB**. Uncorrected, the same data reads 56.9 dB high.

Hardware, 731 MHz / 20 Msps / 2¹⁸ samples, contiguous over 129 packets while the
AGC swept 13 distinct gains: the spectral sum matched the time-domain mean power
to **0.001 dB**, and across a 20 dB range of commanded gain the recovered total
power held to **1.2 dB**.

---

## 6. Pitfalls

* **Contiguity.** Concatenating IQ across dropped samples corrupts the spectrum.
  Read contiguously (request the samples following the previous packet) rather
  than with `BLADERF_META_FLAG_RX_NOW`, which returns whatever is current and
  silently discards the rest without setting an overrun status.
* **The reader lags real time** by up to `num_buffers × buffer_size / fs` — 105 ms
  at 256 buffers of 8192 samples and 20 Msps. A retune takes effect in real time,
  so for that long afterwards the packets still contain the *previous* frequency's
  samples, and anything labelling them by "the frequency I last asked for" is
  wrong. Attribute by timestamp instead.
* **`bladerf_set_frequency()` gives you a window, not a boundary.** It moves the LO
  early in its sequence and then spends milliseconds on band selection and
  recalibration, so bracketing it with `bladerf_get_timestamp()` yields a window
  that only *contains* the change. Measured at 20 Msps: **13.5 ms wide, with the RF
  actually moving 5.8 ms in** — about 130 packets per hop whose frequency is
  unknown. And an unknown frequency means an unknown *gain*:
  `bladerf_rx_gain_tag_to_gain_db()` is band dependent, so converting index 72
  against the wrong side of the hop is off by 1.46 dB between 951 and 731 MHz — a
  plausible-looking wrong number rather than an obvious one. Report those packets
  as uncertain and leave their dB figures empty rather than guessing; the examples
  do this with a `certain` flag, and keep `dBFS`, which is band independent.
* **What the retune *does* guarantee is its end.** The timestamp read after
  `bladerf_set_frequency()` returns is a real boundary: the call does not return
  until the LO is programmed and the band's gain table is loaded, so every sample at
  or after it was captured at the new frequency. Gate a capture on that timestamp
  and no stale sample can enter it — and because a read with
  `meta.timestamp` set discards older buffers without copying them, the wait costs
  nothing but the retune itself. That is also what makes a dwell exact: read until a
  packet's timestamp reaches `ts_after + dwell * fs` rather than dwelling on the
  wall clock, which ends early by the reader's lag.
* **Let the AGC settle** before capturing. Streaming starts at maximum gain; any
  samples the front end compressed there are nonlinear and no gain correction can
  undo them.
* **The cal table is one scalar per frequency**, not a response across the band,
  so accuracy degrades toward the band edges. Tune each signal near centre.
* **`locked` is fast-attack only.** To ask whether the gain was steady in any
  mode, use `gain_index_min == gain_index_max` with `CHANGED` clear.
* **Reading many packets per call and using only `meta.reserved`** silently loses
  resolution: `chunk_gain_index` there describes the first packet, so a 129-packet
  read gets a profile for 1/129th of its samples and no error is reported. Either
  read one packet per call or use `bladerf_get_rx_gain_tags()`.
* **A chunk that contains a transition** knows its gain only to chunk resolution.
  Splitting at the midpoint (§5) keeps the residual under 0.01 dB in slow-attack,
  so packets need not be dropped — but in fast-attack the same chunk may hold many
  decisions, and there `CHANGED` packets are better excluded.

---

## 7. Examples in this directory

All of them reconstruct the gain from `rx_gain_tags()` rather than from the
per-call summary, so reads are decoupled from packets. The two hop tools read
several packets per `sync_rx()` (`--read-packets`, 2 by default) and still emit a
row, a PSD and an `.iq` file per packet, at a fraction of the call rate; the two
power tools take a whole capture in one read. `--buffer-size` sizes the USB
transfers independently of either.

| script | purpose |
|---|---|
| `gain_profile.py` | shared module: chunk spans, per-sample gain, per-packet power, memoised index → dB |
| `rx_gain_tag_sweep.py` | per-packet timestamp and gain across a frequency hop list; CSV, per-dwell AGC summary. `--power` reports each packet's absolute power using its own per-chunk profile |
| `rx_hop_waterfall.py` | per-step IQ files plus a waterfall with one PSD row per packet; shows the retune lag directly |
| `rx_psd_dbm.py` | one large contiguous read → per-packet gain from `rx_gain_tags()` → Welch PSD calibrated in dBm, with an RSSI band |
| `rx_lte_rssi.py` | RSSI of LTE cells given as `CENTER:CHANNEL_BW`, each tuned to its own centre |

All need the in-repo bindings and a libbladeRF containing the tag support:

```bash
export PYTHONPATH=$PWD/host/libraries/libbladeRF_bindings/python
export LD_LIBRARY_PATH=$PWD/host/build/output
```
