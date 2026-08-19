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

## 2. Sample rate versus the 1000 µs slow-attack AGC interval

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
[§4](#build-the-per-chunk-gain-array) for the residual error, which is under
0.01 dB.

Fast-attack is a different regime — 1 µs interval, so ~25 decisions per chunk at
20 Msps. Measured over 292k packets: 85.8% flat, 8.6% with more than one chunk
transition, and a largest excursion of 29 indices in one packet (21 of them inside
a single chunk), three short of the ±32 clamp. Use slow-attack for calibrated
power work.

---

## 3. The API

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
all of them while `chunk_gain_index` describes the first. **Set `buffer_size` to
2048 samples on SuperSpeed (1024 on Hi-Speed) to get exactly one packet per
call**, and the summary and the profile then describe the same packet.

### Python

```python
from bladerf import _bladerf

tag = _bladerf.rx_gain_tag(meta)       # None if the FPGA supplied no tag
tag.gain_index, tag.chunk_gain_index, tag.changed, tag.locked
dev.rx_gain_tag_to_gain_db(ch, idx)    # dB, or None if idx is outside the table
dev.get_timestamp(_bladerf.Direction.RX)
dev.set_gain_calibration(ch, path)     # bladerf_load_gain_calibration
```

---

## 4. Removing the front-end gain to get dBm and RSSI

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

## 5. Pitfalls

* **Contiguity.** Concatenating IQ across dropped samples corrupts the spectrum.
  Read contiguously (request the samples following the previous packet) rather
  than with `BLADERF_META_FLAG_RX_NOW`, which returns whatever is current and
  silently discards the rest without setting an overrun status.
* **The reader lags real time** by up to `num_buffers × payload / fs` — 26 ms at
  1024 buffers and 20 Msps. `bladerf_set_frequency()` takes effect in real time,
  so for that long after a retune the packets still contain the *previous*
  frequency's samples. Attribute by timestamp, bracketing each retune with
  `bladerf_get_timestamp()` before and after; `set_frequency()` moves the LO early
  and then spends milliseconds recalibrating, so a single timestamp taken
  afterwards is ~8 ms late.
* **Let the AGC settle** before capturing. Streaming starts at maximum gain; any
  samples the front end compressed there are nonlinear and no gain correction can
  undo them.
* **The cal table is one scalar per frequency**, not a response across the band,
  so accuracy degrades toward the band edges. Tune each signal near centre.
* **`locked` is fast-attack only.** To ask whether the gain was steady in any
  mode, use `gain_index_min == gain_index_max` with `CHANGED` clear.
* **A chunk that contains a transition** knows its gain only to chunk resolution.
  Splitting at the midpoint (§4) keeps the residual under 0.01 dB in slow-attack,
  so packets need not be dropped — but in fast-attack the same chunk may hold many
  decisions, and there `CHANGED` packets are better excluded.

---

## 6. Examples in this directory

| script | purpose |
|---|---|
| `rx_gain_tag_sweep.py` | per-packet timestamp and gain across a frequency hop list; CSV, per-dwell AGC summary |
| `rx_hop_waterfall.py` | per-step IQ files plus a waterfall with one PSD row per packet; shows the retune lag directly |
| `rx_psd_dbm.py` | contiguous capture → Welch PSD calibrated in dBm, with an RSSI band |
| `rx_lte_rssi.py` | RSSI of LTE cells given as `CENTER:CHANNEL_BW`, each tuned to its own centre |

All need the in-repo bindings and a libbladeRF containing the tag support:

```bash
export PYTHONPATH=$PWD/host/libraries/libbladeRF_bindings/python
export LD_LIBRARY_PATH=$PWD/host/build/output
```
