# GPS time over UART → bladeRF RX metadata (UBX-NAV-TIMEUTC)

The `gps_uart` module (`../vhdl/gps_uart.vhd`) receives time from a u-blox GPS over a
UART + PPS and stamps every RX DMA block's metadata with an NTP-style UTC timestamp:

```
timestamp[63:32] = Unix epoch seconds (UTC)
timestamp[31:0]  = sub-second fraction = number of RX SAMPLES since the last PPS edge
```

It decodes the **binary UBX-NAV-TIMEUTC** message (class 0x01, id 0x21), verifying the
Fletcher checksum and the message's `validUTC` flag, and disciplines the seconds with PPS.

---

## bladeRF side (FPGA)

| Signal | `exp_gpio` | FPGA ball | Expansion-connector pin |
|--------|-----------|-----------|-------------------------|
| UART RX (bladeRF ← GPS TX) | `exp_gpio[0]` | J8 | **56** |
| UART TX (bladeRF → GPS RX) | `exp_gpio[1]` | J7 | **54** |
| PPS     (bladeRF ← GPS)    | `exp_gpio[2]` | H9 | **50** |
| GND                        | —         | —  | **52** (next to PPS); also 15/16, 21/22, 27/28, 33/34, 39/40, 45/46, 51/52, 59/60 |

- **Baud: 110803.** The `uart_rx` bit period is `2 * C_GPS_DIV_CNT_115k = 2 * 361 = 722`
  `sys_clock` cycles; at 80 MHz that is 9.025 µs → **110803 baud**. The GPS UART must be set
  to this exact rate (see below) so there is no baud error.
- Do **not** connect the GPS's 3.3 V output pins to the bladeRF. Avoid connector pins
  57/58 (+5 V) and pin 1 (XB_VCCIO).
- GND pins (schematic U74, Samtec BSH-030-01-F-D-A): 15, 16, 21, 22, 27, 28, 33, 34, 39, 40,
  45, 46, 51, 52, 59, 60 are all on the ground rail. Pin **52** is the pair directly below
  PPS pin 50, so tie the GW16143 J2 pin 4 (timepulse GND) there; a PPS lead with no nearby
  return floats and picks up UART edges.

---

## GPS side (u-blox ZED-F9P, e.g. Gateworks GW16143) — UART1 config

Configure UART1 (the port wired to the bladeRF) with `ubxtool` on the host attached to the
F9P (e.g. the Jetson). These are written to RAM+BBR+Flash (layer 7) so they **persist across
power cycles**. The **USB port (gpsd) is untouched** and keeps its NMEA stream.

Run `./f9p_uart1_ubx.sh` (next to this file), or the equivalent commands:

```sh
ubxtool -z CFG-UART1-BAUDRATE,110803,7              # match the bladeRF RX baud exactly
ubxtool -z CFG-UART1OUTPROT-UBX,1,7                 # allow UBX out on UART1
ubxtool -z CFG-UART1OUTPROT-NMEA,0,7               # drop NMEA -> clean binary stream
ubxtool -z CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1,1,7    # emit NAV-TIMEUTC every epoch (1 Hz)
ubxtool -z CFG-TP-TP1_ENA,1,7                       # PPS/timepulse on (1 Hz, after fix)
```

Verify:

```sh
ubxtool -g CFG-UART1-BAUDRATE                 # 110803
ubxtool -g CFG-UART1OUTPROT-NMEA              # 0
ubxtool -g CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   # 1
```

Notes:
- `CFG-UART1-DATABITS`/`STOPBITS`/`PARITY` stay at the default 8N1.
- PPS only pulses once the receiver has a position fix; an active antenna with sky view is
  required (GW16143 J1 MMCX).
- The GW16143 has two UART headers: **J5 = UART1**, J3 = UART2. Use **J5**.

### GW16143 → bladeRF wiring

| GW16143 | signal | → bladeRF connector pin |
|---------|--------|--------------------------|
| J5 pin 2 | UART_TX (GPS out) | 56 (bladeRF RX) |
| J5 pin 3 | UART_RX (GPS in)  | 54 (bladeRF TX) |
| J2 pin 2 | Timepulse / PPS   | 50 |
| J2 pin 4 | Timepulse GND | **52** (adjacent to pin 50, keeps the PPS return short) |
| J5 pin 4 | UART GND     | 59 / 60 (or 51/52) |

(GW16143 J2 pin 3 "Interrupt" = F9P EXTINT, an input; not used by this design.)

---

## Host-side use (FPGA v0.19.0, libbladeRF 2.11.0)

The RX metadata header is **24 bytes** in `SC16_Q11_META` / `SC8_Q7_META` on this image:
the standard 16-byte header (reserved/gain tag, 64-bit timestamp, flags) followed by two
extension dwords, with two fewer samples per message (2042 on SuperSpeed, 250 on Hi-Speed).
libbladeRF 2.11.0 recognises the image (`BLADERF_CAP_FPGA_RX_META_V2`, FPGA >= 0.19.0) and
sizes messages accordingly; older libbladeRF on this image would mistake the two extension
dwords for samples.

**The 64-bit timestamp is still the stock free-running sample counter.** Nothing about the
sync layer, timed reads, overrun detection, or host software that times samples against NTP
changes; with no GPS fitted, no fix, or no PPS, that path works exactly as before. GPS time is
*additional*, in the extension dwords, sampled at the same instant as the counter:

| offset | content |
|---|---|
| 0x10 | GPS word: bits 25:0 samples since the PPS (saturating), 26 UTC_VALID, 27 PPS_ALIVE, 28 FRAC_VALID, 29 UBX_FRESH, 30 GPS_LOCK, 31 GPS_PRESENT |
| 0x14 | UTC epoch seconds (0 until UTC_VALID) |

```
utc(payload sample n of a message) = gps_epoch + (gps_frac + n) / sample_rate
```

Through the sync API, `bladerf_sync_rx()` overlays a `struct bladerf_rx_gps_time` on
`bladerf_metadata.reserved` right after the gain tag (`version`, `flags`, `epoch`, `frac` of
the first returned sample, and `pps_interval`), and sets these status flags:

| `bladerf_metadata.status` flag | meaning |
|---|---|
| `BLADERF_META_STATUS_GPS_PRESENT` | the image supplies GPS time (overlay valid) |
| `BLADERF_META_STATUS_GPS_UTC_VALID` | the epoch has been loaded from a valid UBX-NAV-TIMEUTC at least once |
| `BLADERF_META_STATUS_GPS_PPS_ALIVE` | a PPS edge was accepted within the last 1.5 s |
| `BLADERF_META_STATUS_GPS_UBX_FRESH` | a valid UBX time message within the last 5 s (clear + PPS_ALIVE = holdover) |
| `BLADERF_META_STATUS_GPS_FRAC_VALID` | a PPS edge has been seen since RX was enabled (see below) |
| `BLADERF_META_STATUS_GPS_LOCK` | UTC_VALID and PPS_ALIVE and FRAC_VALID, for **every** message the call returned samples from |

Per message, `bladerf_get_rx_gain_tags()` entries carry `gps_epoch`, `gps_frac`,
`pps_interval` and `BLADERF_RX_GAIN_TAG_GPS_LOCK`.

**Why FRAC_VALID exists:** the FPGA counts RX samples, and samples only flow while RX is enabled.
After `bladerf_enable_module(RX, true)` the fraction is short by however long RX was off, until
the next PPS re-anchors it; so the first second of every session is unlocked.

**PPS interval:** the counter value at the PPS that began a second is `timestamp - gps_frac`,
the same for every message of that second; from one second to the next it moves by exactly the
number of samples in between. libbladeRF derives `pps_interval` that way (e.g. 39 999 990 at
40 MSps) -- the sample rate as measured against GPS, for converting the fraction to seconds
exactly. Raw-header readers can do the same.

The async API still works for raw headers (bytes 4..11 = counter, 16..19 = GPS word, 20..23 =
epoch); `gps_pin_check.c` and the scratch tools read them that way.

**Async API ordering (libbladeRF 2.11.0 fix).** With `bladerf_init_stream()` /
`bladerf_enable_module()` / `bladerf_stream()`, RX used to be enabled before any USB transfer
was pending, and the FX3 then lost the first two dwords of the session: every message of the
stream was framed two samples early relative to its header. With the 16-byte header that was
an invisible constant timestamp offset; with the extension it showed up as the GPS entry
sitting in the last two dwords of every message. `bladerf_stream()` now drops and re-raises
the FX3's RX stream around the submission of its transfers (the sync path always primed its
transfers first), so both APIs give the same, correctly framed messages.

Verified on hardware 2026-11-03 at 40 and 61.44 MSps: async raw headers carry the GPS entry
at offset 16 in every message; `drift_cap` (async) measures the PPS interval 39 999 989.7
samples, sd 0.6, 0 anomalies; `libbladeRF_test_rx_gps_time` (sync, 40 MSps, 20 s) passes with
0 overruns, 0 counter or GPS steps wrong, every message GPS_LOCK after the session's first PPS,
and a timed read landing on the exact counter value. Correlate the epoch with
the gpsd position/time fix read over the F9P's USB port (same receiver, same PPS).

Gain tag: the reserved word still carries the RFIC gain profile (FPGA >= 0.17.0). Its chunks
still span 511 dwords each; the extension entry is the first two dwords of chunk 0, so payload
sample `s` (0-based) belongs to chunk `(s + 2) / 511`. The time marker (flags bit 4, FPGA
>= 0.18.0) is unchanged.

## Sample-exact timestamp in the rx_clock domain (`rx_sample_ts`)

The timestamp that reaches the metadata is owned by the **rx_clock** domain (`../vhdl/rx_sample_ts.vhd`):
the fraction is a count of RX sample strobes since the PPS, and the seconds advance in the **same rx cycle**
in which that count restarts, so epoch and fraction can never disagree. `gps_uart` (sys_clock) still decodes
UBX and qualifies the PPS; it hands over two single-bit toggles (accepted PPS edge, UBX epoch load) and the
epoch/lock state as one coherent word through `ts_cdc`. The UBX load lands ~42 ms after the PPS, so it never
collides with the per-second advance. Counting samples instead of snapshotting a sys_clock counter removes the
clock-domain snapshot jitter (±~100 ns, ±7 ticks) that the earlier design carried: `fifo_writer` latches the
timestamp at the first sample of each message, so each block is stamped exactly to the sample
(25 ns at 40 MSps). The host needs only the sample rate to place every sample in UTC.

## Timestamp integrity across the clock domains (`ts_cdc`)

The epoch and lock state are generated in the **`sys_clock`** domain (UBX decoder) but
consumed in the **`rx_clock`** (AD9361) domain. Sampling a multi-bit word across asynchronous
clocks without synchronization **tears**: when the capture instant coincides with an update
some bits are captured before the update and some after, producing a garbage value.

`ts_cdc` (`../vhdl/ts_cdc.vhd`) makes this crossing safe with a toggle/ack handshake: the
source latches a snapshot into a holding register and flips a request toggle, and does not
latch the next snapshot until the destination has acknowledged — so the held value is
guaranteed stable while the destination captures it. In the current design `ts_cdc` carries the 34-bit
{lock, sync toggle, epoch} word into `rx_sample_ts`; the final 64-bit timestamp is generated in rx_clock directly. The refresh latency is one handshake round trip (a few cycles each
way, well under a microsecond), far finer than a DMA block. It is instantiated inside
`rx_sample_ts`, which `bladerf-hosted.vhd` places between `gps_uart` (sys_clock) and
`U_rx.rx_timestamp` (rx_clock).

## PPS qualification (glitch filter + refractory window)

On hardware the raw PPS pin picked up induced edges from the **UART RX line**: ~42 ms after
each real PPS (exactly when the F9P sends NAV-TIMEUTC) a ~3 ms burst of false edges bumped
`epoch_sec` by up to +12 and kept zeroing the fraction, until the frame's checksum passed
and the UBX resync snapped the epoch back. Only ~0.2 % of messages were affected, but each
one carried a wrong epoch *and* a wrong fraction.

`gps_uart` now qualifies PPS (generics `G_PPS_FILTER_CYCLES = 8`, `G_PPS_REFRACTORY =
72 000 000`): an edge counts only after 8 consecutive high samples (100 ns), and after an
accepted edge the input is ignored for 0.9 s. A real PPS is 1 Hz, so anything else is
rejected. The fraction counter is preloaded with the qualifier latency (`G + 4` cycles) so it
still counts from the physical edge. **PCB note:** keep the PPS trace away from the UART
lines (and give it a clean ground return) — the filter makes the design immune, but the
coupling is real on the pigtail setup.

## Reset: GPS time persists across stream enable/disable

`gps_uart` is reset only by a **power-on** reset (synchronized from the sys_clock PLL lock,
`U_reset_sync_gps`), *not* by the FX3 stream reset (`fx3_ctl(7)`) that libbladeRF pulses on
every RX enable. Previously that reset wiped the GPS time at the start of every capture,
leaving the first ~1 s of blocks untimed (`epoch == 0`) until the next NAV-TIMEUTC. GPS time
is independent of the sample stream, so it now carries over and the very first block of a
capture is already timed.

## Verifying on hardware — read the RAW headers

Do **not** judge the timestamp through `bladerf_sync_rx()`: libbladeRF's sync layer treats
`metadata.timestamp` as a sample counter and does arithmetic on it (seeks, offsets,
`RX_NOW` re-anchoring), so the value it hands back can be mangled or zero-filled. Use the
async stream API with `BLADERF_FORMAT_SC16_Q11_META` and parse the 128-bit header of each
8192-byte message yourself (timestamp = bytes 4..11, little-endian, FX3 fw ≥ 2.5.0 on
SuperSpeed). Check: epoch matches an NTP-synced clock, epoch only steps +1 at the PPS, and
`frac` is monotonic between PPS edges (it advances by exactly 2044 per message, one per sample).

Verified on hardware 2026-11-03 (build `hostedxA4-2026-11-03_11.39.45`): epoch steps `+1`
exactly once per second at the PPS, no spurious edges, `epoch == 0` never occurs, the first
message of a capture is already timed, and the epoch matches an NTP-synced host to the second.

**Measured oscillator (2026-11-03, 10 min at 40 MSps, sys_clock-tick build):** PPS-to-PPS interval
mean 79 999 982.7 ticks = VCTCXO **−0.216 ppm**, per-second sd 0.04 ppm, warm-up trend +0.05 ppm/10 min;
a free-running counter would drift −129 µs in 10 min, the PPS-disciplined one is bounded to one second's
worth (0.33 µs worst). Host software may rescale `frac` by the measured PPS interval for ~40 ns residual.

**Sample-counting build verified (2026-11-03, `hostedxA4-2026-11-03_12.21.03`, 60 s at 40 MSps,
timepulse GND on pin 52):** PPS-to-PPS interval mean 39 999 991.4 samples (−0.215 ppm, the same
VCTCXO offset as above), per-second **sd 1.2 samples (30 ns)**, min/max 39 999 988 / 39 999 993;
every message exactly 2044 samples after the previous one; epoch − host = 0 for every second;
0 epoch anomalies. The ±7-tick snapshot jitter of the tick build is gone.

**10-minute runs at other rates (2026-11-03, same build):**

| Rate | seconds | PPS interval mean | offset | sd | min / max | free-running 10 min | epoch = host |
|------|---------|-------------------|--------|----|-----------|---------------------|--------------|
| 61.44 MSps | 598 | 61 439 989.6 | −0.169 ppm | 1.5 samples (25 ns) | 61 439 980 / 61 439 992 | −101 µs | 598/598 |
| 20.48 MSps | 598 | 20 479 995.1 | −0.238 ppm | 0.5 samples (23 ns) | 20 479 994 / 20 479 996 | −142 µs | 598/598 |

Zero epoch anomalies, zero dropped USB buffers, every message exactly 2044 samples apart at both rates
(61.44 MSps is the maximum USB 3 rate). The per-second scatter is ~25 ns rms at any rate, i.e. the GPS
time-pulse jitter plus one sample of quantization. The mean offset differs slightly between rates because the
AD9361 BBPLL rounds each requested rate with its fractional-N divider; it is constant per rate and can be
calibrated out host-side if needed. Chart: https://claude.ai/artifact/CQQn9HWJjmpyL81tk39RKJ

**Known startup artifact:** the first ~3 messages (~3 ms) of the *first* stream after an FPGA
load carry garbage headers (epoch ≈ 4.28e9, i.e. year 2105) — stale FX3/GPIF buffer contents
on the first enable. Every later stream start is clean from message 0. Hosts should discard
any message whose epoch is implausible (e.g. `epoch > 0x7fffffff`), which also covers this.
