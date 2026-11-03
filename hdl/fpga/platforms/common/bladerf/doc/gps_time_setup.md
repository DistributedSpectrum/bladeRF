# GPS time over UART → bladeRF RX metadata (UBX-NAV-TIMEUTC)

The `gps_uart` module (`../vhdl/gps_uart.vhd`) receives time from a u-blox GPS over a
UART + PPS and stamps every RX DMA block's metadata with an NTP-style UTC timestamp:

```
timestamp[63:32] = Unix epoch seconds (UTC)
timestamp[31:0]  = sub-second fraction = sys_clock (80 MHz) ticks since the last PPS edge
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
| GND                        | —         | —  | 59 / 60 |

- **Baud: 110803.** The `uart_rx` bit period is `2 * C_GPS_DIV_CNT_115k = 2 * 361 = 722`
  `sys_clock` cycles; at 80 MHz that is 9.025 µs → **110803 baud**. The GPS UART must be set
  to this exact rate (see below) so there is no baud error.
- Do **not** connect the GPS's 3.3 V output pins to the bladeRF. Avoid connector pins
  57/58 (+5 V) and pin 1 (XB_VCCIO).

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
| J2 or J5 pin 4 | GND         | 59 / 60 |

(GW16143 J2 pin 3 "Interrupt" = F9P EXTINT, an input; not used by this design.)

---

## Host-side decode of `bladerf_metadata.timestamp`

```
ts    = metadata.timestamp;            // uint64, from SC16_Q11_META blocks
epoch = ts >> 32;                       // Unix epoch seconds (UTC); 0 => GPS not locked yet
frac  = ts & 0xffffffff;                // sys_clock ticks since PPS
block_start_utc   = epoch + frac / 80e6;
utc(sample n)     = block_start_utc + n / sample_rate;
```

Correlate `block_start_utc` with the gpsd position/time fix read over the F9P's USB
interface (`/dev/ttyACM0`) to tag each IQ capture with the sensor's UTC time and location.

`epoch == 0` means the GPS has not yet delivered a valid UTC (no lock / `validUTC=0`);
treat such blocks as untimed.

---

## Timestamp integrity across the clock domains (`ts_cdc`)

The 64-bit timestamp is generated in the **`sys_clock`** domain (the fraction counter
free-runs at 80 MHz) but latched by the RX block in the **`rx_clock`** (AD9361) domain at
the start of each DMA block. Sampling a 64-bit word across asynchronous clocks without
synchronization **tears**: when the latch instant coincides with the per-second update
(PPS rollover — `epoch += 1` and `frac → 0` on the same cycle) some bits are captured
before the update and some after, producing a garbage timestamp.

`ts_cdc` (`../vhdl/ts_cdc.vhd`) makes this crossing safe with a toggle/ack handshake: the
source latches a snapshot into a holding register and flips a request toggle, and does not
latch the next snapshot until the destination has acknowledged — so the held value is
guaranteed stable while the destination captures it. `U_rx` therefore always latches a
**coherent** 64-bit value. The refresh latency is one handshake round trip (a few cycles each
way, well under a microsecond), far finer than a DMA block. It is instantiated in
`bladerf-hosted.vhd` between `gps_uart.timestamp` (sys_clock) and `U_rx.rx_timestamp`
(rx_clock).

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
`frac` is monotonic between PPS edges (it advances exactly 40 ticks per sample at 2 Msps).

Verified on hardware 2026-11-03 (build `hostedxA4-2026-11-03_11.39.45`): epoch steps `+1`
exactly once per second at the PPS, no spurious edges, `epoch == 0` never occurs, the first
message of a capture is already timed, and the epoch matches an NTP-synced host to the second.

**Known startup artifact:** the first ~3 messages (~3 ms) of the *first* stream after an FPGA
load carry garbage headers (epoch ≈ 4.28e9, i.e. year 2105) — stale FX3/GPIF buffer contents
on the first enable. Every later stream start is clean from message 0. Hosts should discard
any message whose epoch is implausible (e.g. `epoch > 0x7fffffff`), which also covers this.
