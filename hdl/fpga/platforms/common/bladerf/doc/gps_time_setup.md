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
