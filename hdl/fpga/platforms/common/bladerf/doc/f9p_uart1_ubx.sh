#!/bin/sh
# Configure a u-blox ZED-F9P (e.g. Gateworks GW16143) UART1 to feed the bladeRF
# gps_uart UBX-NAV-TIMEUTC decoder. Settings are written to RAM+BBR+Flash (layer 7)
# so they persist across power cycles. The USB port (gpsd) is left untouched.
#
# Run on the host attached to the F9P (e.g. the Jetson), with gpsd/ubxtool available.
# See gps_time_setup.md for the full wiring and format notes.

set -x

ubxtool -z CFG-UART1-BAUDRATE,110803,7            # match bladeRF RX baud (2*361 @ 80 MHz)
ubxtool -z CFG-UART1OUTPROT-UBX,1,7               # allow UBX on UART1
ubxtool -z CFG-UART1OUTPROT-NMEA,0,7              # drop NMEA -> clean binary stream
ubxtool -z CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1,1,7   # NAV-TIMEUTC every epoch (1 Hz)
ubxtool -z CFG-TP-TP1_ENA,1,7                      # PPS/timepulse on (1 Hz, active after fix)

set +x
echo
echo "Verify:"
echo "  ubxtool -g CFG-UART1-BAUDRATE               # expect 110803"
echo "  ubxtool -g CFG-UART1OUTPROT-NMEA            # expect 0"
echo "  ubxtool -g CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1 # expect 1"
