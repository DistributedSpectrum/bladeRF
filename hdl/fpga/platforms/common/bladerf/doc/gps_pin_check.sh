#!/bin/sh
# GPS UART pin self-check.
#   Loads the gps_uart FPGA, streams RX metadata, and tells you whether the GPS
#   UTC decodes (UART RX pin 56) and PPS resets the fraction (pin 50).
# Usage: ./gps_pin_check.sh [path/to/hostedxA4.rbf]
#   (defaults to the newest build under hdl/quartus)

REPO=/home/subrad/ds-bladerf/bladeRF
OUT=$REPO/host/build/output                 # the clean (un-hacked) libbladeRF + bladeRF-cli
HERE=$(dirname "$0")
BIN=$HERE/gps_pin_check
RBF=${1:-$(ls -dt "$REPO"/hdl/quartus/hostedxA4-*/hostedxA4.rbf 2>/dev/null | head -1)}

if [ -z "$RBF" ] || [ ! -f "$RBF" ]; then
    echo "ERROR: no gps_uart .rbf found; pass one as the first argument." >&2
    exit 2
fi

# build the checker once (against the clean libbladeRF)
if [ ! -x "$BIN" ] || [ "$HERE/gps_pin_check.c" -nt "$BIN" ]; then
    echo "Building gps_pin_check ..."
    gcc -O2 -I"$REPO/host/libraries/libbladeRF/include" -o "$BIN" "$HERE/gps_pin_check.c" \
        -L"$OUT" -lbladeRF -Wl,-rpath,"$OUT" || { echo "build failed" >&2; exit 2; }
fi

echo "Loading FPGA: $RBF"
env -u BLADERF_FORCE_NO_FPGA_PRESENT LD_LIBRARY_PATH="$OUT" \
    "$OUT/bladeRF-cli" -l "$RBF" >/dev/null 2>&1 || { echo "FPGA load failed" >&2; exit 2; }

env -u BLADERF_FORCE_NO_FPGA_PRESENT LD_LIBRARY_PATH="$OUT" "$BIN"
