/* GPS UART pin self-check.
 * Streams the bladeRF RX metadata and reports whether the gps_uart timestamp is
 * decoding GPS UTC (=> UART RX pigtail on the right pin) and whether PPS is
 * resetting the sub-second fraction (=> PPS pigtail on the right pin).
 * Assumes the gps_uart FPGA is already loaded (the .sh wrapper does that).
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <libbladeRF.h>

int main(void)
{
    struct bladerf *dev = NULL;
    int st;

    st = bladerf_open(&dev, NULL);
    if (st) { fprintf(stderr, "open: %s\n", bladerf_strerror(st)); return 2; }

    bladerf_set_sample_rate(dev, BLADERF_CHANNEL_RX(0), 2000000u, NULL);
    bladerf_set_frequency(dev, BLADERF_CHANNEL_RX(0), 2400000000ULL);

    st = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11_META,
                             16, 8192, 8, 3500);
    if (st) { fprintf(stderr, "sync_config: %s\n", bladerf_strerror(st)); return 2; }
    st = bladerf_enable_module(dev, BLADERF_RX, true);
    if (st) { fprintf(stderr, "enable: %s\n", bladerf_strerror(st)); return 2; }

    const unsigned N = 65536;                 /* ~32.7 ms per block @ 2 Msps */
    int16_t *buf = malloc((size_t)N * 2 * sizeof(int16_t));
    uint32_t max_epoch = 0, last_frac = 0;
    int pps_seen = 0, first = 1;
    unsigned long nblk = 0;

    printf("Streaming RX metadata for ~6 s ...\n");
    time_t t0 = time(NULL);
    while (time(NULL) - t0 < 6) {
        struct bladerf_metadata meta;
        memset(&meta, 0, sizeof(meta));
        meta.flags = BLADERF_META_FLAG_RX_NOW;
        st = bladerf_sync_rx(dev, buf, N, &meta, 2000);
        if (st) { fprintf(stderr, "rx: %s\n", bladerf_strerror(st)); break; }

        uint32_t epoch = (uint32_t)(meta.timestamp >> 32);
        uint32_t frac  = (uint32_t)(meta.timestamp & 0xffffffffULL);
        if (epoch > max_epoch) max_epoch = epoch;
        if (!first && frac < last_frac) pps_seen = 1;   /* fraction dropped => PPS reset */
        last_frac = frac;
        first = 0;
        nblk++;
    }
    bladerf_enable_module(dev, BLADERF_RX, false);
    free(buf);

    printf("\n==================== GPS UART pin check ====================\n");
    printf("blocks read: %lu\n", nblk);

    if (max_epoch) {
        time_t t = (time_t)max_epoch; struct tm g; gmtime_r(&t, &g);
        char s[64]; strftime(s, sizeof s, "%Y-%m-%d %H:%M:%S", &g);
        printf("UART RX (pin 56): OK   -- decoded GPS UTC %s (epoch %u)\n", s, max_epoch);
    } else {
        printf("UART RX (pin 56): FAIL -- no UTC decoded (epoch stayed 0)\n");
        printf("   -> put the YELLOW pigtail (GW16143 J5 pin2, GPS TX) on bladeRF pin 56,\n");
        printf("      and make sure the GPS has a fix with validUTC=1.\n");
    }

    printf("PPS      (pin 50): %s\n",
           pps_seen ? "OK   -- 1 Hz reset of the fraction seen"
                    : "FAIL -- no 1 Hz reset seen (put PPS on bladeRF pin 50; needs a GPS fix)");

    printf("(UART TX pin 54 is not functionally testable here -- the F9P is configured\n");
    printf(" over USB/ubxtool and ignores the bladeRF's TX.)\n");
    printf("===========================================================\n");

    bladerf_close(dev);
    return max_epoch ? 0 : 1;
}
