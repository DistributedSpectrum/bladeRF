/* GPS UART pin self-check.
 * Reads the RAW 128-bit metadata headers of the RX stream (async API) and
 * reports whether the gps_uart timestamp is decoding GPS UTC (=> UART RX
 * pigtail on the right pin) and whether PPS is resetting the sub-second
 * fraction (=> PPS pigtail on the right pin), plus timestamp integrity.
 *
 * Raw headers are used on purpose: libbladeRF's sync layer treats
 * metadata.timestamp as a sample counter and does arithmetic on it, so values
 * read through bladerf_sync_rx() can be mangled or zero-filled.
 *
 * Assumes the gps_uart FPGA is already loaded (the .sh wrapper does that).
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <libbladeRF.h>

#define MSG_BYTES 8192u   /* FX3 fw >= 2.5.0 on SuperSpeed */
#define HDR_BYTES 24u     /* FPGA >= 0.19.0: 16-byte header + status + PPS interval */
#define RUN_SECONDS 6

static time_t t0;
static unsigned long nmsg, nzero, ngood, nhigh, nonmono, fracbad, pps_steps;
static long worst_high;
static uint32_t first_epoch, last_epoch, prev_epoch, prev_frac;

static void *cb(struct bladerf *dev, struct bladerf_stream *st,
                struct bladerf_metadata *m, void *samples, size_t n, void *ud)
{
    (void)dev; (void)st; (void)m; (void)ud;
    const uint8_t *b = samples;
    size_t bytes = n * 4;
    for (size_t off = 0; off + HDR_BYTES <= bytes; off += MSG_BYTES) {
        uint64_t ts; memcpy(&ts, b + off + 4, 8);            /* stock sample counter */
        uint32_t gw, ep; memcpy(&gw, b + off + 16, 4); memcpy(&ep, b + off + 20, 4);
        uint32_t fr = gw & 0x03ffffffu;                       /* GPS: samples since PPS */
        long d = (long)ep - (long)time(NULL);

        if (nmsg == 0) first_epoch = ep;
        else {
            if (ep == prev_epoch + 1 && fr < prev_frac) pps_steps++;        /* clean PPS */
            if (ep < prev_epoch || (ep == prev_epoch && fr < prev_frac)) nonmono++;
            if (ep > prev_epoch + 1) nonmono++;
        }
        if (fr > 64512000u) fracbad++;   /* fraction = samples since PPS; > 61.44 MSps*1.05 is impossible */
        if (ep == 0) nzero++;
        else if (d >= -2 && d <= 1) ngood++;
        else if (d > 1) { nhigh++; if (d > worst_high) worst_high = d; }

        last_epoch = ep; prev_epoch = ep; prev_frac = fr; nmsg++;
    }
    return (time(NULL) - t0 >= RUN_SECONDS) ? BLADERF_STREAM_SHUTDOWN : samples;
}

int main(void)
{
    struct bladerf *dev = NULL; struct bladerf_stream *st; void **bufs; int s;

    s = bladerf_open(&dev, NULL);
    if (s) { fprintf(stderr, "open: %s\n", bladerf_strerror(s)); return 2; }
    bladerf_set_sample_rate(dev, BLADERF_CHANNEL_RX(0), 2000000u, NULL);
    bladerf_set_frequency(dev, BLADERF_CHANNEL_RX(0), 2400000000ULL);

    const size_t spb = (MSG_BYTES / 4) * 16;
    s = bladerf_init_stream(&st, dev, cb, &bufs, 16, BLADERF_FORMAT_SC16_Q11_META, spb, 8, NULL);
    if (s) { fprintf(stderr, "init_stream: %s\n", bladerf_strerror(s)); return 2; }
    s = bladerf_enable_module(dev, BLADERF_RX, true);
    if (s) { fprintf(stderr, "enable: %s\n", bladerf_strerror(s)); return 2; }

    printf("Reading raw RX metadata headers for ~%d s ...\n", RUN_SECONDS);
    t0 = time(NULL);
    s = bladerf_stream(st, BLADERF_RX_X1);
    bladerf_enable_module(dev, BLADERF_RX, false);
    if (s) fprintf(stderr, "stream: %s\n", bladerf_strerror(s));

    time_t hnow = time(NULL);
    int uart_ok = (last_epoch != 0) && (labs((long)last_epoch - (long)hnow) <= 2);
    int pps_ok  = pps_steps > 0;

    printf("\n==================== GPS UART pin check ====================\n");
    printf("messages read: %lu   (first message %s)\n", nmsg,
           first_epoch ? "already timed" : "untimed (epoch 0)");

    if (last_epoch) {
        time_t t = (time_t)last_epoch; struct tm g; gmtime_r(&t, &g);
        char ss[64]; strftime(ss, sizeof ss, "%Y-%m-%d %H:%M:%S", &g);
        struct tm h; gmtime_r(&hnow, &h); char hs[64]; strftime(hs, sizeof hs, "%H:%M:%S", &h);
        printf("UART RX (pin 56): %s -- decoded GPS UTC %s (epoch %u); host now %s UTC\n",
               uart_ok ? "OK  " : "FAIL", ss, last_epoch, hs);
    } else {
        printf("UART RX (pin 56): FAIL -- no UTC decoded (epoch stayed 0)\n");
        printf("   -> put the YELLOW pigtail (GW16143 J5 pin2, GPS TX) on bladeRF pin 56,\n");
        printf("      and make sure the GPS has a fix with validUTC=1.\n");
    }

    printf("PPS      (pin 50): %s\n",
           pps_ok ? "OK   -- 1 Hz epoch step with fraction reset seen"
                  : "FAIL -- no clean 1 Hz step seen (put PPS on bladeRF pin 50; needs a GPS fix)");

    printf("Integrity: %lu/%lu msgs correct, %lu epoch==0, %lu high outliers (worst +%lds), "
           "%lu non-monotonic, %lu frac>1.05s\n",
           ngood, nmsg, nzero, nhigh, worst_high, nonmono, fracbad);
    printf("   -> %s\n", (nonmono == 0 && fracbad == 0 && nhigh == 0)
           ? "OK: timestamps clean (no tears, no spurious PPS edges)"
           : "WARN: timestamp glitches present (spurious PPS edges or a torn crossing)");

    printf("(UART TX pin 54 is not functionally testable here -- the F9P is configured\n");
    printf(" over USB/ubxtool and ignores the bladeRF's TX.)\n");
    printf("===========================================================\n");

    bladerf_deinit_stream(st);
    bladerf_close(dev);
    return (uart_ok && pps_ok) ? 0 : 1;
}
