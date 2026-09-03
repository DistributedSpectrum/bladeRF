/*
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2026 Distributed Spectrum
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Exercises the RX time marker (FPGA >= v0.18.0) and reports what it buys.
 *
 * Each exchange reads the host clock, flips the marker, reads the clock again,
 * then receives one message at a time until the header flag carries the new
 * value. That header's timestamp is bounded by
 *
 *     ts(T_before) <= header.timestamp <= ts(T_after) + samples_per_message
 *
 * so every exchange yields one (host time, FPGA ticks) pair with a known
 * half-window. After N of them the pairs are fitted to wall = a + b * ticks,
 * which gives the offset and the drift (skew) between the host clock and the
 * FPGA sample counter, and the residuals say whether the per-exchange bound
 * was honest.
 *
 * This is also the hardware acceptance test for the FPGA feature: a marker
 * that never comes back within the wait limit is a failure, and so is a
 * message-count bound that the fit residuals violate.
 */

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <libbladeRF.h>

#define TIMEOUT_MS      1000
#define NUM_BUFFERS     16
#define NUM_TRANSFERS   8
#define MAX_WAIT_MSGS   4096

struct opts {
    const char *devstr;
    double freq_hz;
    double rate_hz;
    unsigned int exchanges;
    unsigned int interval_ms;
    bladerf_log_level verbosity;
};

struct exchange {
    int64_t t_mid_ns;       /* CLOCK_REALTIME midpoint of the write */
    int64_t rtt_ns;         /* T_after - T_before */
    uint64_t ts_m;          /* timestamp of the first header carrying the new value */
    unsigned int waited;    /* messages read before it showed up */
    bool valid;
};

static int64_t now_ns(clockid_t clk)
{
    struct timespec ts;
    clock_gettime(clk, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void usage(const char *argv0)
{
    printf("Usage: %s [options]\n", argv0);
    printf("  -d <device>     Device string (default: first available)\n");
    printf("  -f <hz>         RX frequency (default: 915e6)\n");
    printf("  -s <hz>         Sample rate (default: 10e6)\n");
    printf("  -n <count>      Number of exchanges (default: 50)\n");
    printf("  -i <ms>         Pause between exchanges (default: 200)\n");
    printf("  -v <level>      libbladeRF verbosity 0-6 (default: 3)\n");
    printf("  -h              This text\n");
}

static int parse_opts(int argc, char **argv, struct opts *o)
{
    int c;

    o->devstr      = NULL;
    o->freq_hz     = 915e6;
    o->rate_hz     = 10e6;
    o->exchanges   = 50;
    o->interval_ms = 200;
    o->verbosity   = BLADERF_LOG_LEVEL_INFO;

    while ((c = getopt(argc, argv, "d:f:s:n:i:v:h")) != -1) {
        switch (c) {
            case 'd': o->devstr      = optarg; break;
            case 'f': o->freq_hz     = strtod(optarg, NULL); break;
            case 's': o->rate_hz     = strtod(optarg, NULL); break;
            case 'n': o->exchanges   = (unsigned int)strtoul(optarg, NULL, 0); break;
            case 'i': o->interval_ms = (unsigned int)strtoul(optarg, NULL, 0); break;
            case 'v': o->verbosity   = (bladerf_log_level)strtoul(optarg, NULL, 0); break;
            case 'h': usage(argv[0]); return 1;
            default:  usage(argv[0]); return -1;
        }
    }

    if (o->exchanges < 2) {
        fprintf(stderr, "Need at least 2 exchanges for a fit.\n");
        return -1;
    }

    return 0;
}

/* Payload samples per USB message for SC16_Q11_META. The header is 16 bytes,
 * a message is 8192 bytes on SuperSpeed and 4096 on Hi-Speed with FX3 >=
 * 2.5.0 (2048 before that). Verified against the gain-tag message count below
 * rather than trusted. */
static unsigned int guess_samples_per_msg(struct bladerf *dev)
{
    struct bladerf_version fw;
    bladerf_dev_speed speed = bladerf_device_speed(dev);

    if (speed == BLADERF_DEVICE_SPEED_SUPER) {
        return 2044;
    }

    if (bladerf_fw_version(dev, &fw) == 0 &&
        (fw.major > 2 || (fw.major == 2 && fw.minor >= 5))) {
        return 1020;
    }

    return 508;
}

static int rx_one_msg(struct bladerf *dev, int16_t *buf, unsigned int spm,
                      struct bladerf_metadata *meta)
{
    memset(meta, 0, sizeof(*meta));
    meta->flags = BLADERF_META_FLAG_RX_NOW;
    return bladerf_sync_rx(dev, buf, spm, meta, TIMEOUT_MS);
}

/* Least-squares wall = a + b * ticks over the valid exchanges. */
static void fit(const struct exchange *ex, unsigned int n, double fs,
                double *a_ns, double *b_ns_per_tick, double *rms_ns,
                double *max_abs_ns, unsigned int *used)
{
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    double x0 = 0, y0 = 0;
    unsigned int i, m = 0;
    bool have_origin = false;

    /* Work relative to the first valid point to keep the sums small. */
    for (i = 0; i < n; i++) {
        if (!ex[i].valid) continue;
        if (!have_origin) {
            x0 = (double)ex[i].ts_m;
            y0 = (double)ex[i].t_mid_ns;
            have_origin = true;
        }
        double x = (double)ex[i].ts_m - x0;
        double y = (double)ex[i].t_mid_ns - y0;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
        m++;
    }

    *used = m;
    if (m < 2) {
        *a_ns = *b_ns_per_tick = *rms_ns = *max_abs_ns = NAN;
        return;
    }

    double denom = m * sxx - sx * sx;
    double b = (denom != 0.0) ? (m * sxy - sx * sy) / denom : 1e9 / fs;
    double a = (sy - b * sx) / m;

    double ss = 0, mx = 0;
    for (i = 0; i < n; i++) {
        if (!ex[i].valid) continue;
        double x = (double)ex[i].ts_m - x0;
        double y = (double)ex[i].t_mid_ns - y0;
        double r = y - (a + b * x);
        ss += r * r;
        if (fabs(r) > mx) mx = fabs(r);
    }

    *a_ns          = a + y0 - b * x0;   /* back to absolute */
    *b_ns_per_tick = b;
    *rms_ns        = sqrt(ss / m);
    *max_abs_ns    = mx;
}

int main(int argc, char **argv)
{
    struct opts o;
    struct bladerf *dev = NULL;
    struct bladerf_version fpga;
    struct bladerf_metadata meta;
    struct exchange *ex = NULL;
    int16_t *buf = NULL;
    unsigned int spm, buf_samples, i, n_tags;
    bladerf_sample_rate actual_rate;
    bool marker, cur;
    int status, ret = 1;

    status = parse_opts(argc, argv, &o);
    if (status != 0) {
        return status < 0 ? 1 : 0;
    }

    bladerf_log_set_verbosity(o.verbosity);

    status = bladerf_open(&dev, o.devstr);
    if (status != 0) {
        fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(status));
        return 1;
    }

    bladerf_fpga_version(dev, &fpga);
    printf("FPGA v%u.%u.%u, %s\n", fpga.major, fpga.minor, fpga.patch,
           bladerf_device_speed(dev) == BLADERF_DEVICE_SPEED_SUPER ? "SuperSpeed" :
           bladerf_device_speed(dev) == BLADERF_DEVICE_SPEED_HIGH  ? "Hi-Speed" : "unknown speed");

    status = bladerf_get_rx_time_marker(dev, &marker);
    if (status == BLADERF_ERR_UNSUPPORTED) {
        fprintf(stderr, "This FPGA does not echo the RX time marker (needs v0.18.0 or later).\n");
        goto out;
    } else if (status != 0) {
        fprintf(stderr, "bladerf_get_rx_time_marker: %s\n", bladerf_strerror(status));
        goto out;
    }
    printf("Marker currently %u\n", marker ? 1 : 0);

    status = bladerf_set_sample_rate(dev, BLADERF_CHANNEL_RX(0),
                                     (bladerf_sample_rate)o.rate_hz, &actual_rate);
    if (status != 0) {
        fprintf(stderr, "set_sample_rate: %s\n", bladerf_strerror(status));
        goto out;
    }
    status = bladerf_set_bandwidth(dev, BLADERF_CHANNEL_RX(0),
                                   (bladerf_bandwidth)actual_rate, NULL);
    if (status != 0) {
        fprintf(stderr, "set_bandwidth: %s\n", bladerf_strerror(status));
        goto out;
    }
    status = bladerf_set_frequency(dev, BLADERF_CHANNEL_RX(0),
                                   (bladerf_frequency)o.freq_hz);
    if (status != 0) {
        fprintf(stderr, "set_frequency: %s\n", bladerf_strerror(status));
        goto out;
    }

    spm = guess_samples_per_msg(dev);
    buf_samples = (spm + 4) * 4;    /* four whole messages, a multiple of 1024 */

    status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11_META,
                                 NUM_BUFFERS, buf_samples, NUM_TRANSFERS, TIMEOUT_MS);
    if (status != 0) {
        fprintf(stderr, "sync_config: %s\n", bladerf_strerror(status));
        goto out;
    }

    buf = malloc(sizeof(int16_t) * 2 * spm);
    ex  = calloc(o.exchanges, sizeof(*ex));
    if (buf == NULL || ex == NULL) {
        fprintf(stderr, "Out of memory\n");
        goto out;
    }

    status = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), true);
    if (status != 0) {
        fprintf(stderr, "enable_module: %s\n", bladerf_strerror(status));
        goto out;
    }

    /* Prime the stream and confirm one read == one message, using the gain
     * tag message count as the witness. */
    for (i = 0; i < 8; i++) {
        status = rx_one_msg(dev, buf, spm, &meta);
        if (status != 0) {
            fprintf(stderr, "sync_rx (prime): %s\n", bladerf_strerror(status));
            goto out;
        }
    }
    if (bladerf_get_rx_gain_tags(dev, NULL, 0, &n_tags) == 0 && n_tags != 1) {
        fprintf(stderr, "A %u-sample read spanned %u messages; samples-per-message "
                        "guess is wrong for this USB speed/firmware.\n", spm, n_tags);
        goto out;
    }

    printf("Sample rate %u Hz, %u samples per message (%.1f us)\n\n",
           actual_rate, spm, 1e6 * spm / actual_rate);
    printf("%4s  %8s  %6s  %16s  %10s\n", "#", "rtt_us", "msgs", "ts_m", "bound_us");

    cur = marker;
    for (i = 0; i < o.exchanges; i++) {
        struct exchange *e = &ex[i];
        bool target = !cur;
        bool overran = false;
        int64_t t0, t1;
        unsigned int w;

        t0 = now_ns(CLOCK_REALTIME);
        status = bladerf_set_rx_time_marker(dev, target);
        t1 = now_ns(CLOCK_REALTIME);
        if (status != 0) {
            fprintf(stderr, "set_rx_time_marker: %s\n", bladerf_strerror(status));
            goto out;
        }

        e->t_mid_ns = t0 + (t1 - t0) / 2;
        e->rtt_ns   = t1 - t0;
        e->valid    = false;

        for (w = 0; w < MAX_WAIT_MSGS; w++) {
            bool flag;

            status = rx_one_msg(dev, buf, spm, &meta);
            if (status != 0) {
                fprintf(stderr, "sync_rx: %s\n", bladerf_strerror(status));
                goto out;
            }
            if (meta.status & BLADERF_META_STATUS_OVERRUN) {
                /* Samples were dropped somewhere before this header, so the
                 * message-count bound no longer holds. Keep reading until the
                 * echo arrives so the next exchange starts from a clean
                 * stream, but do not record this one. */
                if (!overran) {
                    fprintf(stderr, "  overrun during exchange %u, discarding it\n", i);
                }
                overran = true;
            }

            flag = (meta.status & BLADERF_META_FLAG_RX_HW_TIME_MARK) != 0;
            if (flag == target) {
                e->ts_m   = meta.timestamp;
                e->waited = w;
                e->valid  = !overran;
                break;
            }
        }

        if (!e->valid) {
            if (!overran) {
                fprintf(stderr, "  exchange %u: marker %u never echoed within %u messages\n",
                        i, target ? 1 : 0, MAX_WAIT_MSGS);
            }
        } else {
            double bound_us = (e->rtt_ns / 2) / 1e3 + 1e6 * spm / actual_rate;
            printf("%4u  %8.1f  %6u  %16" PRIu64 "  %10.1f\n", i, e->rtt_ns / 1e3,
                   e->waited, e->ts_m, bound_us);
        }

        cur = target;

        /* Space the exchanges out by continuing to read, not by sleeping: a
         * pause with the stream running overruns the buffer pool (at 10 Msps
         * the 16 x 8192-sample pool is 130 ms deep) and the next exchange
         * inherits the discontinuity. This is also how a real receiver would
         * use the marker -- interleaved with its normal reads. */
        if (o.interval_ms) {
            unsigned int drain = (unsigned int)((double)o.interval_ms * 1e-3 *
                                                actual_rate / spm);
            for (w = 0; w < drain; w++) {
                status = rx_one_msg(dev, buf, spm, &meta);
                if (status != 0) {
                    fprintf(stderr, "sync_rx (drain): %s\n", bladerf_strerror(status));
                    goto out;
                }
            }
        }
    }

    {
        double a, b, rms, mx;
        unsigned int used;
        double nominal = 1e9 / actual_rate;

        fit(ex, o.exchanges, actual_rate, &a, &b, &rms, &mx, &used);

        printf("\n%u of %u exchanges valid\n", used, o.exchanges);
        if (used >= 2) {
            double skew_ppm = (b / nominal - 1.0) * 1e6;
            double msg_us   = 1e6 * spm / actual_rate;
            unsigned int failed = 0;

            printf("fit: wall_ns = %.0f + %.6f * ticks\n", a, b);
            printf("     skew vs nominal %.3f ns/tick: %+.3f ppm\n", nominal, skew_ppm);
            printf("     residual rms %.1f us, max %.1f us\n", rms / 1e3, mx / 1e3);

            /* Every valid exchange's own bound should contain its residual.
             * The per-exchange half-window is rtt/2 + one message; a fit
             * residual beyond that means the bound is not honest. */
            for (i = 0; i < o.exchanges; i++) {
                if (!ex[i].valid) continue;
                double r = fabs((double)ex[i].t_mid_ns - (a + b * (double)ex[i].ts_m));
                double half = ex[i].rtt_ns / 2.0 + msg_us * 1e3;
                if (r > half) failed++;
            }
            printf("     %u exchange(s) with residual outside their own bound\n", failed);
            ret = (failed == 0) ? 0 : 2;
        }
    }

out:
    if (dev != NULL) {
        bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(dev);
    }
    free(buf);
    free(ex);
    return ret;
}
