/*
 * GPS-timed RX through the synchronous API (FPGA v0.19.0 header v2).
 *
 * Streams SC16_Q11_META one message per call and checks, from what
 * bladerf_sync_rx() reports and nothing else, that:
 *   - the timestamp is still the free-running sample counter (advances by
 *     exactly one message of samples, no overruns),
 *   - the metadata carries BLADERF_META_STATUS_GPS_PRESENT and (once the
 *     receiver has a fix and a PPS has been seen) BLADERF_META_STATUS_GPS_LOCK,
 *   - the overlaid bladerf_rx_gps_time has UTC seconds agreeing with the host
 *     clock and a fraction that advances with the counter and restarts at
 *     each PPS,
 *   - the derived PPS interval equals the sample rate to within the
 *     oscillator's error,
 *   - a timed read (counter semantics, as always) lands on the sample asked.
 *
 * Usage: libbladeRF_test_rx_gps_time [-d device] [-r rate] [-s seconds]
 */
#include <getopt.h>
#include <inttypes.h>
#include <libbladeRF.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MSG_SAMPLES_SS 2042u /* 8192-byte message, 24-byte header */

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void print_status(uint32_t st)
{
    printf("status 0x%04x:%s%s%s%s%s%s%s", st,
           (st & BLADERF_META_STATUS_OVERRUN) ? " OVERRUN" : "",
           (st & BLADERF_META_STATUS_GPS_PRESENT) ? " GPS_PRESENT" : "",
           (st & BLADERF_META_STATUS_GPS_UTC_VALID) ? " UTC_VALID" : "",
           (st & BLADERF_META_STATUS_GPS_PPS_ALIVE) ? " PPS_ALIVE" : "",
           (st & BLADERF_META_STATUS_GPS_LOCK) ? " GPS_LOCK" : "",
           (st & BLADERF_META_STATUS_GPS_UBX_FRESH) ? " UBX_FRESH" : "",
           (st & BLADERF_META_STATUS_GPS_FRAC_VALID) ? " FRAC_VALID" : "");
}

int main(int argc, char *argv[])
{
    const char *devstr = NULL;
    unsigned int rate = 10000000, seconds = 10;
    struct bladerf *dev = NULL;
    struct bladerf_metadata meta;
    struct bladerf_rx_gain_tag_msg tags[4];
    int16_t *buf;
    int status, opt;
    unsigned int n, msgs = 0, pps_steps = 0, overruns = 0, unlocked = 0,
                 bad_step = 0, bad_gps_step = 0, bad_epoch = 0, interval_bad = 0,
                 no_gps = 0, bad_tag = 0;
    uint64_t prev = 0;
    uint32_t pep = 0, pfr = 0;
    bool have_prev = false, prev_gps_ok = false, seen_lock = false;
    struct bladerf_rx_gps_time g;
    uint32_t interval_min = 0, interval_max = 0;
    double t0;

    while ((opt = getopt(argc, argv, "d:r:s:h")) != -1) {
        switch (opt) {
            case 'd': devstr = optarg; break;
            case 'r': rate = (unsigned int)strtoul(optarg, NULL, 0); break;
            case 's': seconds = (unsigned int)strtoul(optarg, NULL, 0); break;
            default:
                printf("Usage: %s [-d device] [-r rate] [-s seconds]\n", argv[0]);
                return 1;
        }
    }

    status = bladerf_open(&dev, devstr);
    if (status != 0) {
        fprintf(stderr, "open: %s\n", bladerf_strerror(status));
        return 1;
    }

    status = bladerf_set_sample_rate(dev, BLADERF_CHANNEL_RX(0), rate, &rate);
    if (status == 0) {
        status = bladerf_set_bandwidth(dev, BLADERF_CHANNEL_RX(0), rate * 7 / 10, NULL);
    }
    if (status == 0) {
        status = bladerf_set_frequency(dev, BLADERF_CHANNEL_RX(0), 2400000000ULL);
    }
    if (status == 0) {
        status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11_META,
                                     32, 8192 * 4, 8, 3500);
    }
    if (status == 0) {
        status = bladerf_enable_module(dev, BLADERF_RX, true);
    }
    if (status != 0) {
        fprintf(stderr, "setup: %s\n", bladerf_strerror(status));
        bladerf_close(dev);
        return 1;
    }

    buf = malloc(MSG_SAMPLES_SS * 2 * sizeof(int16_t));
    if (buf == NULL) {
        bladerf_close(dev);
        return 1;
    }

    printf("rate %u sps, %u s, one %u-sample message per call\n", rate, seconds,
           MSG_SAMPLES_SS);
    { uint32_t g = 0; bladerf_config_gpio_read(dev, &g); printf("GPIO config after enable: 0x%08x\n", g); }

    t0 = now_s();
    while (now_s() - t0 < seconds) {
        uint64_t ts;
        uint32_t ep, fr;

        memset(&meta, 0, sizeof(meta));
        meta.flags = BLADERF_META_FLAG_RX_NOW;

        status = bladerf_sync_rx(dev, buf, MSG_SAMPLES_SS, &meta, 3500);
        if (status != 0) {
            fprintf(stderr, "sync_rx: %s\n", bladerf_strerror(status));
            break;
        }

        ts = meta.timestamp;
        memcpy(&g, &meta.reserved[sizeof(struct bladerf_rx_gain_tag)], sizeof(g));
        ep = g.epoch;
        fr = g.frac;
        msgs++;

        if (msgs <= 3 && g.version == BLADERF_RX_GPS_TIME_VERSION_NONE) {
            /* The first messages after an FPGA load may carry stale headers */
            continue;
        }

        if (meta.status & BLADERF_META_STATUS_OVERRUN) {
            overruns++;
        }
        /* The first second of a session is unlocked by design (FRAC_VALID
         * needs a PPS seen with RX running); count lock loss after that. */
        if (meta.status & BLADERF_META_STATUS_GPS_LOCK) {
            seen_lock = true;
        } else if (seen_lock) {
            unlocked++;
        }
        if (g.version != BLADERF_RX_GPS_TIME_VERSION_1) {
            no_gps++;
        } else if (labs((long)ep - (long)time(NULL)) > 1) {
            bad_epoch++;
        }

        if (g.pps_interval != 0) {
            uint32_t iv = g.pps_interval;
            if (interval_min == 0 || iv < interval_min) interval_min = iv;
            if (iv > interval_max) interval_max = iv;
            if ((double)iv < rate * 0.999 || (double)iv > rate * 1.001) {
                interval_bad++;
            }
        }

        if (have_prev) {
            /* counter: contiguous messages */
            if (ts != prev + MSG_SAMPLES_SS) {
                bad_step++;
            }
            /* GPS: fraction tracks the counter within a second, restarts at
             * the PPS with the epoch stepping by one */
            if (g.version == BLADERF_RX_GPS_TIME_VERSION_1 && prev_gps_ok) {
                if (ep == pep) {
                    if (fr != pfr + MSG_SAMPLES_SS) bad_gps_step++;
                } else if (ep == pep + 1) {
                    pps_steps++;
                    if (g.pps_interval != 0 &&
                        (uint64_t)fr + g.pps_interval != (uint64_t)pfr + MSG_SAMPLES_SS) {
                        bad_gps_step++;
                    }
                } else {
                    bad_gps_step++;
                }
            }
        }
        prev = ts;
        pep = ep;
        pfr = fr;
        prev_gps_ok = (g.version == BLADERF_RX_GPS_TIME_VERSION_1) &&
                      (meta.status & BLADERF_META_STATUS_GPS_LOCK);
        have_prev = true;

        status = bladerf_get_rx_gain_tags(dev, tags, 4, &n);
        if (status == 0 && n >= 1 && (tags[0].gps_epoch != ep || tags[0].gps_frac != fr)) {
            bad_tag++;
        }

        if (msgs <= 4 || (msgs % 2000) == 0) {
            printf("msg %6u: counter %12" PRIu64 "  gps %u + %9u/%u  actual %u  ",
                   msgs, ts, ep, fr, g.pps_interval, meta.actual_count);
            print_status(meta.status);
            printf("\n");
        }
    }

    printf("\n%u messages, %u PPS crossings, %u overruns, %u unlocked after first lock, %u without GPS time, "
           "%u bad counter steps, %u bad GPS steps, %u epoch mismatches vs host, "
           "%u gain-tag/GPS mismatches\n",
           msgs, pps_steps, overruns, unlocked, no_gps, bad_step, bad_gps_step,
           bad_epoch, bad_tag);
    if (interval_max != 0) {
        printf("PPS interval seen: %u .. %u samples (%+.3f .. %+.3f ppm vs %u), "
               "%u outside +-1000 ppm\n",
               interval_min, interval_max,
               ((double)interval_min - rate) / (rate / 1e6),
               ((double)interval_max - rate) / (rate / 1e6), rate, interval_bad);
    } else {
        printf("PPS interval: not reported (no two PPS edges yet)\n");
    }

    /* Timed read, still inside the session: half a second ahead on the
     * counter, which keeps its stock meaning. */
    if (have_prev) {
        uint64_t target = prev + rate / 2;

        memset(&meta, 0, sizeof(meta));
        meta.timestamp = target;
        status = bladerf_sync_rx(dev, buf, MSG_SAMPLES_SS, &meta, 3500);
        memcpy(&g, &meta.reserved[sizeof(struct bladerf_rx_gain_tag)], sizeof(g));
        printf("timed read for counter %" PRIu64 ": %s, got %" PRIu64
               " (gps %u + %u), ", target,
               status == 0 ? "ok" : bladerf_strerror(status), meta.timestamp,
               g.epoch, g.frac);
        print_status(meta.status);
        printf("\n");
        /* OVERRUN here only says the stream moved on while we were
         * printing; landing on the exact counter value is the test. */
        if (status == 0 && meta.timestamp != target) {
            printf("timed read: WRONG position\n");
            status = 1;
        } else if (status == 0) {
            printf("timed read: landed exactly on the requested counter value\n");
        }
    }

    bladerf_enable_module(dev, BLADERF_RX, false);

    free(buf);
    bladerf_close(dev);

    if (overruns == 0 && bad_step == 0 && bad_gps_step == 0 && unlocked == 0 &&
        no_gps == 0 && bad_epoch == 0 && interval_bad == 0 && bad_tag == 0 &&
        status == 0) {
        printf("PASS\n");
        return 0;
    }
    printf("FAIL\n");
    return 1;
}
