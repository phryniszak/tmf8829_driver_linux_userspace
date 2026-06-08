
/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <getopt.h>

#include "tmf8829.h"
#include "tmf8829_driver.h"
#include "tmf8829_shim.h"
#include "tmf8829_frameparser.h"

#ifdef ENABLE_JSON_LOGGING
#include "tmf8829_json.h"
#endif
#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif

/* GPIO pin number used to drive the chip's enable line, set via the
 * GPIO_ENABLE_PIN CMake cache variable (defaults to 40). */
#ifndef GPIO_ENABLE_PIN
#define GPIO_ENABLE_PIN 40
#endif

static tmf8829_chip g_tof_chip;
static volatile sig_atomic_t g_stop_requested = 0;

static void usage(const char *program)
{
    printf("\n\n");
    printf("Help: %s [options]\n", program);
    printf("\n\n");
    printf("Options:\n");
    printf("    -m, --measurement         : perform measurement\n");
    printf("    -b, --bus <type>          : bus type: 0=i2c, 1=spi (default)\n");
    printf("    -d, --mode <mode>         : mode (0-13, default: 0):\n");
    printf("                                0:  8x8, 1: 8x8 long range, 2: 8x8 high accuracy, 3: 8x8 dual, 4: 8x8 long range dual\n");
    printf("                                5:  16x16,  6: 16x16 high accuracy,  7: 16x16 dual\n");
    printf("                                8:  32x32,  9: 32x32 high accuracy, 10: 32x32 dual\n");
    printf("                                11: 48x32, 12: 48x32 high accuracy, 13: 48x32 dual\n");
    printf("    -t, --timeout <seconds>   : timeout in seconds (default: 5), 0 means run forever\n");
    printf("    -i, --iterations <n>      : number of iterations (default: 1800)\n");
    printf("    -s, --shortiterations <n> : number of short iterations (default: 100)\n");
    printf("    -r, --threshold <n>       : confidence threshold (default: 6)\n");
    printf("    -p, --period <ms>         : period value (default: 33)\n");
#ifdef ENABLE_HISTOGRAM
    printf("    -h, --histogram           : enable histogram information in result (default: disabled)\n");
#endif
    printf("    -g, --signal              : enable signal information in result (default: disabled)\n");
    printf("    -n, --noise               : enable noise information in result (default: disabled)\n");
    printf("    -x, --xtalk               : enable xtalk information in result (default: disabled)\n");
    printf("    -o, --objects <n>         : number of peaks in result (default: 1)\n");
#ifdef ENABLE_JSON_LOGGING
    printf("    -j, --json                : enable JSON file logging (default: disabled)\n");
#endif
#ifdef ENABLE_KEYSTONE
    printf("    -k, --keystone            : enable keystone angle calculation (default: disabled)\n");
#endif
    printf("    -u, --debug               : enable debug print output (default: disabled)\n");
    printf("\n\n");
}

void catch_signal(int parm)
{
    (void)parm;  /* suppress unused parameter warning */
    g_stop_requested = 1;
}

int main (int argc, char * argv[])
{
    int c;
    int is_measurement = 0;
    int bus_type = 1;  /* 0: I2C, 1: SPI, default SPI */
    int mode = 0;
    int preConfiguration = 0;
    int dualMode = 0;
    int timeout = 5;  /* default 5 seconds */
    int iterations = 1800;  /* default 1800 iterations */
    int short_iterations = 100;  /* default 100 short iterations */
    int threshold = 6;  /* default 6 threshold */
    int period = 33;  /* default 33 period */
    int enable_signal = 0;  /* default 0: disabled */
    int enable_noise = 0;   /* default 0: disabled */
    int enable_xtalk = 0;   /* default 0: disabled */
#ifdef ENABLE_HISTOGRAM
    int enable_histogram = 0;  /* default 0: disabled */
#endif
#ifdef ENABLE_JSON_LOGGING
    int enable_json = 0;      /* default 0: disabled */
#endif
#ifdef ENABLE_KEYSTONE
    int enable_keystone = 0;   /* default 0: disabled */
#endif
    int num_peaks = 1;      /* default 1 */
    time_t start_time;
    tmf8829_cfg_t tof_cfg;
    tmf8829_chip *tof_chip = &g_tof_chip;

    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal);

    /* Define long options */
    static struct option long_options[] = {
        {"measurement",     no_argument,       0, 'm'},
        {"bus",             required_argument, 0, 'b'},
        {"mode",            required_argument, 0, 'd'},
        {"timeout",         required_argument, 0, 't'},
        {"iterations",      required_argument, 0, 'i'},
        {"shortiterations", required_argument, 0, 's'},
        {"threshold",       required_argument, 0, 'r'},
        {"period",          required_argument, 0, 'p'},
#ifdef ENABLE_HISTOGRAM
        {"histogram",       no_argument,       0, 'h'},
#endif
        {"signal",          no_argument,       0, 'g'},
        {"noise",           no_argument,       0, 'n'},
        {"xtalk",           no_argument,       0, 'x'},
        {"objects",         required_argument, 0, 'o'},
#ifdef ENABLE_JSON_LOGGING
        {"json",            no_argument,       0, 'j'},
#endif
#ifdef ENABLE_KEYSTONE
        {"keystone",        no_argument,       0, 'k'},
#endif
        {"debug",           no_argument,       0, 'u'},
        {0, 0, 0, 0}
    };

    while ((c = getopt_long(argc, argv, "mb:d:t:i:s:r:p:hgnxo:jku", long_options, NULL)) != -1) {
        switch (c) {
            case 'm':
                is_measurement = 1;
                break;
            case 'b':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &bus_type);
                if (bus_type != 0 && bus_type != 1) {
                    PRINT_INFO("Error: invalid bus type '%d'. Use 0 for I2C or 1 for SPI.\n", bus_type);
                    return 1;
                }
                PRINT_INFO("bus type: %s\n", bus_type == 0 ? "i2c" : "spi");
                break;
            }
            case 'd':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);

                sscanf(parg, "%d", &mode);
                PRINT_INFO("mode:%d\n", mode);
                break;
            }
            case 't':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &timeout);
                PRINT_INFO("timeout:%d seconds\n", timeout);
                break;
            }
            case 'i':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &iterations);
                PRINT_INFO("iterations:%d\n", iterations);
                break;
            }
            case 's':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &short_iterations);
                PRINT_INFO("short iterations:%d\n", short_iterations);
                break;
            }
            case 'r':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &threshold);
                PRINT_INFO("threshold:%d\n", threshold);
                break;
            }
            case 'p':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &period);
                PRINT_INFO("period:%d\n", period);
                break;
            }
#ifdef ENABLE_HISTOGRAM
            case 'h':
            {
                enable_histogram = 1;
                PRINT_INFO("histogram information enabled\n");
                break;
            }
#endif
            case 'g':
            {
                enable_signal = 1;
                PRINT_INFO("signal information enabled\n");
                break;
            }
            case 'n':
            {
                enable_noise = 1;
                PRINT_INFO("noise information enabled\n");
                break;
            }
            case 'x':
            {
                enable_xtalk = 1;
                PRINT_INFO("xtalk information enabled\n");
                break;
            }
            case 'o':
            {
                char *parg = (optarg == NULL ? "NULL" : optarg);
                sscanf(parg, "%d", &num_peaks);
                PRINT_INFO("number of peaks:%d\n", num_peaks);
                break;
            }
#ifdef ENABLE_JSON_LOGGING
            case 'j':
            {
                enable_json = 1;
                PRINT_INFO("JSON logging enabled\n");
                break;
            }
#endif
#ifdef ENABLE_KEYSTONE
            case 'k':
            {
                enable_keystone = 1;
                PRINT_INFO("Keystone angle calculation enabled\n");
                break;
            }
#endif
            case 'u':
            {
                g_debug_enabled = 1;
                PRINT_INFO("Debug output enabled\n");
                break;
            }
            default:
                PRINT_INFO("Error parsing cmdline args.\n");
                usage(argv[0]);
                return 1;
        }
    }

    /* Display help if no arguments provided */
    if (argc == 1) {
        usage(argv[0]);
        return 0;
    }

    PRINT_DEBUG("driver size:%zu, chip size:%zu\n", sizeof(tmf8829Driver), sizeof(tmf8829_chip));

	tof_chip->gpiod_enable = GPIO_ENABLE_PIN;
    enablePinLow(tof_chip);
    if (enablePinHigh(tof_chip) == -1)
        return 0;

    calculatePreconfigurationAndDualMode(mode, &preConfiguration, &dualMode);

    tof_cfg.conf_threshold = threshold;
    tof_cfg.deadtime = 60;
    tof_cfg.period = period;
    /* Configure resultFormat based on command line options */
    tof_cfg.resultFormat = 0x01;  /* Default: result enabled */
    /* Configure number of peaks (objects) - low 3 bits */
    tof_cfg.resultFormat = (tof_cfg.resultFormat & ~TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK) |
                          (num_peaks & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK);
    if (enable_signal) {
        tof_cfg.resultFormat |= TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK;  /* Enable signal */
    }
    if (enable_noise) {
        tof_cfg.resultFormat |= TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK;  /* Enable noise */
    }
    if (enable_xtalk) {
        tof_cfg.resultFormat |= TMF8829_CFG_RESULT_FORMAT_XTALK_MASK;  /* Enable xtalk */
    }
    tof_cfg.iteration = iterations;
    tof_cfg.shortIteration = short_iterations;
#ifdef ENABLE_HISTOGRAM
    tof_cfg.histogram_dump = enable_histogram;  /* Enable histogram dumping if requested */
#else
    tof_cfg.histogram_dump = 0;  /* Histogram dumping disabled */
#endif
    tof_cfg.dualMode = dualMode;
    tof_cfg.fpMode = preConfiguration; /* Store fp mode for config */

    memset(tof_chip, 0, sizeof(*tof_chip));

#ifdef ENABLE_JSON_LOGGING
    tof_chip->json_enabled = enable_json;
#endif
#ifdef ENABLE_KEYSTONE
    tof_chip->keystoneEnabled = enable_keystone;
#endif

    tmf8829_set_busType(tof_chip, bus_type);
    if (tmf8829_probe(tof_chip) == -1)
        return 0;

    if (tmf8829ConfigMode(tof_chip, preConfiguration) != 0)
        return 0;

    if (tmf8829SettingConfiguration(tof_chip, &tof_cfg) != 0)
        return 0;

#ifdef ENABLE_JSON_LOGGING
    /* Save configuration to JSON logger */
    if (tof_chip->jsonLogger.is_open)
    {
        tmf8829_json_save_config(&tof_chip->jsonLogger, &tof_cfg);
    }
#endif

    if (is_measurement)
    {
        if (tmf8829StartMeasurement(&tof_chip->tof_core) != 0)
            return 0;

        start_time = time(NULL);
        while (!g_stop_requested) {
            tmf8829_app_process_irq(tof_chip);
            //delayInMicroseconds(1000*10);

            /* Check timeout */
            if (timeout > 0 && (time(NULL) - start_time) >= timeout) {
                PRINT_INFO("Timeout reached (%d seconds), stopping...\n", timeout);
                g_stop_requested = 1;
            }
        }
        
        /* Print FPS statistics before exiting */
        tmf8829PrintFpsStats(&tof_chip->frameParser);

        /* Cleanup on exit */
        tmf8829_cleanup(tof_chip);
    }

    return 0;
}
