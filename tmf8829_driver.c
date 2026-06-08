/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/*! \file tmf8829_driver.c
 * \brief Device driver for measuring distance in mm.
 */

/* -------------------------------- includes -------------------------------- */

#include "tmf8829_driver.h"
#include "tmf8829_fw.h"
#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif

/**************************************************************************/
/*  TMF8829 Common Functions                                              */
/**************************************************************************/

/* Dual mode configuration values */
#define TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__disabled       0
#define TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__regular_range  1
#define TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__long_range     2
#define TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__reserved       3

/* Configuration register offsets */
#define TMF8829_CFG_HA_KILO_ITERATIONS_LSB      0x4a
#define TMF8829_CFG_HA_KILO_ITERATIONS_MSB      0x4b
#define TMF8829_CFG_ENABLE_DUAL_MODE            0x4c
#define TMF8829_CFG_ALG_DISTANCE                0x52
#define TMF8829_CFG_ALG_CONFIDENCE_THRESHOLD    0x53

/* FOV correction info register - bits 0-1: X correction, bits 2-3: Y correction */
#define TMF8829_CFG_INFO_FOV_CORR                0x78

void calculatePreconfigurationAndDualMode(int mode, int *preConfiguration, int *dualMode)
{
    *dualMode = TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__disabled;

    switch (mode)
    {
        case 0: /* 8x8 mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8;
            break;
        case 1: /* 8x8 long range */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_LONG_RANGE;
            break;
        case 2: /* 8x8 high accuracy */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_HIGH_ACCURACY;
            break;
        case 3: /* 8x8 dual mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8;
            *dualMode = TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__regular_range;
            break;
        case 4: /* 8x8 long range dual mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_LONG_RANGE;
            *dualMode = TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__long_range;
            break;
        case 5: /* 16x16 */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16;
            break;
        case 6: /* 16x16 high accuracy */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16_HIGH_ACCURACY;
            break;
        case 7: /* 16x16 dual mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16;
            *dualMode = TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__regular_range;
            break;
        case 8: /* 32x32 */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32;
            break;
        case 9: /* 32x32 high accuracy */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32_HIGH_ACCURACY;
            break;
        case 10: /* 32x32 dual mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32;
            *dualMode = TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__regular_range;
            break;
        case 11: /* 48x32 */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32;
            break;
        case 12: /* 48x32 high accuracy */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32_HIGH_ACCURACY;
            break;
        case 13: /* 48x32 dual mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32;
            *dualMode = TMF8829_CFG_ENABLE_DUAL_MODE__dual_mode__regular_range;
            break;
        default: /* 8x8 mode */
            *preConfiguration = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8;
            break;
    }
}

void tmf8829_set_busType(tmf8829_chip *chip, int bus_type)
{
    if (bus_type == 0)
        chip->bustype = BUS_I2C;
    else
        chip->bustype = BUS_SPI;

    PRINT_DEBUG("tmf8829_set_busType: %d\n", chip->bustype);

    if (chip->bustype == BUS_SPI)
    {
        spi_init();
    }
}

void tmf8829_reprogramSlaveI2C(tmf8829_chip *chip, int newAddress)
{
    tmf8829Driver *driver = &chip->tof_core;

    rxReg(driver, driver->i2cSlaveAddress, 0xe0, 1, driver->dataBuffer);
    PRINT_DEBUG("I2C address: 0x%x\n", driver->dataBuffer[0] >> 1);

    if (tmf8829GetConfiguration(driver) != APP_SUCCESS_OK)
        return;

    driver->config[0x90 - TMF8829_CFG_PERIOD_MS_LSB] = newAddress << 1;
    
    tmf8829SetConfiguration(driver);

    driver->dataBuffer[0] = 0;
    driver->i2cSlaveAddress = newAddress;

    rxReg(driver, driver->i2cSlaveAddress, 0xe0, 1, driver->dataBuffer);
    PRINT_DEBUG("New I2C address: 0x%x\n", driver->dataBuffer[0] >> 1);
}

int tmf8829SettingConfiguration(tmf8829_chip *chip, tmf8829_cfg_t *cfg)
{
    int error = 0;
    tmf8829Driver *driver = &chip->tof_core;

    error = tmf8829GetConfiguration(driver);
    if (error != APP_SUCCESS_OK)
        return error;

    driver->config[0] = cfg->period & 0xFF;
    driver->config[TMF8829_CFG_PERIOD_MS_MSB - TMF8829_CFG_PERIOD_MS_LSB] = (cfg->period >> 8) & 0xFF;
    driver->config[TMF8829_CFG_KILO_ITERATIONS_LSB - TMF8829_CFG_PERIOD_MS_LSB] = cfg->iteration & 0xFF;
    driver->config[TMF8829_CFG_KILO_ITERATIONS_MSB - TMF8829_CFG_PERIOD_MS_LSB] = (cfg->iteration >> 8) & 0xFF;
    driver->config[TMF8829_CFG_HA_KILO_ITERATIONS_LSB - TMF8829_CFG_PERIOD_MS_LSB] = cfg->shortIteration & 0xFF;
    driver->config[TMF8829_CFG_HA_KILO_ITERATIONS_MSB - TMF8829_CFG_PERIOD_MS_LSB] = (cfg->shortIteration >> 8) & 0xFF;
    driver->config[TMF8829_CFG_RESULT_FORMAT - TMF8829_CFG_PERIOD_MS_LSB] = cfg->resultFormat;
    driver->config[TMF8829_CFG_DUMP_HISTOGRAMS - TMF8829_CFG_PERIOD_MS_LSB] = cfg->histogram_dump;
    driver->config[TMF8829_CFG_SPAD_DEADTIME - TMF8829_CFG_PERIOD_MS_LSB] = cfg->deadtime;
    driver->config[TMF8829_CFG_ALG_CONFIDENCE_THRESHOLD - TMF8829_CFG_PERIOD_MS_LSB] = cfg->conf_threshold;
    driver->config[TMF8829_CFG_ENABLE_DUAL_MODE - TMF8829_CFG_PERIOD_MS_LSB] = cfg->dualMode;

    /* Update frame parser with dual mode setting */
    chip->frameParser.dualMode = cfg->dualMode;

    /* Read FOV correction from device configuration and store in parser */
    chip->frameParser.fovCorrection = driver->config[TMF8829_CFG_INFO_FOV_CORR - TMF8829_CFG_PERIOD_MS_LSB];
    PRINT_INFO("FOV correction: 0x%02x (X=%d, Y=%d)\n",
               chip->frameParser.fovCorrection,
               chip->frameParser.fovCorrection & 0x03,
               (chip->frameParser.fovCorrection >> 2) & 0x03);

    return tmf8829SetConfiguration(driver);
}

int tmf8829ConfigMode(tmf8829_chip *chip, int cmd)
{
    if ((cmd < TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8) || 
        (cmd > TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32_HIGH_ACCURACY))
    {
        return -1;
    }

    return tmf8829Command(&chip->tof_core, cmd);
}

void tmf8829_dump_registers(tmf8829_chip *chip)
{
    tmf8829Driver *driver = &chip->tof_core;
    uint8_t reg_buffer[256];

    int8_t stat = tmf8829CmdLoadConfigPage(driver);

    if (stat != APP_SUCCESS_OK)
    {
        PRINT_INFO("tmf8829CmdLoadConfigPage failed\n");
        return;
    }

    rxReg(driver, driver->i2cSlaveAddress, 0x00, 256, reg_buffer);
    for (int i = 0; i < 256; i += 16)
    {
        PRINT_INFO("0x%02X: ", i);
        for (int j = 0; j < 16 && (i + j) < 256; j++)
        {
            PRINT_INFO("%02X ", reg_buffer[i + j]);
        }
        PRINT_INFO("\n");
    }
}

int tmf8829_app_process_irq(tmf8829_chip *chip)
{
    uint8_t intStatus = 0;
    int8_t res = APP_SUCCESS_OK;
    tmf8829Driver *driver = &chip->tof_core;

    intStatus = tmf8829GetAndClrInterrupts(driver, TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_HISTOGRAMS);

    if (intStatus & TMF8829_APP_INT_RESULTS)
    {
        res = tmf8829ReadResults(driver);
        if (res == APP_SUCCESS_OK)
        {
            //PRINT_DEBUG("read result successful\n");
            //tmf8829_dump_registers(chip);
        }
    }

    if (intStatus & TMF8829_APP_INT_HISTOGRAMS)
    {
        res = tmf8829ReadHistogram(driver);
        if (res == APP_SUCCESS_OK)
        {
            //PRINT_DEBUG("read histogram successful\n");
        }
    }

    return 0;
}

int tmf8829_probe(tmf8829_chip *chip)
{
    int error = -1;
    tmf8829Driver *driver = &chip->tof_core;
    uint8_t fw_version[4];

    /* Initialize core driver */
    tmf8829Initialise(driver);

    /* Initialize frame parser */
    tmf8829FrameParserInit(&chip->frameParser);

#ifdef ENABLE_KEYSTONE
    /* Set keystone enabled flag in parser and initialize context if enabled */
    chip->frameParser.keystoneEnabled = chip->keystoneEnabled;
    if (chip->keystoneEnabled) {
        keystoneInit(&chip->keystoneCtx);
    }
#endif

    /* The chip auto-boots into its bootloader when EN goes high; the
     * PON/cpu_ready handshake (tmf8829PowerUp + tmf8829IsCpuReady) is not
     * needed and the cpu_ready bit in the ENABLE register never sets on this
     * hardware revision.  A 20 ms settle delay is sufficient (mirrors the
     * approach used in tmf8829_rpi3_spi_app/src/main.c). */
    delayInMicroseconds( 20 * 1000 );

    /* Switch off unused communication interface */
    if (chip->bustype == BUS_SPI)
        error = tmf8829BootloaderCmdI2cOff(driver);
    else
        error = tmf8829BootloaderCmdSpiOff(driver);

    if (error != BL_SUCCESS_OK)
    {
        PRINT_INFO("Failed to switch off interface\n");
        return -1;
    }

    /* Download firmware */
    error = tmf8829DownloadFirmware(driver, TMF8829_IMAGE_START, patchImage, sizeof(patchImage), 0);
    if (error != BL_SUCCESS_OK)
    {
        PRINT_INFO("Download Error %d\n", error);
        return -1;
    }

    tmf8829ClrAndEnableInterrupts(driver, TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_HISTOGRAMS);

    if (tmf8829GetConfiguration(driver) != APP_SUCCESS_OK)
    {
        PRINT_INFO("Read device configuration error.\n");
        return -1;
    }

    //PRINT_INFO("read device information......\n");
    if (tmf8829ReadDeviceInfo(driver) != APP_SUCCESS_OK)
    {
        PRINT_INFO("Read device information error.\n");
        return -1;
    }

#ifdef ENABLE_JSON_LOGGING
    /* Initialize JSON logger if enabled */
    if (chip->json_enabled)
    {
        fw_version[0] = driver->device.appVersion[0];
        fw_version[1] = driver->device.appVersion[1];
        fw_version[2] = driver->device.appVersion[2];
        fw_version[3] = driver->device.appVersion[3];

        if (tmf8829_json_init(&chip->jsonLogger, "tmf8829", driver->device.deviceSerialNumber, fw_version) != 0)
        {
            PRINT_INFO("Failed to initialize JSON logger\n");
            chip->json_enabled = 0;
        }
    }
#endif

    return 0;
}

void tmf8829_cleanup(tmf8829_chip *chip)
{
    if (chip == NULL)
    {
        return;
    }

    /* Cleanup frame parser resources */
    tmf8829FrameParserCleanup(&chip->frameParser);
    
#ifdef ENABLE_JSON_LOGGING
    /* Close JSON logger */
    if (chip->jsonLogger.is_open)
    {
        tmf8829_json_flush(&chip->jsonLogger);
        tmf8829_json_close(&chip->jsonLogger);
    }
#endif
    PRINT_DEBUG("Cleanup done\n");
}
