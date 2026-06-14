/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/*
 * History of Versions:
 * 1.0 ... first release version
 * 2.0 ... release for ROM 1v1 Parts
 *     ... update to new core driver
 *     ... some clean up
 *     ... chip_enable_store: start ROM or RAM application(with patch)
 *     ... request_ram_patch_store, execute a Get configuration
 *     ... switch off unused communication interface
 * 2.1 ... clock correction bugfix 125kHz instead of 128kHz
 *     ... remove unused ROM start functionality
 *     ... clock correction
 *     ... probe error handling changed
 * 2.2 ... proximity and motion interrupt handling in core driver added
 * 
 */

/*! \file tmf8829_driver.h - TMF8829 linux driver
 * \brief Device driver for measuring distance in mm.
 */

#ifndef TMF8829_DRIVER_H
#define TMF8829_DRIVER_H

#include "tmf8829.h"
#include "tmf8829_shim.h"
#include "tmf8829_frameparser.h"

#ifdef ENABLE_JSON_LOGGING
#include "tmf8829_json.h"
#endif

#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif

#ifdef ENABLE_HANDPOSE
#include "handpose.h"
#endif

#define BUS_I2C               1
#define BUS_SPI               2

/**
 * @brief TMF8829 chip structure
 * Contains the core driver and frame parser
 * Note: tof_core.dataBuffer is used for raw data from device
 */
typedef struct _tmf8829_chip
{
    tmf8829Driver tof_core;              /**< core driver - MUST be first member */
    tmf8829FrameParser_t frameParser;    /**< frame parser context */
#ifdef ENABLE_JSON_LOGGING
    tmf8829_json_logger_t jsonLogger;    /**< JSON logger context */
    uint8_t json_enabled;                /**< JSON logging enabled flag */
#endif
#ifdef ENABLE_KEYSTONE
    keystoneContext_t keystoneCtx;         /**< Keystone calculation context */
    uint8_t keystoneEnabled;             /**< Keystone enabled flag */
#endif
    int gpiod_enable;                    /**< GPIO enable pin number */
    uint8_t bustype;                     /**< bus type: BUS_I2C or BUS_SPI */
    uint8_t stream_enabled;              /**< emit newline-delimited JSON frames to stdout */
#ifdef ENABLE_HANDPOSE
    handpose_t handpose;                 /**< hand-pose CNN context (model + cfg) */
    uint8_t handpose_enabled;            /**< hand-pose inference enabled flag */
#endif
} tmf8829_chip;

/* Configuration functions */
void calculatePreconfigurationAndDualMode(int mode, int *preConfiguration, int *dualMode);
void tmf8829_set_busType(tmf8829_chip *chip, int isUseI2C);
void tmf8829_reprogramSlaveI2C(tmf8829_chip *chip, int newAddress);
int tmf8829SettingConfiguration(tmf8829_chip *chip, tmf8829_cfg_t *cfg);
int tmf8829ConfigMode(tmf8829_chip *chip, int cmd);

/* IRQ processing */
int tmf8829_app_process_irq(tmf8829_chip *chip);
void tmf8829_dump_registers(tmf8829_chip *chip);

/* Cleanup and close */
void tmf8829_cleanup(tmf8829_chip *chip);

/* Probe and initialization */
int tmf8829_probe(tmf8829_chip *chip);

#endif /* TMF8829_DRIVER_H */
