/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/** @file This is the shim for raspberry pi
 * Defines, macro and functions to match the target platform.
 */

#ifndef TMF8829_SHIM_EVM_H
#define TMF8829_SHIM_EVM_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <linux/types.h>

#include "tmf8829_frameparser.h"

#define DATA_BUFFER_SIZE               30000

/** @brief macros to cast a pointer to an address - adapt for your machine-word size
 */
#define PTR_TO_UINT(ptr)               ( (uintptr_t)(ptr) )

/** @brief macros to replace the platform specific printing
 */
#define PRINT_CHAR(c)                  fprintf( stderr, "%c", c )
#define PRINT_INT(i)                   fprintf( stderr, "%ld", (long)i )
#define PRINT_UINT(i)                  fprintf( stderr, "%lu", (unsigned long)i )
#define PRINT_UINT_HEX(i)              fprintf( stderr, "%lX", (unsigned long)i )
#define PRINT_STR(str)                 fprintf( stderr, "%s", str )
#define PRINT_LN()                     fprintf( stderr, "\n" )

extern int g_debug_enabled;

#define PRINT_DEBUG(fmt, ...)          do { \
                                            if (g_debug_enabled) { \
                                                fprintf( stderr, fmt, ##__VA_ARGS__); \
                                            } \
                                        } while (0)

#define PRINT_INFO(fmt, ...)           fprintf( stderr, fmt, ##__VA_ARGS__)

/** Which character to use to separate the entries in printing */
#define SEPARATOR                      ','

// for clock correction insert here the number in relation to your host
#define HOST_TICKS_PER_1000_US         1000     /**< number of host ticks every 1000 microseconds */  
#define TMF8829_TICKS_PER_1000_US      125      /**< number of tmf8829 ticks every 1000 microseconds  125kHz */ 


/**  Return codes for i2c functions: 
 */
#define I2C_SUCCESS             0       /**< successful execution no error */
#define I2C_ERROR              -1       /**< i2c error */

int spi_init(void);


void delayInMicroseconds(uint32_t wait);
uint32_t getSysTick(void);
uint8_t readProgramMemoryByte(uintptr_t address);
int8_t txReg(void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, uint8_t *txData);
int8_t rxReg(void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t *rxData);
int enablePinHigh(void *dptr);
int enablePinLow(void *dptr);

#endif /* TMF8829_SHIM_EVM_H */
