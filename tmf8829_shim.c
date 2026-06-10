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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <gpiod.h>

#include "tmf8829_driver.h"
#include "tmf8829_shim.h"

/** @brief Debug print macros for two-level print control
 *  PRINT_DEBUG: Controlled by --debug flag, prints debug and tracking information
 *  PRINT_INFO:  Always prints, prints errors and important messages
 */
int g_debug_enabled = 0;

/////////////////////////////////////////////////
// GPIO via libgpiod v2

#define GPIO_CHIP_PATH "/dev/gpiochip0"

static struct gpiod_chip *g_gpio_chip = NULL;
static struct gpiod_line_request *g_gpio_enable_req = NULL;
static unsigned int g_gpio_enable_offset = 0;

static struct gpiod_line_request *
gpio_request_output( struct gpiod_chip *chip, unsigned int offset,
                     enum gpiod_line_value init_value, const char *consumer )
{
    struct gpiod_line_request *req = NULL;
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();

    if ( !settings || !line_cfg || !req_cfg )
    {
        fprintf( stderr, "gpiod: failed to allocate config objects\n" );
        goto out;
    }

    gpiod_line_settings_set_direction( settings, GPIOD_LINE_DIRECTION_OUTPUT );
    gpiod_line_settings_set_output_value( settings, init_value );

    if ( gpiod_line_config_add_line_settings( line_cfg, &offset, 1, settings ) < 0 )
    {
        fprintf( stderr, "gpiod_line_config_add_line_settings(offset=%u): %s\n", offset, strerror( errno ) );
        goto out;
    }

    gpiod_request_config_set_consumer( req_cfg, consumer );

    req = gpiod_chip_request_lines( chip, req_cfg, line_cfg );
    if ( !req )
        fprintf( stderr, "gpiod_chip_request_lines(offset=%u): %s\n", offset, strerror( errno ) );

out:
    gpiod_request_config_free( req_cfg );
    gpiod_line_config_free( line_cfg );
    gpiod_line_settings_free( settings );
    return req;
}

/////////////////////////////////////////////////

/////////////////////////////////////////////////
// spi implemented function

#define SPI_CHANNEL                0
#define SPI_MAX_SPEED              10000000
#define SPI_WR_CMD                 0x02
#define SPI_RD_CMD                 0x03

static char data_buff[1024];
static char *spi_devname = "/dev/spidev0.0";
static int fd_spi = -1;

void print_data(unsigned char *data, int len, char *msg)
{
    int i;
    int n = 0;

    return;
    if (len > 200)
    {
        PRINT_DEBUG("data len is too long, only print 100 bytes\n");
        len = 200;
    }
    //PRINT_DEBUG("%s"\n", msg);

    memset(data_buff, 0, sizeof(data_buff));
    n += sprintf(data_buff+n, "%s:", msg);
    for (i = 0; i <= len; i++)
    {
        if (i >= len)
        {
            PRINT_DEBUG("%s\n", data_buff);
            break;
        }
        if (i != 0 && i%8 == 0)
        {
            PRINT_DEBUG("%s\n", data_buff);
            memset(data_buff, 0, sizeof(data_buff));
            n = 0;
        }
        n += sprintf(data_buff+n, "0x%x ", data[i]);
    }
}

int spi_data_rw(unsigned char *writeData, int writeLen, unsigned char *readData, int readLen)
{
  struct spi_ioc_transfer spi[2] ;

    memset (spi, 0, sizeof (spi));

    if (writeLen)
    {
        spi[0].tx_buf        = (unsigned long)writeData ;
        //spi[0].rx_buf        = (unsigned long)data ;
        spi[0].len           = writeLen ;
        spi[0].delay_usecs   = 0 ;
        spi[0].speed_hz      = SPI_MAX_SPEED;
        spi[0].bits_per_word = 8 ;
    }

    if (readLen)
    {
        //spi[0].tx_buf        = (unsigned long)writeData ;
        spi[1].rx_buf        = (unsigned long)readData ;
        spi[1].len           = readLen;
        spi[1].delay_usecs   = 0 ;
        spi[1].speed_hz      = SPI_MAX_SPEED;
        spi[1].bits_per_word = 8 ;
    }

    int transfer_count = (writeLen > 0 ? 1 : 0) + (readLen > 0 ? 1 : 0);
    if (transfer_count == 0) {
        PRINT_INFO("SPI data transfer: no data to transfer\n");
        return -1;
    }

    return ioctl(fd_spi, SPI_IOC_MESSAGE(transfer_count), spi) ;
}

int spi_init(void)
{
    char spi_bits;
    int spi_speed;
    int mode;
    int ret;

    fd_spi = open(spi_devname, O_RDWR);
    if (fd_spi < 0)
    {
        PRINT_INFO("Failed to open SPI device %s, fd_spi = %d\n", spi_devname, fd_spi);
        return -1;
    }

    mode = 0;
    ret = ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
    if (ret < 0)
    {
        PRINT_INFO("Failed to set SPI mode\n");
        close(fd_spi);
        return ret;
    }

    spi_bits = 8;
    ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    if (ret < 0)
    {
        PRINT_INFO("Failed to set SPI bits per word\n");
        close(fd_spi);
        return ret;
    }

    spi_speed = SPI_MAX_SPEED;
    ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (ret < 0)
    {
        PRINT_INFO("Failed to set SPI max speed\n");
        close(fd_spi);
        return ret;
    }
    PRINT_DEBUG("SPI mode: 0x%x\n", mode);
    PRINT_DEBUG("SPI bits per word: %d\n", spi_bits);
    PRINT_DEBUG("SPI max speed: %d Hz (%d KHz)\n", spi_speed, spi_speed/1000);
    PRINT_INFO("Successfully initialized SPI %s, fd_spi = %d\n", spi_devname, fd_spi);

    return fd_spi;
}

int spi_uninit(void)
{
    int ret;

    if (fd_spi < 0)
        return 0;

    PRINT_DEBUG("SPI uninitializing, fd_spi: %d\n", fd_spi);
    ret = close(fd_spi);
    if (0 != ret)
    {
        PRINT_INFO("Failed to close SPI, ret = %d\n", ret);
    }
    else
    {
        fd_spi = -1;
        PRINT_INFO("Successfully closed SPI, ret = %d\n", ret);
    }

    return 0;
}

int spi_write(uint8_t regAddr, const char *data, int len)
{
    int ret;
	int i;
	unsigned char writebuf[1024];

    writebuf[0] = SPI_WR_CMD;
    writebuf[1] = regAddr;

    for(i = 0 ;i < len;i++)
    {
        writebuf[i+2] = (unsigned char)data[i];
    }

    print_data(writebuf, len + 2, "spi write");

    ret = spi_data_rw(writebuf, len + 2, 0, 0);
    if (ret < 0)
    {
        PRINT_INFO("SPI write failed\n");
        return ret;
    }

    return 0;
}

int spi_read(uint8_t regAddr, unsigned char *rxData, int len)
{
    int ret = 0;
    unsigned char wrbuf[3];
    int offset = 0;
    int chunkSize;
    int maxChunkSize = 4096;  /* Linux SPI driver typically has 4KB limit */

    wrbuf[0] = SPI_RD_CMD;
    wrbuf[1] = regAddr;
    wrbuf[2] = 0;          // dummy

    print_data(wrbuf, 2, "spi read cmd");

    /* Read data in chunks for large transfers */
    while (offset < len)
    {
        chunkSize = len - offset;
        if (chunkSize > maxChunkSize)
        {
            chunkSize = maxChunkSize;
        }

        ret = spi_data_rw(wrbuf, 3, rxData + offset, chunkSize);
        if (ret < 0)
        {
            PRINT_INFO("SPI read failed (offset %d, chunk %d)\n", offset, chunkSize);
            return ret;
        }

        offset += chunkSize;
    }

    print_data(rxData, len, "spi read data");

    return 0;
}
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// i2c implemented function
static char *i2cdev_fp = "/dev/i2c-0";

int write_i2c_block(uint32_t slave_addr, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    int32_t i2c_fd;
    uint8_t *outbuf = NULL;
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages[1];

    if (buf == NULL || len == 0) {
        return -1;
    }

    outbuf = (uint8_t *)malloc(len + 1);
    if (!outbuf) {
        return -1;
    }

    i2c_fd = open(i2cdev_fp, O_RDWR);
    if (i2c_fd < 0) {
        return -1;
    }

    data.msgs = messages;
    data.nmsgs = 1;

    messages[0].addr  = slave_addr;
    messages[0].flags = 0;
    messages[0].buf   = outbuf;
    messages[0].len   = len + 1;

    outbuf[0] = reg;
    memcpy(outbuf + 1, buf, len);

    if (ioctl(i2c_fd, I2C_RDWR, &data) < 0) {
        close(i2c_fd);
        free(outbuf);
        PRINT_INFO("I2C write failed\n");
        return -1;
    }


    close(i2c_fd);
    free(outbuf);

    return 0;
}

int read_i2c_block(uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint32_t len)
{
    int32_t i2c_fd;
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages[2];

    if (buf == NULL || len == 0) {
        return -1;
    }

    i2c_fd = open(i2cdev_fp, O_RDWR);
    if (i2c_fd < 0) {
        return -1;
    }

    //PRINT_DEBUG("read_i2c_block reg:0x%x, len:%d"\n", reg, len);
    data.msgs = messages;
    data.nmsgs = 2;

    messages[0].addr  = slave_addr;
    messages[0].flags = 0;
    messages[0].buf   = &reg;
    messages[0].len   = 1;

    messages[1].addr  = slave_addr;
    messages[1].flags = I2C_M_RD;
    messages[1].buf   = buf;
    messages[1].len   = len;

    if (ioctl(i2c_fd, I2C_RDWR, &data) < 0) {
        close(i2c_fd);
        PRINT_INFO("I2C read failed\n");
        return -1;
    }

    close(i2c_fd);

    return 0;
}
////////////////////////////////////////////////////////////////

void delayInMicroseconds(uint32_t wait)
{
    usleep(wait);
}

uint32_t getSysTick(void) //Note: is only for 70 minutes
{
    struct timespec ts;

    clock_gettime(CLOCK_REALTIME, &ts);

    return (uint32_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

uint8_t readProgramMemoryByte(uintptr_t address)
{
    uintptr_t *ptr = (void *) address;
    uint8_t byte = *ptr;
    return byte;
}

int8_t txReg(void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, uint8_t *txData)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
        return (int8_t)write_i2c_block(slaveAddr, regAddr, txData, toTx);
    return spi_write(regAddr, (const char *)txData, toTx);
}

int8_t rxReg(void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t *rxData)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
        return (int8_t)read_i2c_block(slaveAddr, regAddr, rxData, toRx);
    return spi_read(regAddr, rxData, toRx);
}

static int gpio_drive_enable( void *dptr, enum gpiod_line_value value )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    unsigned int offset = (unsigned int)driver->gpiod_enable;
    /* After memset(tof_chip,0) gpiod_enable is 0; fall back to the saved offset */
    if ( offset == 0 )
        offset = g_gpio_enable_offset;
    if ( offset == 0 )
        return ( value == GPIOD_LINE_VALUE_INACTIVE ) ? 0 : -1;

    if ( !g_gpio_chip )
    {
        g_gpio_chip = gpiod_chip_open( GPIO_CHIP_PATH );
        if ( !g_gpio_chip )
        {
            fprintf( stderr, "gpiod_chip_open(%s): %s\n", GPIO_CHIP_PATH, strerror( errno ) );
            return -1;
        }
    }

    if ( !g_gpio_enable_req || g_gpio_enable_offset != offset )
    {
        if ( g_gpio_enable_req )
        {
            gpiod_line_request_release( g_gpio_enable_req );
            g_gpio_enable_req = NULL;
        }
        g_gpio_enable_offset = offset;
        g_gpio_enable_req = gpio_request_output( g_gpio_chip, offset, value, "tmf8829-en" );
        if ( !g_gpio_enable_req )
            return -1;
        return 0;
    }

    return gpiod_line_request_set_value( g_gpio_enable_req, offset, value );
}

int enablePinHigh(void *dptr)
{
    return gpio_drive_enable( dptr, GPIOD_LINE_VALUE_ACTIVE );
}

int enablePinLow(void *dptr)
{
    return gpio_drive_enable( dptr, GPIOD_LINE_VALUE_INACTIVE );
}
