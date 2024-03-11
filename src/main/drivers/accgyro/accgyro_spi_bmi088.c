/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_BMI088 BMI088 Functions
 * @brief Hardware functions to deal with the 6DOF gyro / accel sensor
 * @{
 *
 * @file       pios_BMI088.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @brief      BMI088 Gyro / Accel Sensor Routines
 * @see        The GNU Public License (GPL) Version 3
 ******************************************************************************/

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI088

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi088.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"



/*
  device registers, names follow datasheet conventions, with REGA_
  prefix for accel, and REGG_ prefix for gyro
 */
#define REGA_CHIPID        0x00
#define REGA_ERR_REG       0x02
#define REGA_STATUS        0x03
#define REGA_X_LSB         0x12
#define REGA_INT_STATUS_1  0x1D
#define REGA_TEMP_LSB      0x22
#define REGA_TEMP_MSB      0x23
#define REGA_CONF          0x40
#define REGA_RANGE         0x41
#define REGA_PWR_CONF      0x7C
#define REGA_PWR_CTRL      0x7D
#define REGA_SOFTRESET     0x7E
#define REGA_FIFO_CONFIG0  0x48
#define REGA_FIFO_CONFIG1  0x49
#define REGA_FIFO_DOWNS    0x45
#define REGA_FIFO_DATA     0x26
#define REGA_FIFO_LEN0     0x24
#define REGA_FIFO_LEN1     0x25

#define REGG_CHIPID        0x00
#define REGG_RATE_X_LSB    0x02
#define REGG_INT_CTRL      0x15
#define REGG_INT_STATUS_1  0x0A
#define REGG_INT_STATUS_2  0x0B
#define REGG_INT_STATUS_3  0x0C
#define REGG_FIFO_STATUS   0x0E
#define REGG_RANGE         0x0F
#define REGG_BW            0x10
#define REGG_LPM1          0x11
#define REGG_RATE_HBW      0x13
#define REGG_BGW_SOFTRESET 0x14
#define REGG_FIFO_CONFIG_1 0x3E
#define REGG_FIFO_DATA     0x3F




// 10 MHz max SPI frequency
#define BMI088_MAX_SPI_CLK_HZ 10000000

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Global Variables
static const extDevice_t *BMI088Detected = 0;

static IO_t gyroCsPin,accCsPin;

static void bmi088_switch_to_acc(accDev_t *acc){
    gyroCsPin = acc->gyro->dev.busType_u.spi.csnPin;
    acc->gyro->dev.busType_u.spi.csnPin = accCsPin;
}
static void bmi088_switch_to_gyro(accDev_t *acc){
    acc->gyro->dev.busType_u.spi.csnPin = gyroCsPin;
}


uint8_t bmi088Detect(const extDevice_t *dev)
{
    if (BMI088Detected) {
        if(BMI088Detected->busType_u.spi.csnPin
            == dev->busType_u.spi.csnPin)
            return BMI_088_SPI;
        else{
            return MPU_NONE;
        }
    }
    accCsPin = IOGetByTag(IO_TAG(ACC_1_CS_PIN));

    IOInit(accCsPin, OWNER_GYRO_CS, 6);
    IOConfigGPIO(accCsPin, SPI_IO_CS_CFG);
    IOHi(accCsPin); // Ensure device is disabled

    for (uint8_t  attempts = 0; attempts < 5; attempts++) {
        delay(100);
        uint8_t chipId = spiReadRegMsk(dev, REGG_CHIPID);

        if (chipId == 0x0F) {
            BMI088Detected = dev;
            spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
            return BMI_088_SPI;
        }
    }

    return MPU_NONE;
}

/*
void BMI088ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
*/
static bool BMI088AccRead(accDev_t *acc)
{
    bmi088_switch_to_acc(acc);
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_NO_INT:
    {
        //busReadBuf(acc->busDev, REGA_STATUS, buffer, 2) && (buffer[1] & 0x80)

        acc->gyro->dev.txBuf[1] = REGA_X_LSB | 0x80;
        memset(acc->gyro->dev.rxBuf, 0x00, 8);

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &acc->gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &acc->gyro->dev.rxBuf[1];

        spiSequence(&acc->gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&acc->gyro->dev);

        int16_t *accData = (int16_t *)(acc->gyro->dev.rxBuf+1);
        acc->ADCRaw[X] = accData[1] * 3 / 4;
        acc->ADCRaw[Y] = accData[2] * 3 / 4;
        acc->ADCRaw[Z] = accData[3] * 3 / 4;
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }
    bmi088_switch_to_gyro(acc);
    return true;
}


static bool BMI088GyroRead(gyroDev_t *gyro)
{
    int16_t *gyroData = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 14);
        gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        break;
    }
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[1] = REGG_RATE_X_LSB | 0x80;
        memset(gyro->dev.rxBuf, 0x00, 8);

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
    }

    default:
        break;
    }

    return true;
}


void bmi088SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev=&gyro->dev;
    // Soft reset
    spiWriteReg(dev, REGG_BGW_SOFTRESET, 0xB6);
    delay(100);

    // ODR 2kHz, BW 532Hz
    spiWriteReg(dev, REGG_BW, 0x81);
    delay(1);

    // Enable sampling
    spiWriteReg(dev, REGG_INT_CTRL, 0x80);
    delay(1);

    spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

void bmi088SpiAccInit(accDev_t *acc)
{
    bmi088_switch_to_acc(acc);
    extDevice_t *dev=&acc->gyro->dev;

    //switch to spi
    spiReadRegMsk(dev, REGA_CHIPID);
    delay(1);

    // Soft reset
    spiWriteReg(dev, REGA_SOFTRESET, 0xB6);
    delay(100);

    //switch to spi
    spiReadRegMsk(dev, REGA_CHIPID);
    delay(1);

    // Active mode
    spiWriteReg(dev, REGA_PWR_CONF, 0);
    delay(100);

    // ACC ON
    spiWriteReg(dev, REGA_PWR_CTRL, 0x04);
    delay(100);

    // OSR4, ODR 1600Hz
    spiWriteReg(dev, REGA_CONF, 0x8C);
    delay(1);

    // Range 12g
    spiWriteReg(dev, REGA_RANGE, 0x02);
    delay(1);
    
    bmi088_switch_to_gyro(acc);
    acc->acc_1G = 2048;
}


bool bmi088SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_088_SPI) {
        return false;
    }

    acc->initFn = bmi088SpiAccInit;
    acc->readFn = BMI088AccRead;

    return true;
}


bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_088_SPI) {
        return false;
    }

    gyro->initFn = bmi088SpiGyroInit;
    gyro->readFn = BMI088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif // USE_ACCGYRO_BMI088
