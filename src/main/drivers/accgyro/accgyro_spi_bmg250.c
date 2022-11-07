/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_GYRO_BMG250

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmg250.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 10 MHz max SPI frequency
#define BMG250_MAX_SPI_CLK_HZ 10000000

/* BMG250 Registers */
#define BMG250_REG_CHIPID 0x00
#define BMG250_REG_ERR_REG 0x02
#define BMG250_REG_PMU_STATUS 0x03
#define BMG250_REG_GYR_DATA_X_LSB 0x12
#define BMG250_REG_SENSORTIME 0x18
#define BMG250_REG_STATUS 0x1B
#define BMG250_REG_INT_STATUS_1 0x1D
#define BMG250_REG_TEMPERATURE 0x20
#define BMG250_REG_FIFO_LENGTH 0x22
#define BMG250_REG_FIFO_DATA 0x24
#define BMG250_REG_GYR_CONF 0x42
#define BMG250_REG_GYR_RANGE 0x43
#define BMG250_REG_FIFO_DOWNS 0x45
#define BMG250_REG_FIFO_CONFIG 0x46
#define BMG250_REG_INT_EN 0x51
#define BMG250_REG_INT_OUT_CTRL 0x53
#define BMG250_REG_INT_IN_CTRL 0x54
#define BMG250_REG_INT_MAP 0x56
#define BMG250_REG_CONF 0x6A
#define BMG250_REG_IF_CONF 0x6B
#define BMG250_REG_PMU_TRIGGER 0x6C
#define BMG250_REG_SELF_TEST 0x6D
#define BMG250_REG_NV_CONF 0x70
#define BMG250_REG_OFFSET_3 0x74
#define BMG250_REG_CMD 0x7E

/* BMG250 Register Values */
#define BMG250_PMU_CMD_PMU_GYR_NORMAL (0x01 << 2)
#define BMG250_PMU_CMD_PMU_GYR_FAST_START_UP (0x03 << 2)
#define BMG250_INT_EN1_DRDY 0x10
#define BMG250_INT_OUT_CTRL_INT1_CONFIG 0x0A
#define BMG250_REG_INT_MAP_INT1_DRDY 0x80
#define BMG250_CMD_START_FOC 0x03
#define BMG250_CMD_PROG_NVM 0xA0
#define BMG250_REG_STATUS_NVM_RDY 0x10
#define BMG250_REG_STATUS_FOC_RDY 0x08
#define BMG250_REG_CONF_NVM_PROG_EN 0x02
#define BMG250_VAL_GYRO_CONF_BWP_OSR4 0x00
#define BMG250_VAL_GYRO_CONF_BWP_OSR2 0x10
#define BMG250_VAL_GYRO_CONF_BWP_NORM 0x20

/* BMG250 Parameter Values (not complete)*/
#define BMG250_GYRO_RANGE_2000DPS 0x00
#define BMG250_GYRO_ODR_6400HZ = 0x0E

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Global Variables
static volatile bool BMG250InitDone = false;
static volatile bool BMG250Detected = false;

//! Private functions
static int32_t BMG250_Config(const extDevice_t *dev);
// static int32_t BMG250_do_foc(const extDevice_t *dev); // reserved in datasheet

uint8_t bmg250Detect(const extDevice_t *dev)
{
    if (BMG250Detected) {
        return BMG_250_SPI;
    }

    // Toggle CS to activate SPI (see https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmg250-ds000.pdf)
    spiWrite(dev, 0xFF);

    delay(100); // Give SPI some time to start up

    // Check the chip ID

    if (spiReadRegMsk(dev,BMG250_REG_CHIPID) != 0xD5) {
        return MPU_NONE;
    }

    BMG250Detected = true;
    spiSetClkDivisor(dev, spiCalculateDivider(BMG250_MAX_SPI_CLK_HZ));

    return BMG_250_SPI;
}

/**
 * @brief Initialize the BMG250 gyro
 * @return 0 for success, -1 for failure to allocate,
 *           -10 for failure to get irq 
 */
static void BMG250_Init(const extDevice_t *dev)
{
    if (BMG250InitDone || !BMG250Detected) {
        return;
    }

    /* Configure the BMB250 Sensor */
    if (BMG250_Config(dev) != 0) {
        return;
    }
    // foc is reserved in datasheet
    // bool do_foc = false;
    // /* Perform fast offset compensation if requested */
    // if (do_foc) {
    //     BMG250_do_foc(dev);
    // }



    BMG250InitDone = true;
}

static uint8_t getBmiOsrMode()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMG250_VAL_GYRO_CONF_BWP_OSR4;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return BMG250_VAL_GYRO_CONF_BWP_OSR2;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return BMG250_VAL_GYRO_CONF_BWP_NORM;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return BMG250_VAL_GYRO_CONF_BWP_NORM;
    }
    return 0;
}

/** 
 * @brief Configure the BMG250 sensor
*/
static int32_t BMG250_Config(const extDevice_t *dev)
{
    // Set normal power mode for gyro
    spiWriteReg(dev, BMG250_REG_CMD, BMG250_PMU_CMD_PMU_GYR_FAST_START_UP);
    delay(100); // can take up to 80ms
    // fixme: check the time consumed on real device

    // Verify that normal power mode was entered
    uint8_t pmu_status = spiReadRegMsk(dev, BMG250_REG_PMU_STATUS);
    if ((pmu_status & 0x0C) != 0x0C) {
        return -3;
    }

    uint8_t osrmode = getBmiOsrMode();
    uint8_t gyro_conf = (osrmode<<4) | BMG250_GYRO_ODR_6400HZ;
    // Set odr and range
    spiWriteReg(dev, BMG250_REG_GYR_CONF, gyro_conf);
    delay(1);

    spiWriteReg(dev, BMG250_REG_GYR_RANGE, BMG250_GYRO_RANGE_2000DPS);
    delay(1);

    // Enable offset compensation
    uint8_t val = spiReadRegMsk(dev, BMG250_REG_OFFSET_3);
    spiWriteReg(dev, BMG250_REG_OFFSET_3, val | 0xC0);

    // Enable data ready interrupt
    spiWriteReg(dev, BMG250_REG_INT_EN, BMG250_INT_EN1_DRDY);
    delay(1);

    // Enable INT1 pin
    spiWriteReg(dev, BMG250_REG_INT_OUT_CTRL, BMG250_INT_OUT_CTRL_INT1_CONFIG);
    delay(1);

    // Map data ready interrupt to INT1 pin
    spiWriteReg(dev, BMG250_REG_INT_MAP, BMG250_REG_INT_MAP_INT1_DRDY);
    delay(1);

    return 0;
}

// reserved in datasheet, not used in real device
// static int32_t BMG250_do_foc(const extDevice_t *dev)
// {
//     // assume sensor is mounted on top
//     uint8_t val = 0x7D;
//     // fixme: check this val from datasheet
//     spiWriteReg(dev, BMG250_REG_FOC_CONF, val);

//     // Start FOC
//     spiWriteReg(dev, BMG250_REG_CMD, BMG250_CMD_START_FOC);

//     // Wait for FOC to complete
//     for (int i = 0; i < 100; i++) {
//         delay(10);
//         val = spiReadRegMsk(dev, BMG250_REG_STATUS);
//         if (val & BMG250_REG_STATUS_FOC_RDY) {
//             break;
//         }
//     }
//     if (!(val & BMG250_REG_STATUS_FOC_RDY)) {
//         return -3;
//     }

//     // Program NVM
//     val = spiReadRegMsk(dev, BMG250_REG_CONF);
//     spiWriteReg(dev, BMG250_REG_CONF, val | BMG250_REG_CONF_NVM_PROG_EN);

//     spiWriteReg(dev, BMG250_REG_CMD, BMG250_CMD_PROG_NVM);

//     // Wait for NVM programming to complete
//     for (int i = 0; i < 100; i++) {
//         delay(10);
//         val = spiReadRegMsk(dev, BMG250_REG_STATUS);
//         if (val & BMG250_REG_STATUS_NVM_RDY) {
//             break;
//         }
//     }
//     if (!(val & BMG250_REG_STATUS_NVM_RDY)) {
//         return -6;
//     }

//     return 0;
// }

extiCallbackRec_t bmg250IntCallbackRec;

#ifdef USE_GYRO_EXTI
// CAlled in ISR context
// Gyro read has just completed
busStatus_e bmg250Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmg250ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    // Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
    // not have an associated timer
    uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(&gyro->dev, gyro->segments);
    }

    gyro->detectedEXTI++;

}

static void bmg250IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmg250ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#else
void bmg250ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

static bool bmg250GyroRead(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 14);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[1] = BMG250_REG_GYR_DATA_X_LSB | 0x80;
                gyro->segments[0].len = 14;
                gyro->segments[0].callback = bmg250Intcallback;
                gyro->segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else
#endif
        {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[1] = BMG250_REG_GYR_DATA_X_LSB | 0x80;

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

void bmg250SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    BMG250_Init(dev);
#if defined(USE_GYRO_EXTI)
    bmg250IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMG250_MAX_SPI_CLK_HZ));
}

bool bmg250SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMG_250_SPI) {
        return false;
    }

    gyro->initFn = bmg250SpiGyroInit;
    gyro->readFn = bmg250GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif
