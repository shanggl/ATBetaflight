/*
 * This file is part of Cleanflight and ATBetaflight (forked by flightng).
 *
 * Cleanflight and ATBetaflight (forked by flightng) are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and ATBetaflight (forked by flightng) are distributed in the hope that they
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

#ifdef USE_ACCGYRO_SC7I22

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_sc7i22.h"
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
#define SC7I22_MAX_SPI_CLK_HZ 10000000

#define SC7I22_CHIP_ID 0x6A

// SC7I22 register configuration values
typedef enum {
    SC7I22_VAL_SOFT_RST_TRIGGER = 0xA5,       // write once to SOFT_RST

    // COM_CFG explicit default: BDU=1, Addr_Auto=1, 4-wire SPI.
    SC7I22_VAL_COM_CFG_DEFAULT = 0x50,

    // PWR_CTRL: TEMP_EN | ACC_EN | GYR_EN
    SC7I22_VAL_PWR_CTRL_ALL_ON = 0x0E,

    // ACC_RANGE FS[1:0] = 0b11 → ±16g
    SC7I22_VAL_ACC_RANGE_16G = 0x03,

    // GYR_RANGE FS[2:0] = 0b000 → ±2000dps
    SC7I22_VAL_GYR_RANGE_2000DPS = 0x00,

    // ACC_CONF: high-performance | OSR4_AVG1 | 800 Hz.
    // bit7=1 ACC_FILTER_PERF, bits6:4=000 OSR4_AVG1, bits3:0=1011 (800 Hz).
    // OSR4 ≈ 163 Hz -3dB @ 800 Hz ODR (Bosch BMI270 table).
    SC7I22_VAL_ACC_CONF_HIGHPERF_800HZ = 0x8B,

    // GYR_CONF: high-performance | noise-perf on | NORM_AVG4 | 3200 Hz.
    // bit7=1 GYR_FILTER_PERF, bit6=1 GYR_NOISE_PERF, bits5:4=10 NORM_AVG4,
    // bits3:0=1101 (3200 Hz). NORM_AVG4 ≈ 751 Hz -3dB @ 3200 Hz ODR.
    // NOISE_PERF=1 requires acc+gyr disabled at write time.
    SC7I22_VAL_GYR_CONF_HIGHPERF_3200HZ = 0xED,
} sc7i22ConfigValues_e;

// SC7I22 uses BMI270-style register access: one dummy byte before the register value.
static uint8_t sc7i22RegisterRead(const extDevice_t *dev, sc7i22Register_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void sc7i22RegisterWrite(const extDevice_t *dev, sc7i22Register_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

uint8_t sc7i22Detect(const extDevice_t *dev)
{
    uint8_t chipID = 0;
    uint8_t i = 0;

    while ((chipID != SC7I22_CHIP_ID) && (i++ < 5)) {
        chipID = sc7i22RegisterRead(dev, SC7I22_REG_WHO_AM_I);
        if ((i == 5) && (chipID != SC7I22_CHIP_ID)) {
            return MPU_NONE;
        }
    }
    return SC7I22_SPI;
}

static void sc7i22Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Soft reset: write 0xA5 once to SOFT_RST (unlike SC7U22 which writes twice).
    sc7i22RegisterWrite(dev, SC7I22_REG_SOFT_RST, SC7I22_VAL_SOFT_RST_TRIGGER, 50);

    // SC7I22 ordering: PWR_CTRL first (enable temp+acc+gyro), then configure.
    // This is the opposite of SC7U22 where PWR_CTRL is written last.
    sc7i22RegisterWrite(dev, SC7I22_REG_PWR_CTRL, SC7I22_VAL_PWR_CTRL_ALL_ON, 10);

    // Force COM_CFG to documented default (BDU=1, Addr_Auto=1, 4-wire SPI).
    sc7i22RegisterWrite(dev, SC7I22_REG_COM_CFG, SC7I22_VAL_COM_CFG_DEFAULT, 1);

    // Ranges: ±16g accel, ±2000dps gyro
    sc7i22RegisterWrite(dev, SC7I22_REG_ACC_RANGE, SC7I22_VAL_ACC_RANGE_16G, 1);
    sc7i22RegisterWrite(dev, SC7I22_REG_GYR_RANGE, SC7I22_VAL_GYR_RANGE_2000DPS, 1);

    // ACC_CONF: high-performance, OSR4_AVG1, 800 Hz ODR
    sc7i22RegisterWrite(dev, SC7I22_REG_ACC_CONF, SC7I22_VAL_ACC_CONF_HIGHPERF_800HZ, 1);

    // GYR_CONF requires acc+gyr disabled when changing GYR_NOISE_PERF.
    // Default has NOISE_PERF=0, we want NOISE_PERF=1, so briefly disable PWR_CTRL.
    sc7i22RegisterWrite(dev, SC7I22_REG_PWR_CTRL, 0x00, 1);
    sc7i22RegisterWrite(dev, SC7I22_REG_GYR_CONF, SC7I22_VAL_GYR_CONF_HIGHPERF_3200HZ, 1);
    sc7i22RegisterWrite(dev, SC7I22_REG_PWR_CTRL, SC7I22_VAL_PWR_CTRL_ALL_ON, 10);
}

#ifdef USE_GYRO_EXTI
static void sc7i22IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, sc7i22ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void sc7i22SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    sc7i22Config(gyro);

#ifdef USE_GYRO_EXTI
    sc7i22IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(SC7I22_MAX_SPI_CLK_HZ));
}

static void sc7i22SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool sc7i22SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != SC7I22_SPI) {
        return false;
    }

    acc->initFn = sc7i22SpiAccInit;
    acc->readFn = sc7i22AccRead;

    return true;
}

bool sc7i22SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != SC7I22_SPI) {
        return false;
    }

    gyro->initFn = sc7i22SpiGyroInit;
    gyro->readFn = sc7i22GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // USE_ACCGYRO_SC7I22
