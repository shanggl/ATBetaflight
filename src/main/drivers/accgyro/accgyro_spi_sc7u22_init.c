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

#ifdef USE_ACCGYRO_SC7U22

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_sc7u22.h"
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
#define SC7U22_MAX_SPI_CLK_HZ 10000000

#define SC7U22_CHIP_ID 0x6A

// SC7U22 register configuration values
typedef enum {
    SC7U22_VAL_SOFT_RST_TRIGGER = 0xA5,       // write twice to SOFT_RST (unlike SC7I22)

    // COM_CONF explicit default: BDU=1, Addr_Auto=1, push-pull, active-high, 4-wire SPI
    SC7U22_VAL_COM_CONF_DEFAULT = 0x50,

    // PWR_CTRL: TEMP_EN | ACC_EN | GYR_EN
    SC7U22_VAL_PWR_CTRL_ALL_ON = 0x0E,

    // ACC_RANGE FS[1:0] = 0b11 → ±16g
    SC7U22_VAL_ACC_RANGE_16G = 0x03,

    // GYR_RANGE FS[2:0] = 0b000 → ±2000dps
    SC7U22_VAL_GYR_RANGE_2000DPS = 0x00,

    // ACC_CONF: high-performance | OSR4_AVG1 | 800 Hz.
    // bit7=1 ACC_FILTER_PERF, bits6:4=000 OSR4_AVG1, bits3:0=1011 (800 Hz).
    // OSR4 ≈ 163 Hz -3dB @ 800 Hz ODR (Bosch BMI270 table).
    SC7U22_VAL_ACC_CONF_HIGHPERF_800HZ = 0x8B,

    // GYR_CONF: high-performance | noise-perf on | NORM_AVG4 | 3200 Hz.
    // bit7=1 GYR_FILTER_PERF, bit6=1 GYR_NOISE_PERF, bits5:4=10 NORM_AVG4,
    // bits3:0=1101 (3200 Hz). NORM_AVG4 ≈ 751 Hz -3dB @ 3200 Hz ODR.
    // NOISE_PERF=1 needs acc+gyr disabled — naturally satisfied since
    // SC7U22 writes PWR_CTRL last (after GYR_CONF).
    SC7U22_VAL_GYR_CONF_HIGHPERF_3200HZ = 0xED,
} sc7u22ConfigValues_e;

// SC7U22 uses BMI270-style register access: one dummy byte before the register value.
static uint8_t sc7u22RegisterRead(const extDevice_t *dev, sc7u22Register_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void sc7u22RegisterWrite(const extDevice_t *dev, sc7u22Register_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

uint8_t sc7u22Detect(const extDevice_t *dev)
{
    uint8_t chipID = 0;
    uint8_t i = 0;

    while ((chipID != SC7U22_CHIP_ID) && (i++ < 5)) {
        chipID = sc7u22RegisterRead(dev, SC7U22_REG_WHO_AM_I);
        if ((i == 5) && (chipID != SC7U22_CHIP_ID)) {
            return MPU_NONE;
        }
    }
    return SC7U22_SPI;
}

static void sc7u22Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Soft reset: write 0xA5 twice to SOFT_RST (unlike SC7I22 which writes once).
    sc7u22RegisterWrite(dev, SC7U22_REG_SOFT_RST, SC7U22_VAL_SOFT_RST_TRIGGER, 1);
    sc7u22RegisterWrite(dev, SC7U22_REG_SOFT_RST, SC7U22_VAL_SOFT_RST_TRIGGER, 60);

    // SC7U22 ordering: configure COM_CONF / ranges / ODR first, then PWR_CTRL last.
    // This is the mirror of SC7I22 which writes PWR_CTRL first.

    // Force COM_CONF to documented default (BDU=1, Addr_Auto=1, push-pull, 4-wire SPI)
    sc7u22RegisterWrite(dev, SC7U22_REG_COM_CONF, SC7U22_VAL_COM_CONF_DEFAULT, 1);

    // Ranges: ±16g accel, ±2000dps gyro
    sc7u22RegisterWrite(dev, SC7U22_REG_ACC_RANGE, SC7U22_VAL_ACC_RANGE_16G, 1);
    sc7u22RegisterWrite(dev, SC7U22_REG_GYR_RANGE, SC7U22_VAL_GYR_RANGE_2000DPS, 1);

    // ACC_CONF: high-performance, OSR4_AVG1, 800 Hz ODR
    sc7u22RegisterWrite(dev, SC7U22_REG_ACC_CONF, SC7U22_VAL_ACC_CONF_HIGHPERF_800HZ, 1);

    // GYR_CONF: high-performance, noise-perf on, NORM_AVG4, 3200 Hz ODR.
    // NOISE_PERF=1 is naturally satisfied here because PWR_CTRL has not been
    // written yet (sensors are still disabled from reset). No toggle needed.
    sc7u22RegisterWrite(dev, SC7U22_REG_GYR_CONF, SC7U22_VAL_GYR_CONF_HIGHPERF_3200HZ, 1);

    // Enable temp + accel + gyro last.
    sc7u22RegisterWrite(dev, SC7U22_REG_PWR_CTRL, SC7U22_VAL_PWR_CTRL_ALL_ON, 10);
}

#ifdef USE_GYRO_EXTI
static void sc7u22IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, sc7u22ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void sc7u22SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    sc7u22Config(gyro);

#ifdef USE_GYRO_EXTI
    sc7u22IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(SC7U22_MAX_SPI_CLK_HZ));
}

static void sc7u22SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool sc7u22SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != SC7U22_SPI) {
        return false;
    }

    acc->initFn = sc7u22SpiAccInit;
    acc->readFn = sc7u22AccRead;

    return true;
}

bool sc7u22SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != SC7U22_SPI) {
        return false;
    }

    gyro->initFn = sc7u22SpiGyroInit;
    gyro->readFn = sc7u22GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // USE_ACCGYRO_SC7U22
