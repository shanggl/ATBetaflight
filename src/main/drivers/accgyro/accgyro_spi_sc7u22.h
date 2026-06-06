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

#pragma once

#include "drivers/bus.h"
#include "drivers/exti.h"

// SC7U22 registers (Bosch-family 6-axis IMU, SPI only)
// WHO_AM_I = 0x6A at reg 0x01. Data is MSB-first (big-endian).
// CPOL=1, CPHA=1 (SPI mode 3), max 10 MHz SPC clock.
//
// Deltas vs SC7I22:
//   1. COM_CONF lives at 0x04 (SC7I22 has COM_CFG at 0x05).
//   2. Soft reset requires writing 0xA5 twice (SC7I22 writes once).
//   3. Configuration order is ranges → ODR → PWR_CTRL last (SC7I22 writes PWR_CTRL first).
typedef enum {
    SC7U22_REG_WHO_AM_I  = 0x01,
    SC7U22_REG_COM_CONF  = 0x04, // default 0x50 — BDU=1, Addr_Auto=1, push-pull, 4-wire SPI
    SC7U22_REG_ACC_XH    = 0x0C, // ACC X high byte (MSB-first)
    SC7U22_REG_GYR_XH    = 0x12, // GYR X high byte (MSB-first)
    SC7U22_REG_TEMP_H    = 0x22, // Temperature MSB
    SC7U22_REG_ACC_CONF  = 0x40,
    SC7U22_REG_ACC_RANGE = 0x41,
    SC7U22_REG_GYR_CONF  = 0x42,
    SC7U22_REG_GYR_RANGE = 0x43,
    SC7U22_REG_SOFT_RST  = 0x4A,
    SC7U22_REG_PWR_CTRL  = 0x7D,
} sc7u22Register_e;

// Contained in accgyro_spi_sc7u22_init.c which is size optimized
uint8_t sc7u22Detect(const extDevice_t *dev);
bool sc7u22SpiAccDetect(accDev_t *acc);
bool sc7u22SpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_sc7u22.c which is speed optimized
void sc7u22ExtiHandler(extiCallbackRec_t *cb);
bool sc7u22AccRead(accDev_t *acc);
bool sc7u22GyroRead(gyroDev_t *gyro);
