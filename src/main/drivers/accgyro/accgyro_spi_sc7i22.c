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

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// SC7I22 data is MSB-first (big-endian). Macro to assemble int16_t from
// two consecutive rxBuf bytes: high byte first.
#define SC7I22_INT16_FROM_MSB(buf, idx) ((int16_t)(((uint16_t)(buf)[(idx)] << 8) | (buf)[(idx) + 1]))

#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed
busStatus_e sc7i22Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void sc7i22ExtiHandler(extiCallbackRec_t* cb)
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
#else
void sc7i22ExtiHandler(extiCallbackRec_t* cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

bool sc7i22AccRead(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = SC7I22_REG_ACC_XH | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 8, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
        segments[0].u.buffers.rxData = acc->gyro->dev.rxBuf;

        spiSequence(&acc->gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&acc->gyro->dev);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        // SC7I22 data is MSB-first: rxBuf[1]=XH, rxBuf[2]=XL, rxBuf[3]=YH, rxBuf[4]=YL, rxBuf[5]=ZH, rxBuf[6]=ZL
        uint8_t *rxBuf = acc->gyro->dev.rxBuf;
        acc->ADCRaw[X] = SC7I22_INT16_FROM_MSB(rxBuf, 1);
        acc->ADCRaw[Y] = SC7I22_INT16_FROM_MSB(rxBuf, 3);
        acc->ADCRaw[Z] = SC7I22_INT16_FROM_MSB(rxBuf, 5);
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

bool sc7i22GyroRead(gyroDev_t *gyro)
{
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
                // DMA burst-reads ACC+GYRO together (12 data bytes) from 0x0C
                gyro->dev.txBuf[0] = SC7I22_REG_ACC_XH | 0x80;
                gyro->segments[0].len = 14;
                gyro->segments[0].callback = sc7i22Intcallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = gyro->dev.rxBuf;
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
        // gyro data reading — burst-read GYR_XH..GYR_ZL (6 bytes) from 0x12
        gyro->dev.txBuf[0] = SC7I22_REG_GYR_XH | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = gyro->dev.rxBuf;

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        // SC7I22 data is MSB-first: rxBuf[1]=XH, rxBuf[2]=XL, rxBuf[3]=YH, rxBuf[4]=YL, rxBuf[5]=ZH, rxBuf[6]=ZL
        uint8_t *rxBuf = gyro->dev.rxBuf;
        gyro->gyroADCRaw[X] = SC7I22_INT16_FROM_MSB(rxBuf, 1);
        gyro->gyroADCRaw[Y] = SC7I22_INT16_FROM_MSB(rxBuf, 3);
        gyro->gyroADCRaw[Z] = SC7I22_INT16_FROM_MSB(rxBuf, 5);
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // DMA burst-reads from ACC_XH (0x0C): rxBuf has 14 bytes total
        // rxBuf[0] = echo, rxBuf[1..6] = acc (MSB-first), rxBuf[7..12] = gyro (MSB-first)
        uint8_t *rxBuf = gyro->dev.rxBuf;
        gyro->gyroADCRaw[X] = SC7I22_INT16_FROM_MSB(rxBuf, 7);
        gyro->gyroADCRaw[Y] = SC7I22_INT16_FROM_MSB(rxBuf, 9);
        gyro->gyroADCRaw[Z] = SC7I22_INT16_FROM_MSB(rxBuf, 11);
        break;
    }

    default:
        break;
    }

    return true;
}

#endif // USE_ACCGYRO_SC7I22
