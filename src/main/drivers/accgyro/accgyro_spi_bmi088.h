#pragma once

#include "drivers/bus.h"


uint8_t bmi088Detect(const extDevice_t *dev);
bool bmi088SpiAccDetect(accDev_t *acc);
bool bmi088SpiGyroDetect(gyroDev_t *gyro);
