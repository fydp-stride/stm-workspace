#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#include "stm32l4xx_hal.h"
#include "bno055.h"

extern I2C_HandleTypeDef hi2c1;

void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device);
void bno055_delay(int time);
void bno055_writeData(uint8_t reg, uint8_t data);
void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len);

#endif  // BNO055_STM32_H_
