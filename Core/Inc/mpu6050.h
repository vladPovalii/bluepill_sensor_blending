/*
 * mpu6050.h
 *
 *  Created on: Apr 23, 2025
 *      Author: vladp
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"

#define MPU6050_ADDR 0xD0  // 0x68 << 1 for HAL (write mode)

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t* ax, int16_t* ay, int16_t* az,
		  int16_t* gx, int16_t* gy, int16_t* gz);

#endif
