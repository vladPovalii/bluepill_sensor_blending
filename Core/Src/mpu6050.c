/*
 * mpu6050.c
 *
 *  Created on: Apr 23, 2025
 *      Author: vladp
 */

#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // Check device ID WHO_AM_I
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x75, 1, &check, 1, HAL_MAX_DELAY);
    if (check == 0x68) {
        // Wake up MPU6050 - write 0 to power management
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
    }
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t* ax, int16_t* ay, int16_t* az,
					  int16_t* gx, int16_t* gy, int16_t* gz) {
	uint8_t Rec_Data[14];

    // Read 14 bytes starting from ACCEL_XOUT_H
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, HAL_MAX_DELAY);

    // Parse Accel
	*ax = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	*ay = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	*az = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// Skip Temperature (Rec_Data[6] and Rec_Data[7])

	// Parse Gyro
	*gx = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	*gy = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	*gz = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
}
