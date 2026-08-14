#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

#define MPU6050_ADDR 0x68 
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_INT_ENABLE 0x38

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_ZOUT_H 0x47

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef MPU6050_Init(void);

HAL_StatusTypeDef MPU6050_Read_Accel(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z);
HAL_StatusTypeDef MPU6050_Read_Gyro(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z);

#endif