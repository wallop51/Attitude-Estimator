#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_Init(void) {
    uint8_t id;
    HAL_StatusTypeDef status;

    // check MPU6050 ID
    status = HAL_I2C_Mem_Read(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_WHO_AM_I,
        I2C_MEMADD_SIZE_8BIT,
        &id,
        1,
        HAL_MAX_DELAY
    );
    
    if (status != HAL_OK || id != 0x68) {
        return HAL_ERROR; // Communication failed or wrong ID
    }

    status = HAL_I2C_Mem_Write(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_PWR_MGMT_1,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t)0x00, // Set to zero to wake up the MPU6050
        1,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to wake
    }

    // enable interrupts
    status = HAL_I2C_Mem_Write(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_INT_ENABLE,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t)0x01, // Enable data ready interrupt
        1,
        HAL_MAX_DELAY
    );
    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to enable interrupts
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Accel(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z) {

    uint8_t buffer[6];

    /*
    HAL_I2C_Mem_Read reads 6 bytes from the MPU6050 starting at the ACCEL_XOUT_H register.
    the data is read into the buffer and then split into xyz values
    */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_ACCEL_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        6,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    *accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    *accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    *accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_Gyro(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z) {

    uint8_t buffer[6];

    /*
    HAL_I2C_Mem_Read reads 6 bytes from the MPU6050 starting at the GYRO_XOUT_H register.
    the data is read into the buffer and then split into xyz values
    */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_GYRO_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        6,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    *gyro_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    *gyro_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    *gyro_z = (int16_t)(buffer[4] << 8 | buffer[5]);

    return HAL_OK;
}