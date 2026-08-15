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

    uint8_t pwr = 0x00;

    status = HAL_I2C_Mem_Write(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_PWR_MGMT_1,
        I2C_MEMADD_SIZE_8BIT,
        &pwr, // Set to zero to wake up the MPU6050
        1,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to wake
    }

    // enable interrupts
    uint8_t int_enable = 0x01;
    status = HAL_I2C_Mem_Write(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_INT_ENABLE,
        I2C_MEMADD_SIZE_8BIT,
        &int_enable, // Enable data ready interrupt
        1,
        HAL_MAX_DELAY
    );
    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to enable interrupts
    }

    // enable DLPF at 98Hz (gyro) / 94Hz (accel)
    /* 
    Gyro sample output rate becomes 1kHz. 98Hz/94Hz bandwidth is a middleground between noise filtering and response time while
    avoiding attenuating movements we want to measure (for this project, movements are unlikely to be faster than 98Hz/94Hz).
    */
    uint8_t config = 0x02;
    status = HAL_I2C_Mem_Write(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        &config, // Set DLPF to 98Hz/94Hz
        1,
        HAL_MAX_DELAY
    );
    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to set DLPF
    }

    // set sample rate divider ot 9 -> sample rate = gyro output rate / (1 + SMPLRT_DIV) = 1kHz / (1 + 9) = 100Hz
    uint8_t smplrt_div = 0x09;
    status = HAL_I2C_Mem_Write(
        &hi2c1, MPU6050_ADDR << 1,
        MPU6050_SMPLRT_DIV,
        I2C_MEMADD_SIZE_8BIT,
        &smplrt_div, // Set sample rate divider to 9
        1,
        HAL_MAX_DELAY
    );
    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to set sample rate divider
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