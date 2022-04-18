#ifndef _MPU6050_H
#define _MPU6050_H

#include "main.h"

typedef enum
{
    MPU6050_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
    MPU6050_RANGE_4_G = 0b01,  ///< +/- 4g
    MPU6050_RANGE_8_G = 0b10,  ///< +/- 8g
    MPU6050_RANGE_16_G = 0b11, ///< +/- 16g
} mpu6050_accel_range_t;

typedef enum
{
    MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
    MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
    MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
    MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

typedef enum
{
    MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
    MPU6050_BAND_184_HZ, ///< 184 Hz
    MPU6050_BAND_94_HZ,  ///< 94 Hz
    MPU6050_BAND_44_HZ,  ///< 44 Hz
    MPU6050_BAND_21_HZ,  ///< 21 Hz
    MPU6050_BAND_10_HZ,  ///< 10 Hz
    MPU6050_BAND_5_HZ,   ///< 5 Hz
} mpu6050_bandwidth_t;

uint8_t mpuBegin(I2C_HandleTypeDef *i2cdev);

void setMPUFilterBandwidth(mpu6050_bandwidth_t bw);
void setMPUAccelRange(mpu6050_accel_range_t accel_range);
void setMPUGyroRange(mpu6050_gyro_range_t gyro_range);

void readMPUAccel(float *gx, float *gy, float *gz);
void readMPUGyro(float *gx, float *gy, float *gz);

#endif /* _MPU6050_H */
