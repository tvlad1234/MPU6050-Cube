#include "mpu6050.h"

#define MPU6050_RA_PWR_MGMT_1 0x6B	 // power management register address
#define MPU6050_RA_WHO_AM_I 0x75	 // read-only register containing address (for testing communication)
#define MPU6050_RA_ACCEL_XOUT_H 0x3B // where accel data starts (Xh, Xl, Yh, Yl, Zh, Zl)
#define MPU6050_RA_GYRO_XOUT_H 0x43	 // where gyro data starts (Xh, Xl, Yh, Yl, Zh, Zl)

#define MPU6050_RA_ACCEL_CONF 0x1C // accel config register
#define MPU6050_RA_GYRO_CONF 0x1B  // gyro config register
#define MPU6050_RA_CONF 0x1A	   // general config register

#define MPU6050_DEVICE_ID 0x68
#define MPU6500_DEVICE_ID 0x70

#define SENSORS_GRAVITY_EARTH (9.80665F)

uint16_t mpu_accel_scale;
float mpu_gyro_scale;

static const uint8_t MPU6050_ADDR = 0x68 << 1; // Use 8-bit address
I2C_HandleTypeDef *mpuPort;

void setMPUFilterBandwidth(mpu6050_bandwidth_t bw)
{
	uint8_t a = bw;
	HAL_I2C_Mem_Write(mpuPort, MPU6050_ADDR, MPU6050_RA_CONF, 1, &a, 1,
					  HAL_MAX_DELAY);
}

void setMPUAccelRange(mpu6050_accel_range_t accel_range)
{
	uint8_t a = accel_range << 3;
	HAL_I2C_Mem_Write(mpuPort, MPU6050_ADDR, MPU6050_RA_ACCEL_CONF, 1, &a, 1,
					  HAL_MAX_DELAY);
	if (accel_range == MPU6050_RANGE_16_G)
		mpu_accel_scale = 2048;
	if (accel_range == MPU6050_RANGE_8_G)
		mpu_accel_scale = 4096;
	if (accel_range == MPU6050_RANGE_4_G)
		mpu_accel_scale = 8192;
	if (accel_range == MPU6050_RANGE_2_G)
		mpu_accel_scale = 16384;
}

void setMPUGyroRange(mpu6050_gyro_range_t gyro_range)
{
	uint8_t a = gyro_range << 3;
	HAL_I2C_Mem_Write(mpuPort, MPU6050_ADDR, MPU6050_RA_GYRO_CONF, 1, &a, 1,
					  HAL_MAX_DELAY);
	if (gyro_range == MPU6050_RANGE_250_DEG)
		mpu_gyro_scale = 131;
	if (gyro_range == MPU6050_RANGE_500_DEG)
		mpu_gyro_scale = 65.5;
	if (gyro_range == MPU6050_RANGE_1000_DEG)
		mpu_gyro_scale = 32.8;
	if (gyro_range == MPU6050_RANGE_2000_DEG)
		mpu_gyro_scale = 16.4;
}

uint8_t mpuBegin(I2C_HandleTypeDef *i2cdev)
{
	mpuPort = i2cdev;
	uint8_t a = 0;
	HAL_Delay(100);
	HAL_I2C_Mem_Write(mpuPort, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &a, 1,
					  HAL_MAX_DELAY);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(mpuPort, MPU6050_ADDR, MPU6050_RA_WHO_AM_I, 1, &a, 1,
					 HAL_MAX_DELAY);
	HAL_Delay(100);

	if (a == MPU6050_DEVICE_ID || a == MPU6500_DEVICE_ID)
	{
		setMPUFilterBandwidth(MPU6050_BAND_260_HZ);
		setMPUAccelRange(MPU6050_RANGE_2_G);
		setMPUGyroRange(MPU6050_RANGE_500_DEG);
		return 1;
	}
	else
		return 0;
}

void readMPURawAccel(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buf[6];
	HAL_I2C_Mem_Read(mpuPort, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 1, buf, 6,
					 HAL_MAX_DELAY);
	*x = (buf[0] << 8 | buf[1]);
	*y = (buf[2] << 8 | buf[3]);
	*z = (buf[4] << 8 | buf[5]);
}

void readMPURawGyro(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buf[6];
	HAL_I2C_Mem_Read(mpuPort, MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H, 1, buf, 6,
					 HAL_MAX_DELAY);
	*x = (buf[0] << 8 | buf[1]);
	*y = (buf[2] << 8 | buf[3]);
	*z = (buf[4] << 8 | buf[5]);
}

void readMPUAccel(float *gx, float *gy, float *gz)
{
	int16_t x, y, z;
	readMPURawAccel(&x, &y, &z);
	*gx = ((float)x / mpu_accel_scale) * SENSORS_GRAVITY_EARTH;
	*gy = ((float)y / mpu_accel_scale) * SENSORS_GRAVITY_EARTH;
	*gz = ((float)z / mpu_accel_scale) * SENSORS_GRAVITY_EARTH;
}

void readMPUGyro(float *gx, float *gy, float *gz)
{
	int16_t x, y, z;
	readMPURawGyro(&x, &y, &z);
	*gx = ((float)x / mpu_gyro_scale);
	*gy = ((float)y / mpu_gyro_scale);
	*gz = ((float)z / mpu_gyro_scale);
}