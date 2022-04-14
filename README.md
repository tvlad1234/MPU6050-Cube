# MPU6050-Cube
*MPU6050 Library for STM32Cube*\
Based on [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)

### Initializing the sensor
Before reading the sensor, it must be initialized with _mpuBegin_.\
_mpuBegin_ takes a pointer to the I2C Handler and returns 1 if connection was successful.\
_Example:_ `mpuBegin(&hi2c1);`

### Reading the sensor
`readAccel(float *gx, float *gy, float *gz)` provides the acceleration in m/s^2 \
`readGyro(float *gx, float *gy, float *gz)` provides the angular velocity in deg/s

### Configuring the sensor
The sensor can be configured using the following functions: \
`setMPUFilterBandwidth(mpu6050_bandwidth_t bw)` \
`setAccelRange(mpu6050_accel_range_t accel_range)` \
`setGyroRange(mpu6050_gyro_range_t gyro_range)` 



