#pragma once

void mpu6050AccelInit(void);

void mpu6050GyroInit(void);

uint8_t mpu6050Detect(gyro_t *gyro, accel_t *accel);

void mpu6050AccelRead(int16_t *accValues);

void mpu6050GyroRead(int16_t *gyroValues);

void mpu6050TempRead(int16_t* temperature);
