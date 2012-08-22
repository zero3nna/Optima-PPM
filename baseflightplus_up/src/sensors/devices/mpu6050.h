#pragma once

bool mpu6050Detect(gyro_t *gyro, accel_t *accel, uint8_t scale);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
