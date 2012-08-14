#pragma once

bool mpu6050Init(void);

void mpu6050Read(int16_t accValues[3], int16_t gyroValues[3], int16_t* temperature);
