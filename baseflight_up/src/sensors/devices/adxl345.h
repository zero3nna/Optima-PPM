#pragma once

void adxl345Init(void);

uint8_t adxl345Detect(accel_t *accel);

void adxl345Read(int16_t *values);