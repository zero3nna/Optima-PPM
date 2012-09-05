#pragma once

void hmc5883Read(int16_t *values);
bool hmc5883Detect(mag_t *mag);
void hmc5883Init(void);
