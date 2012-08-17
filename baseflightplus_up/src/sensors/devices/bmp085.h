#pragma once

bool bmp085Init(void);

// polled versions
int16_t bmp085_read_temperature(void);
int32_t bmp085_read_pressure(void);

// interrupt versions
void bmp085_start_ut(void);
uint16_t bmp085_get_ut(void);
void bmp085_start_up(void);
uint32_t bmp085_get_up(void);
int16_t bmp085_get_temperature(uint32_t ut);
int32_t bmp085_get_pressure(uint32_t up);

void bmp085Update(void);
void bmp085Read(int16_t *temperature, int32_t *pressure, int32_t *altitude);