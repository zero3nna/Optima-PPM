/*
 * From Timecop's Original Baseflight
 */

#pragma once

extern uint16_t failsafeCount;

void pwmInit(void); // returns whether driver is asking to calibrate throttle or not
void pwmWrite(uint8_t channel, uint16_t value);
uint16_t pwmRead(uint8_t channel);
uint8_t pwmGetNumOutputChannels(void);