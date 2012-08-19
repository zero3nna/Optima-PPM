/*
 * From Timecop's Original Baseflight
 */
 
#pragma once

typedef void (* uartReceiveCallbackPtr)(uint16_t data);     // used by uart2 driver to return frames to app

void uartInit(uint32_t speed);
uint16_t uartAvailable(void);
bool uartTransmitEmpty(void);
uint8_t uartRead(void);
uint8_t uartReadPoll(void);
void uartWrite(uint8_t ch);
void uartPrint(char *str);
void uart2Init(uint32_t speed, uartReceiveCallbackPtr func);