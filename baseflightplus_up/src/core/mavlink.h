#pragma once

void mavlinkInit(void);

void mavlinkSendData(void);

void mavlinkComm(void);

extern uint8_t mavlinkMode;

// UART2 Receive ISR callback
void currentDataReceive(uint16_t c);