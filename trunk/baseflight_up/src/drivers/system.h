/*
    BaseflightPlus U.P
    Copyright (C) 2012 Scott Driessens

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#pragma once

#define TIMER_MAX_EVENTS 16

/* Course timer utilities */
typedef void (*event_callback)(void);

/* Limited to TIMER_MAX_EVENTS */
void singleEvent(event_callback callback, uint32_t delay);

void periodicEvent(event_callback callback, uint32_t period);

void eventCallbacks(void);

void printEventDeltas(void);

void delayMicroseconds(uint32_t us);

void delay(uint32_t ms);

uint32_t micros(void);

uint32_t millis(void);

void failureMode(uint8_t mode);

void systemReset(bool toBootloader);

void systemInit(void);

void failureMode(uint8_t mode);