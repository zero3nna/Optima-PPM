/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define TIMER_MAX_EVENTS 16

/* Course timer utilities */
typedef void (*event_callback)(void);

/* Limited to TIMER_MAX_EVENTS */
void singleEvent(event_callback callback, uint32_t delay);

void periodicEvent(event_callback callback, uint32_t period);

void eventCallbacks(void);

void printEventDeltas(void);

///////////////////////////////////////////////////////////////////////////////

void delayMicroseconds(uint32_t us);

void delay(uint32_t ms);

uint32_t micros(void);

uint32_t millis(void);

void failureMode(uint8_t mode);

void systemReset(bool toBootloader);

void systemInit(void);

///////////////////////////////////////////////////////////////////////////////
