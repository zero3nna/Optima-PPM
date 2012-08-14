/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

#include "board.h"
#include "core/ring_buffer.h"

///////////////////////////////////////////////////////////////////////////////
// Sensor Types
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SensorSample;

typedef struct {
    float accel[3]; // m/s/s
    float accelRTBias[3];
    
    float attitude[3]; // rad
    
    float gyro[3]; // rad/s
    float gyroScaleFactor[3];
    float gyroRTBias[3];
    float gyroTCBias[3];
    float gyroTemperature;
    
    float mag[3];
    float magScaleFactor[3];
    float pressureAlt;
    
    float batteryVoltage;
    float batteryWarningVoltage;
    uint8_t batteryCellCount;
} sensors_t;

ringBuffer_typedef(SensorSample, AccelBuffer);
ringBuffer_typedef(SensorSample, GyroBuffer);
ringBuffer_typedef(SensorSample, MagBuffer);
ringBuffer_typedef(int32_t, BaroBuffer);

extern sensors_t sensors;

///////////////////////////////////////////////////////////////////////////////
// Sensor reading variables
///////////////////////////////////////////////////////////////////////////////
extern int32_t pressureSum;

extern AccelBuffer *accelSampleBuffer;
extern GyroBuffer *gyroSampleBuffer;
extern MagBuffer *magSampleBuffer;
extern BaroBuffer *baroSampleBuffer;

void magSample(void);

void baroSample(void);

void accelSample(void);

void gyroSample(void);

void readMag();

void batterySample(void);

void sensorsInit(void);

