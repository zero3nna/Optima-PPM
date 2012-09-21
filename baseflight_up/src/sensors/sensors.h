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

#include "board.h"

// Sensor Definitions

#define SENSOR_ACC      1 << 0
#define SENSOR_BARO     1 << 1
#define SENSOR_MAG      1 << 2
#define SENSOR_SONAR    1 << 3
#define SENSOR_GPS      1 << 4

// Sensor Typedefs

typedef struct {
    float accelScaleFactor;
    
    float gyroScaleFactor;
    int32_t gyroRTBias[3];
    float gyroTCBias[3];
    
    float magScaleFactor;
    
} SensorParameters;

// Structure for holding raw sensor data
typedef struct {
    int16_t gyro[3];
    
    int32_t gyroAccum[3];
    
    uint8_t gyroSamples;

    int16_t accel[3];
    
    int32_t accelAccum[3];
    
    uint8_t accelSamples;

    int16_t mag[3];
    
    int32_t magAccum[3];
    
    uint8_t magSamples;

    float gyroTemperature;
    
    int32_t baroAltitude;
    
    float batteryVoltage;
    float batteryWarningVoltage;
    uint8_t batteryCellCount;
    
} RawSensorData;

typedef void (* sensorFuncPtr)(void);                   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef void (* sensorReadFloatFuncPtr)(float *data);
typedef int32_t (* baroCalculateFuncPtr)(void);             // baro calculation (returns altitude in cm based on static data collected)

typedef struct
{
    sensorFuncPtr init;
    sensorReadFuncPtr read;
    sensorReadFloatFuncPtr temperature;
} gyro_t;

typedef struct
{
    sensorFuncPtr init;
    sensorReadFuncPtr read;
} accel_t;

typedef struct
{
    sensorFuncPtr init;
    sensorReadFuncPtr read;
} mag_t;

typedef struct
{
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t repeat_delay;
    sensorFuncPtr start_ut;
    sensorFuncPtr get_ut;
    sensorFuncPtr start_up;
    sensorFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

// External Variables

extern RawSensorData sensorData;
extern SensorParameters sensorParams;

extern accel_t accel;
extern baro_t baro;
extern gyro_t gyro;
extern mag_t mag;

// Functions

void magSample(void);
void accelSample(void);
void gyroSample(void);
void batterySample(void);
void zeroSensorAccumulators(void);

bool sensorsGet(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);

void sensorsInit(void);

