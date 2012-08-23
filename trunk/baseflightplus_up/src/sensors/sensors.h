/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

#include "board.h"
#include "core/ring_buffer.h"

///////////////////////////////////////////////////////////////////////////////
// Sensor Definitions
///////////////////////////////////////////////////////////////////////////////

#define SENSOR_ACC      1 << 0
#define SENSOR_BARO     1 << 1
#define SENSOR_MAG      1 << 2
#define SENSOR_SONAR    1 << 3
#define SENSOR_GPS      1 << 4

///////////////////////////////////////////////////////////////////////////////
// Sensor Typedefs
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    float accel[3]; // m/s/s
    float accelScaleFactor[3];
    
    float attitude[3]; // rad
    
    float gyro[3]; // rad/s
    float gyroScaleFactor[3];
    float gyroRTBias[3];
    float gyroTCBias[3];
    float gyroTemperature;
    
    float mag[3];
    float magScaleFactor[3];
    
    int32_t baroAltitude;
    int32_t altitude;
    
    float batteryVoltage;
    float batteryWarningVoltage;
    uint8_t batteryCellCount;
} sensors_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SensorSample;

ringBuffer_typedef(SensorSample, AccelBuffer);
ringBuffer_typedef(SensorSample, GyroBuffer);
ringBuffer_typedef(SensorSample, MagBuffer);

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
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t repeat_delay;
    sensorFuncPtr start_ut;
    sensorFuncPtr get_ut;
    sensorFuncPtr start_up;
    sensorFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

extern uint16_t sensorsAvailable;
extern sensors_t sensors;
extern AccelBuffer *accelSampleBuffer;
extern GyroBuffer *gyroSampleBuffer;
extern MagBuffer *magSampleBuffer;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void magSample(void);

void accelSample(void);

void gyroSample(void);

void readMag();

void batterySample(void);

void sensorsInit(void);

///////////////////////////////////////////////////////////////////////////////
