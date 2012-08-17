/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"

///////////////////////////////////////
// Sensor readings
///////////////////////////////////////

#define ACCEL_BUFF  16
#define GYRO_BUFF   16
#define MAG_BUFF    8
#define BARO_BUFF   8

sensors_t sensors;
uint16_t sensorsAvailable = 0;

AccelBuffer _accelSampleBuffer;
AccelBuffer *accelSampleBuffer = &_accelSampleBuffer;
SensorSample accelSamples[ACCEL_BUFF];

GyroBuffer _gyroSampleBuffer;
GyroBuffer *gyroSampleBuffer = &_gyroSampleBuffer;
SensorSample gyroSamples[GYRO_BUFF];

MagBuffer _magSampleBuffer;
MagBuffer *magSampleBuffer = &_magSampleBuffer;
SensorSample magSamples[MAG_BUFF];

void batterySample(void)
{
    static uint8_t ind;
    static uint16_t batSamples[8];
    uint16_t batAccum = 0;
    uint8_t i;
    
    batSamples[(ind++) % 8] = adcGet();
    for(i = 0; i < 8; ++i)
        batAccum += batSamples[i];
    sensors.batteryVoltage = batteryAdcToVoltage(batAccum / 8.0f);
    
    // TODO - add buzzer stuff
    /*
    if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) { // VBAT ok, buzzer off
        buzzerFreq = 0;
    } else
        buzzerFreq = 4;     // low battery
    */
}

void magSample(void)
{
    SensorSample magSample;
    readMag();
    magSample.x = rawMag[XAXIS];
    magSample.y = rawMag[YAXIS];
    magSample.z = rawMag[ZAXIS];
    bufferWrite(magSampleBuffer, &magSample);
}

void accelSample(void)
{
    SensorSample accelSample;
    
    readAccel();
    
    accelSample.x = rawAccel[XAXIS];
    accelSample.y = rawAccel[YAXIS];
    accelSample.z = rawAccel[ZAXIS];
    bufferWrite(accelSampleBuffer, &accelSample);
}

void gyroSample(void)
{   
    SensorSample gyroSample;
    
    readGyro();
    
    gyroSample.x = rawGyro[XAXIS];
    gyroSample.y = rawGyro[YAXIS];
    gyroSample.z = rawGyro[ZAXIS];
    bufferWrite(gyroSampleBuffer, &gyroSample);
}

void sensorsInit(void)
{
    bufferInit(accelSampleBuffer, accelSamples, ACCEL_BUFF);
    bufferInit(gyroSampleBuffer, gyroSamples, GYRO_BUFF);
    bufferInit(magSampleBuffer, magSamples, MAG_BUFF);
    
    mpu3050Detect(gyro);
    if(mpu6050Detect(gyro, accel))
        set(sensorsAvailable, SENSOR_ACC);
    
    if(adxl345Detect(accel))
        set(sensorsAvailable, SENSOR_ACC);
    
    initGyro();
    initAccel();
    
    initMag();
    set(sensorsAvailable, SENSOR_MAG);
    
    if(bmp085Init())
        set(sensorsAvailable, SENSOR_BARO);
        
    if(cfg.battery)
        batteryInit();
}