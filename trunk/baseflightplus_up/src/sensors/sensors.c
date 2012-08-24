/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"

#include "drivers/adc.h"

///////////////////////////////////////////////////////////////////////////////
// Local Definitions
///////////////////////////////////////////////////////////////////////////////

#define ACCEL_BUFF  16
#define GYRO_BUFF   16
#define MAG_BUFF    8
#define BARO_BUFF   8

///////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////

void magSample(void)
{
    SensorSample magSample;
    readMag();
    magSample.x = rawMag[XAXIS];
    magSample.y = rawMag[YAXIS];
    magSample.z = rawMag[ZAXIS];
    bufferWrite(magSampleBuffer, &magSample);
}

///////////////////////////////////////////////////////////////////////////////

void accelSample(void)
{
    SensorSample accelSample;
    
    readAccel();
    
    accelSample.x = rawAccel[XAXIS];
    accelSample.y = rawAccel[YAXIS];
    accelSample.z = rawAccel[ZAXIS];
    bufferWrite(accelSampleBuffer, &accelSample);
}

///////////////////////////////////////////////////////////////////////////////

void gyroSample(void)
{   
    SensorSample gyroSample;
    
    readGyro();
    
    gyroSample.x = rawGyro[XAXIS];
    gyroSample.y = rawGyro[YAXIS];
    gyroSample.z = rawGyro[ZAXIS];
    bufferWrite(gyroSampleBuffer, &gyroSample);
}

///////////////////////////////////////////////////////////////////////////////

void sensorsInit(void)
{
    bufferInit(accelSampleBuffer, accelSamples, ACCEL_BUFF);
    bufferInit(gyroSampleBuffer, gyroSamples, GYRO_BUFF);
    bufferInit(magSampleBuffer, magSamples, MAG_BUFF);
    
    // TODO allow user to select harddware id there are multiple choices
    
    if(mpu6050Detect(gyro, accel, cfg.mpu6050Scale)) {
        set(sensorsAvailable, SENSOR_ACC);
    } else if(!mpu3050Detect(gyro)) {
        failureMode(3);
    }
    
    if(adxl345Detect(accel)) {
        set(sensorsAvailable, SENSOR_ACC);
    }
    
    // At the moment we will do this after mpu6050 and overrride mpu6050 accel if detected
    if(mma8452Detect(accel)) {
        set(sensorsAvailable, SENSOR_ACC);
    }
    
    gyro->init();
    accel->init();
    
    if(cfg.magDriftCompensation)
        initMag();
    set(sensorsAvailable, SENSOR_MAG);
    
#ifdef SONAR
    if(feature(FEATURE_PPM);) {
        hcsr04_init(sonar_rc78);
        set(sensorsAvailable, SENSOR_SONAR);
    }
#else
    if (ms5611Detect(baro) || bmp085Detect(baro)) {
        set(sensorsAvailable, SENSOR_BARO);
        singleEvent(baroUpdate, baro->repeat_delay); // Begin baro conversion state machine
    }
#endif
          
    if(feature(FEATURE_VBAT))
        batteryInit();

}

///////////////////////////////////////////////////////////////////////////////