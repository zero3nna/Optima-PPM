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

#include "board.h"

#include "drivers/adc.h"

///////////////////////////////////////////////////////////////////////////////
// Local Definitions
///////////////////////////////////////////////////////////////////////////////

sensors_t sensors;
uint16_t sensorsAvailable = 0;

SensorSamples accelSamples;
SensorSamples gyroSamples;
SensorSamples magSamples;

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
    readMag();
    magSamples.accum[XAXIS] += rawMag[XAXIS];
    magSamples.accum[YAXIS] += rawMag[YAXIS];
    magSamples.accum[ZAXIS] += rawMag[ZAXIS];
    magSamples.numSamples++;
}

///////////////////////////////////////////////////////////////////////////////

void accelSample(void)
{   
    readAccel();
    accelSamples.accum[XAXIS] += rawAccel[XAXIS];
    accelSamples.accum[YAXIS] += rawAccel[YAXIS];
    accelSamples.accum[ZAXIS] += rawAccel[ZAXIS];
    accelSamples.numSamples++;
}

///////////////////////////////////////////////////////////////////////////////

void gyroSample(void)
{   
    readGyro();  
    gyroSamples.accum[XAXIS] += rawGyro[XAXIS];
    gyroSamples.accum[YAXIS] += rawGyro[YAXIS];
    gyroSamples.accum[ZAXIS] += rawGyro[ZAXIS];
    gyroSamples.numSamples++;
}

///////////////////////////////////////////////////////////////////////////////

void zeroSensorSamples(SensorSamples* samples) {
    uint8_t i;
    
    for(i = 0; i < 3; ++i)
        samples->accum[i] = 0;
        
    samples->numSamples = 0;
}

void sensorsInit(void)
{
    zeroSensorSamples(&accelSamples);
    zeroSensorSamples(&gyroSamples);
    zeroSensorSamples(&magSamples);
    
    // TODO allow user to select harddware if there are multiple choices
    
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
    
    if(hmc5883Detect(mag)) {
        mag->init();
        set(sensorsAvailable, SENSOR_MAG);
    }
    
    
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
          
    if(featureGet(FEATURE_VBAT))
        batteryInit();

}

///////////////////////////////////////////////////////////////////////////////