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
#include "sensors/sensors.h"

RawSensorData sensorData;
SensorParameters sensorParams;
static uint32_t enabledSensors = 0;

accel_t accel;
baro_t baro;
gyro_t gyro;
mag_t mag;

void batterySample(void)
{
    static uint8_t ind;
    static uint16_t batSamples[8];
    uint16_t batAccum = 0;
    uint8_t i;
    
    batSamples[(ind++) % 8] = adcGet();
    for(i = 0; i < 8; ++i)
        batAccum += batSamples[i];
    sensorData.batteryVoltage = batteryAdcToVoltage(batAccum / 8.0f);
    
    // TODO - add buzzer stuff
    /*
    if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) { // VBAT ok, buzzer off
        buzzerFreq = 0;
    } else
        buzzerFreq = 4;     // low battery
    */
}

void accelSample(void)
{   
    uint8_t i;
    accel.read(sensorData.accel);
    
    for(i = 0; i < 3; ++i)
        sensorData.accelAccum[i] += sensorData.accel[i] - cfg.accelBias[i];
        
    sensorData.accelSamples++;
}

void gyroSample(void)
{   
    uint8_t i;
    gyro.read(sensorData.gyro);
    
    for(i = 0; i < 3; ++i)
        sensorData.gyroAccum[i] += sensorData.gyro[i] - sensorParams.gyroRTBias[i];
        
    sensorData.gyroSamples++;
}

void magSample(void)
{
    uint8_t i;
    mag.read(sensorData.mag);
    
    for(i = 0; i < 3; ++i)
        sensorData.magAccum[i] += sensorData.mag[i] - cfg.magBias[i];
        
    sensorData.magSamples++;
}

void zeroSensorAccumulators(void)
{
    uint8_t i;
    
    for(i = 0; i < 3; ++i) {
        sensorData.accelAccum[i] = 0.0f;
        sensorData.gyroAccum[i] = 0.0f;
        sensorData.magAccum[i] = 0.0f;
    }
    
    sensorData.accelSamples = 0;
    sensorData.gyroSamples = 0;
    sensorData.magSamples = 0;
}


bool sensorsGet(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}

void sensorsInit(void)
{
    zeroSensorAccumulators();
    
    // TODO allow user to select hardware if there are multiple choices
    
    if(mpu6050Detect(&gyro, &accel, cfg.mpu6050Scale)) {
        sensorsSet(SENSOR_ACC);
    } else if(!mpu3050Detect(&gyro)) {
        failureMode(3);
    }
    
    if(adxl345Detect(&accel)) {
        sensorsSet(SENSOR_ACC);
    }
    
    // At the moment we will do this after mpu6050 and overrride mpu6050 accel if detected
    if(mma8452Detect(&accel)) {
        sensorsSet(SENSOR_ACC);
    }
    
    gyro.init();
    
    if(sensorsGet(SENSOR_ACC))
        accel.init();
    
    if(hmc5883Detect(&mag)) {
        mag.init();
        sensorsSet(SENSOR_MAG);
    }
     
#ifdef SONAR
    if(feature(FEATURE_PPM);) {
        hcsr04_init(sonar_rc78);
        sensorsSet(SENSOR_SONAR);
    }
#else
    if (ms5611Detect(&baro) || bmp085Detect(&baro)) {
        sensorsSet(SENSOR_BARO);
        singleEvent(baroUpdate, baro.repeat_delay); // Begin baro conversion state machine
    }
#endif
          
    if(featureGet(FEATURE_VBAT))
        batteryInit();
}

