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
          
    if(featureGet(FEATURE_VBAT))
        batteryInit();

}

///////////////////////////////////////////////////////////////////////////////