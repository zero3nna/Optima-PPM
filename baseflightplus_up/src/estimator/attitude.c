/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */

#include "board.h"
#include "core/ring_buffer.h"
#include "core/filters.h"

fourthOrderData_t accelFilter[3];

volatile float q0 = -1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame

void updateAttitude(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    
    static uint32_t now, last;
    float dT;
    
    now = micros();
    dT = (float)(now - last) * 1e-6f;
    last = now;
    
    ///////////////////////////////////////////////////////////////////////////////
    
    uint8_t numAccelSamples = 0;
    int32_t accelAccum[3] = {0, 0, 0};
    SensorSample accelSample;
    
    uint8_t numGyroSamples = 0;
    int32_t gyroAccum[3] = {0, 0, 0};
    SensorSample gyroSample;
    
    uint8_t numMagSamples = 0;
    int32_t magAccum[3] = {0, 0, 0};
    SensorSample magSample;
    
    ///////////////////////////////////////////////////////////////////////////////
    
    while(bufferUsed(magSampleBuffer)) {
        bufferRead(magSampleBuffer, &magSample);
        magAccum[XAXIS] += magSample.x;
        magAccum[YAXIS] += magSample.y;
        magAccum[ZAXIS] += magSample.z;
        ++numMagSamples;
    }
    
    if(numMagSamples) {
        sensors.mag[XAXIS] = ((float) magAccum[XAXIS] / numMagSamples - cfg.magBias[XAXIS]) * sensors.magScaleFactor[XAXIS];
    	sensors.mag[YAXIS] = ((float) magAccum[YAXIS] / numMagSamples - cfg.magBias[YAXIS]) * sensors.magScaleFactor[YAXIS];
    	sensors.mag[ZAXIS] = ((float) magAccum[ZAXIS] / numMagSamples - cfg.magBias[ZAXIS]) * sensors.magScaleFactor[ZAXIS]; 
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    while(bufferUsed(accelSampleBuffer)) {
        bufferRead(accelSampleBuffer, &accelSample);
        accelAccum[XAXIS] += accelSample.x;
        accelAccum[YAXIS] += accelSample.y;
        accelAccum[ZAXIS] += accelSample.z;
        ++numAccelSamples;
    }
    
    if(numAccelSamples) {
        sensors.accel[XAXIS] = ((float) accelAccum[XAXIS] / numAccelSamples - cfg.accelBias[XAXIS]) * sensors.accelScaleFactor[XAXIS];
        sensors.accel[YAXIS] = ((float) accelAccum[YAXIS] / numAccelSamples - cfg.accelBias[YAXIS]) * sensors.accelScaleFactor[YAXIS];
        sensors.accel[ZAXIS] = ((float) accelAccum[ZAXIS] / numAccelSamples - cfg.accelBias[ZAXIS]) * sensors.accelScaleFactor[ZAXIS];
        if(cfg.accelLPF) {
            sensors.accel[XAXIS] = fourthOrderFilter(sensors.accel[XAXIS], &accelFilter[XAXIS], cfg.accelLPF_A, cfg.accelLPF_B);
            sensors.accel[YAXIS] = fourthOrderFilter(sensors.accel[YAXIS], &accelFilter[YAXIS], cfg.accelLPF_A, cfg.accelLPF_B);
            sensors.accel[ZAXIS] = fourthOrderFilter(sensors.accel[ZAXIS], &accelFilter[ZAXIS], cfg.accelLPF_A, cfg.accelLPF_B);
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    while(bufferUsed(gyroSampleBuffer)) {
        bufferRead(gyroSampleBuffer, &gyroSample);
        gyroAccum[XAXIS] += gyroSample.x;
        gyroAccum[YAXIS] += gyroSample.y;
        gyroAccum[ZAXIS] += gyroSample.z;
        ++numGyroSamples;
    }

    if(numGyroSamples) {
        readGyroTemp();
        computeGyroTCBias();
        sensors.gyro[XAXIS] = ((float) gyroAccum[XAXIS]  / numGyroSamples - sensors.gyroRTBias[XAXIS] - sensors.gyroTCBias[XAXIS]) * sensors.gyroScaleFactor[XAXIS];
        sensors.gyro[YAXIS] = ((float) gyroAccum[YAXIS]  / numGyroSamples - sensors.gyroRTBias[YAXIS] - sensors.gyroTCBias[YAXIS]) * sensors.gyroScaleFactor[YAXIS];
        sensors.gyro[ZAXIS] = ((float) gyroAccum[ZAXIS]  / numGyroSamples - sensors.gyroRTBias[ZAXIS] - sensors.gyroTCBias[ZAXIS]) * sensors.gyroScaleFactor[ZAXIS];
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    if(numMagSamples && cfg.magDriftCompensation) {
        MahonyAHRSupdate( sensors.gyro[ROLL],   -sensors.gyro[PITCH],  sensors.gyro[YAW],
                        sensors.accel[XAXIS], sensors.accel[YAXIS], sensors.accel[ZAXIS],
                        sensors.mag[XAXIS],    sensors.mag[YAXIS],    sensors.mag[ZAXIS],
                        dT);
    } else {
        MahonyAHRSupdateIMU( sensors.gyro[ROLL],   -sensors.gyro[PITCH],  sensors.gyro[YAW],
                        sensors.accel[XAXIS], sensors.accel[YAXIS], sensors.accel[ZAXIS],
                        dT);
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    float q[4] = {q0, q1, q2, q3};
    
    Quaternion2RPY(q, sensors.attitude);
    
    sensors.attitude[YAW] += cfg.magDeclination * DEG2RAD;
    
    ///////////////////////////////////////////////////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////