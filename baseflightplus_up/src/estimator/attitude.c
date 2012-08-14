/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */

#include "board.h"
#include "core/ring_buffer.h"
#include "estimator/lp_filter.h"

fourthOrderData_t accelFilter[3];

void updateAttitude(void)
{
    static uint32_t now, last;
    float dT;
    
    now = micros();
    dT = (float)(now - last) * 1e-6f;
    last = now;
    
    uint8_t numAccelSamples = 0;
    int32_t accelAccum[3] = {0, 0, 0};
    SensorSample accelSample;
    
    uint8_t numGyroSamples = 0;
    int32_t gyroAccum[3] = {0, 0, 0};
    SensorSample gyroSample;
    
    uint8_t numMagSamples = 0;
    int32_t magAccum[3] = {0, 0, 0};
    SensorSample magSample;
    
    while(bufferUsed(magSampleBuffer)) {
        bufferRead(magSampleBuffer, &magSample);
        magAccum[XAXIS] += magSample.x;
        magAccum[YAXIS] += magSample.y;
        magAccum[ZAXIS] += magSample.z;
        ++numMagSamples;
    }
    
    if(numMagSamples) {
        sensors.mag[XAXIS] = ((float) magAccum[XAXIS] / numMagSamples - sensorConfig.magBias[XAXIS]) * sensors.magScaleFactor[XAXIS];
    	sensors.mag[YAXIS] = ((float) magAccum[YAXIS] / numMagSamples - sensorConfig.magBias[YAXIS]) * sensors.magScaleFactor[YAXIS];
    	sensors.mag[ZAXIS] = ((float) magAccum[ZAXIS] / numMagSamples - sensorConfig.magBias[ZAXIS]) * sensors.magScaleFactor[ZAXIS]; 
    }
    
    while(bufferUsed(accelSampleBuffer)) {
        bufferRead(accelSampleBuffer, &accelSample);
        accelAccum[XAXIS] += accelSample.x;
        accelAccum[YAXIS] += accelSample.y;
        accelAccum[ZAXIS] += accelSample.z;
        ++numAccelSamples;
    }
    
    while(bufferUsed(gyroSampleBuffer)) {
        bufferRead(gyroSampleBuffer, &gyroSample);
        gyroAccum[XAXIS] += gyroSample.x;
        gyroAccum[YAXIS] += gyroSample.y;
        gyroAccum[ZAXIS] += gyroSample.z;
        ++numGyroSamples;
    }
    
    if(numAccelSamples) {
        sensors.accel[XAXIS] = ((float) accelAccum[XAXIS] / numAccelSamples - sensors.accelRTBias[XAXIS] - sensorConfig.accelBias[XAXIS]) * sensorConfig.accelScaleFactor[XAXIS];
        sensors.accel[YAXIS] = ((float) accelAccum[YAXIS] / numAccelSamples - sensors.accelRTBias[YAXIS] - sensorConfig.accelBias[YAXIS]) * sensorConfig.accelScaleFactor[YAXIS];
        sensors.accel[ZAXIS] = ((float) accelAccum[ZAXIS] / numAccelSamples - sensors.accelRTBias[ZAXIS] - sensorConfig.accelBias[ZAXIS]) * sensorConfig.accelScaleFactor[ZAXIS];
        //sensors.accel[XAXIS] = computeFourthOrder200Hz(sensors.accel[XAXIS], &accelFilter[XAXIS]);
        //sensors.accel[YAXIS] = computeFourthOrder200Hz(sensors.accel[YAXIS], &accelFilter[YAXIS]);
        //sensors.accel[ZAXIS] = computeFourthOrder200Hz(sensors.accel[ZAXIS], &accelFilter[ZAXIS]);
    }

    if(numGyroSamples) {
        computeGyroTCBias();
        sensors.gyro[XAXIS] = ((float) gyroAccum[XAXIS]  / numGyroSamples - sensors.gyroRTBias[XAXIS] - sensors.gyroTCBias[XAXIS]) * sensors.gyroScaleFactor[XAXIS];
        sensors.gyro[YAXIS] = ((float) gyroAccum[YAXIS]  / numGyroSamples - sensors.gyroRTBias[YAXIS] - sensors.gyroTCBias[YAXIS]) * sensors.gyroScaleFactor[YAXIS];
        sensors.gyro[ZAXIS] = ((float) gyroAccum[ZAXIS]  / numGyroSamples - sensors.gyroRTBias[ZAXIS] - sensors.gyroTCBias[ZAXIS]) * sensors.gyroScaleFactor[ZAXIS];
    }
    if(numMagSamples && sensorConfig.magDriftCompensation) {
        MahonyAHRSupdate( sensors.gyro[ROLL],   -sensors.gyro[PITCH],  sensors.gyro[YAW],
                        sensors.accel[XAXIS], sensors.accel[YAXIS], sensors.accel[ZAXIS],
                        sensors.mag[XAXIS],    sensors.mag[YAXIS],    sensors.mag[ZAXIS],
                        dT);
    } else {
        MahonyAHRSupdateIMU( sensors.gyro[ROLL],   -sensors.gyro[PITCH],  sensors.gyro[YAW],
                        sensors.accel[XAXIS], sensors.accel[YAXIS], sensors.accel[ZAXIS],
                        dT);
    }
    
    
    float q[4] = {q0, q1, q2, q3};
    
    Quaternion2RPY(q, sensors.attitude);

}