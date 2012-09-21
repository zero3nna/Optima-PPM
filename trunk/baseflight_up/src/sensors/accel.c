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
#include "sensors/sensors.h"

#define CALIBRATION_SAMPLES 2000

void accelCalibration(void)
{
    uint16_t samples;
    int32_t accelSum[3] = { 0, 0, 0 };

    for (samples = 0; samples < CALIBRATION_SAMPLES; ++samples) {
        accel.read(sensorData.accel);

        accelSum[XAXIS] += sensorData.accel[XAXIS];
        accelSum[YAXIS] += sensorData.accel[YAXIS];
        accelSum[ZAXIS] += sensorData.accel[ZAXIS];
        
        if(!(samples % 250))
            LED0_TOGGLE();

        delay(1);
    }

    cfg.accelBias[XAXIS] = accelSum[XAXIS] / CALIBRATION_SAMPLES;
    cfg.accelBias[YAXIS] = accelSum[YAXIS] / CALIBRATION_SAMPLES;
    cfg.accelBias[ZAXIS] = (accelSum[ZAXIS] / CALIBRATION_SAMPLES) + (int32_t)(ACCEL_1G / fabs(sensorParams.accelScaleFactor));
    
    cfg.accelCalibrated = true;
}