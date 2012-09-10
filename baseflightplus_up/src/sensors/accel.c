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

///////////////////////////////////

int16_t rawAccel[3];

accel_t _accel;
accel_t *accel = &_accel;

///////////////////////////////////////////////////////////////////////////////
// Read Accel
///////////////////////////////////////////////////////////////////////////////

void readAccel(void)
{
    accel->read(rawAccel);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Accel Runtime Bias
///////////////////////////////////////////////////////////////////////////////

void accelCalibration(void)
{
    uint16_t samples;
    float accelSum[3] = { 0, 0, 0 };

    for (samples = 0; samples < 2000; ++samples) {
        readAccel();

        accelSum[XAXIS] += rawAccel[XAXIS];
        accelSum[YAXIS] += rawAccel[YAXIS];
        accelSum[ZAXIS] += rawAccel[ZAXIS];
        
        if(!(samples % 250))
            LED0_TOGGLE();

        delay(1);
    }

    cfg.accelBias[XAXIS] = accelSum[XAXIS] / 2000.0f;
    cfg.accelBias[YAXIS] = accelSum[YAXIS] / 2000.0f;
    cfg.accelBias[ZAXIS] = (accelSum[ZAXIS] / 2000.0f) + (ACCEL_1G / fabs(sensors.accelScaleFactor));
    
    cfg.accelCalibrated = true;
}

///////////////////////////////////////////////////////////////////////////////