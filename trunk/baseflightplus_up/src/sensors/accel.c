/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
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
// Accel Initialization
///////////////////////////////////////////////////////////////////////////////

void initAccel(void)
{
    accel->init();
}

///////////////////////////////////////////////////////////////////////////////