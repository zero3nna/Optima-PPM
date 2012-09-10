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

mag_t _mag;
mag_t *mag = &_mag;

int16_t rawMag[3];

///////////////////////////////////////////////////////////////////////////////
// Magnetometer Calibration
///////////////////////////////////////////////////////////////////////////////

void magCalibration(void)
{
    int16_t minMag[3];
    int16_t maxMag[3];
    uint8_t i;
    uint16_t samples;
    
    readMag();
    
    for(i = 0; i < 3; ++i) {
        minMag[i] = rawMag[i];
        maxMag[i] = rawMag[i];
    }

    LED0_ON();
    LED1_OFF();
    for(samples = 0; samples < 500; ++samples) {
        readMag();
        
        for(i = 0; i < 3; ++i) {
            if(rawMag[i] > maxMag[i]) {
                maxMag[i] = rawMag[i];
            }
            if(rawMag[i] < minMag[i]) {
                minMag[i] = rawMag[i];
            }
            delay(20);
        }   
    };
    LED0_OFF();

    for(i = 0; i < 3; ++i) {
        cfg.magBias[i] = (float)(maxMag[i] + minMag[i]) / 2.0f;
    }
    
    cfg.magCalibrated = true;

}

///////////////////////////////////////////////////////////////////////////////

void readMag(void)
{
    mag->read(rawMag);
}

///////////////////////////////////////////////////////////////////////////////