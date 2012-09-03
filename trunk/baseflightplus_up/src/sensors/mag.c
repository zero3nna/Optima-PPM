/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */
 
#include "board.h"

///////////////////////////////////////////////////////////////////////////////

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
    hmc5883Read(rawMag);
}

///////////////////////////////////////////////////////////////////////////////

void initMag(void)
{
    hmc5883Init();
}

///////////////////////////////////////////////////////////////////////////////