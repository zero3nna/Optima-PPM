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
    int16_t minMag[3] = {32767, 32767, 32767};
    int16_t maxMag[3] = {-32768, -32768, -32768};
    uint8_t i;
    uint32_t now = millis();

    while (millis() - now < 10000) {
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

    for(i = 0; i < 3; ++i) {
        sensorConfig.magBias[i] = (float)(maxMag[i] + minMag[i]) / 2.0f;
    }
    
    sensorConfig.magCalibrated = true;

}

void readMag(void)
{
    hmc5883Read(rawMag);
}

///////////////////////////////////////////////////////////////////////////////

void initMag(void)
{
    hmc5883Init();
}