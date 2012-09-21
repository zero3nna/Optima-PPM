/*/*
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

void magCalibration(void)
{
    int16_t minMag[3];
    int16_t maxMag[3];
    uint8_t i;
    uint16_t samples;
    
    mag.read(sensorData.mag);
    
    for(i = 0; i < 3; ++i) {
        minMag[i] = sensorData.mag[i];
        maxMag[i] = sensorData.mag[i];
    }

    LED0_ON();
    LED1_OFF();
    for(samples = 0; samples < 500; ++samples) {
        mag.read(sensorData.mag);
        
        for(i = 0; i < 3; ++i) {
            if(sensorData.mag[i] > maxMag[i]) {
                maxMag[i] = sensorData.mag[i];
            }
            if(sensorData.mag[i] < minMag[i]) {
                minMag[i] = sensorData.mag[i];
            }
            delay(20);
        }   
    };
    LED0_OFF();

    for(i = 0; i < 3; ++i) {
        cfg.magBias[i] = (maxMag[i] + minMag[i]) / 2;
    }
    
    cfg.magCalibrated = true;

}