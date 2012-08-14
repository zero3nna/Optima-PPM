/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */
 
#include "board.h"
#include "sensors/sensors.h"

void updateAltitude(void)
{
    uint8_t numBaroSamples = 0;
    int32_t baroAccum = 0;
    int32_t baroSample;
    
    while(bufferUsed(baroSampleBuffer)) {
        bufferRead(baroSampleBuffer, &baroSample);
        baroAccum += baroSample;
        ++numBaroSamples;
    }
    
    pressureAverage = baroAccum / numBaroSamples;
	calculateTemperature();
	calculatePressureAltitude();
	sensors.pressureAlt = pressureAlt;  
}