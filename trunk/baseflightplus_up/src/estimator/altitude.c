/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */
 
#include "board.h"
#include "sensors/sensors.h"
#include "core/filters.h"

#define ALT_SMOOTH_FACTOR 0.05f

///////////////////////////////////////////////////////////////////////////////

void updateAltitude(void)
{
    int16_t temperature; // 0.1C
    int32_t pressure, altitude; // 1 pa, 1 cm
    
    bmp085Read(&temperature, &pressure, &altitude);
    
    sensors.baroTemperature = (float)temperature / 10.0f;
    sensors.baroPressure = (float)pressure;
    sensors.baroAltitude = filterSmooth(altitude, sensors.baroAltitude, ALT_SMOOTH_FACTOR);
    
    // Sonar/GPS stuff can go here for combination of data, maybe look at including accelerometer as well?
    sensors.altitude = sensors.baroAltitude;
}

///////////////////////////////////////////////////////////////////////////////