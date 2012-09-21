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
#include "core/filters.h"

#define ALT_TAB_SIZE   20

void updateAltitude(void)
{
    uint32_t index;
    static int16_t altHistTab[ALT_TAB_SIZE];
    static uint32_t altHistIdx;
    static int32_t altHigh = 0;

#ifdef SONAR
    int16_t sonarAltitude;
    hcsr04_get_distance(&sonarAltitude);
    altHistTab[altHistIdx] = sonarAltitude / 10;
#else
    altHistTab[altHistIdx] = sensorData.baroAltitude / 10;
#endif
    altHigh += altHistTab[altHistIdx];
    index = (altHistIdx + (ALT_TAB_SIZE / 2)) % ALT_TAB_SIZE;
    altHigh -= altHistTab[index];
    altHistIdx++;
    if (altHistIdx >= ALT_TAB_SIZE)
        altHistIdx = 0;

    stateData.altitude = altHigh * 10 / (ALT_TAB_SIZE / 2);
}

