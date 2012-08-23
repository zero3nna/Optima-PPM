/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */
 
#include "board.h"
#include "sensors/sensors.h"
#include "core/filters.h"

#define ALT_TAB_SIZE   20

///////////////////////////////////////////////////////////////////////////////

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
    altHistTab[altHistIdx] = sensors.baroAltitude / 10;
#endif
    altHigh += altHistTab[altHistIdx];
    index = (altHistIdx + (ALT_TAB_SIZE / 2)) % ALT_TAB_SIZE;
    altHigh -= altHistTab[index];
    altHistIdx++;
    if (altHistIdx >= ALT_TAB_SIZE)
        altHistIdx = 0;

    sensors.altitude = altHigh * 10 / (ALT_TAB_SIZE / 2);
    
    

}

///////////////////////////////////////////////////////////////////////////////