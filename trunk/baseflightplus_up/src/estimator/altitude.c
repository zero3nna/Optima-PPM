/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */
 
#include "board.h"
#include "sensors/sensors.h"
#include "core/filters.h"

#define BARO_TAB_SIZE   40

///////////////////////////////////////////////////////////////////////////////

void updateAltitude(void)
{
    uint32_t index;
    static int16_t BaroHistTab[BARO_TAB_SIZE];
    static uint32_t BaroHistIdx;
    static int32_t BaroHigh = 0;

    BaroHistTab[BaroHistIdx] = sensors.baroAltitude / 10;
    BaroHigh += BaroHistTab[BaroHistIdx];
    index = (BaroHistIdx + (BARO_TAB_SIZE / 2)) % BARO_TAB_SIZE;
    BaroHigh -= BaroHistTab[index];
    BaroHistIdx++;
    if (BaroHistIdx >= BARO_TAB_SIZE)
        BaroHistIdx = 0;

    sensors.altitude = BaroHigh * 10 / (BARO_TAB_SIZE / 2);

}

///////////////////////////////////////////////////////////////////////////////