/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"
#include "core/filters.h"
#include "core/printf_min.h"

///////////////////////////////////

baro_t _baro;
baro_t *baro = &_baro;

void baroUpdate(void)
{
    static uint8_t state = 0;
    int32_t pressure;

    switch (state) {
        case 0:
            baro->start_ut();
            state++;
            singleEvent(baroUpdate, baro->ut_delay);
            break;
        case 1:
            baro->get_ut();
            state++;
            singleEvent(baroUpdate, 500);
            break;
        case 2:
            baro->start_up();
            state++;
            singleEvent(baroUpdate, baro->up_delay);
            break;
        case 3:
            baro->get_up();
            pressure = baro->calculate();
            sensors.baroAltitude = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f; // centimeter
            state = 0;
            singleEvent(baroUpdate, baro->repeat_delay);
            break;
    }
}