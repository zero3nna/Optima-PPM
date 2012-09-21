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

void baroUpdate(void)
{
    static uint8_t state = 0;
    int32_t pressure;

    switch (state) {
        case 0:
            baro.start_ut();
            state++;
            singleEvent(baroUpdate, baro.ut_delay);
            break;
        case 1:
            baro.get_ut();
            state++;
            singleEvent(baroUpdate, 500);
            break;
        case 2:
            baro.start_up();
            state++;
            singleEvent(baroUpdate, baro.up_delay);
            break;
        case 3:
            baro.get_up();
            pressure = baro.calculate();
            sensorData.baroAltitude = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f; // centimeter
            state = 0;
            singleEvent(baroUpdate, baro.repeat_delay);
            break;
    }
}