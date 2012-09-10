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
 
#pragma once

#include "sensors/sensors.h"

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

extern int16_t rawGyro[3];

extern int16_t rawGyroTemperature;

extern gyro_t *gyro;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void readGyro(void);

void readGyroTemp(void);

void computeGyroTCBias(void);

void computeGyroRTBias(void);

void gyroTempCalibration(void);

///////////////////////////////////////////////////////////////////////////////