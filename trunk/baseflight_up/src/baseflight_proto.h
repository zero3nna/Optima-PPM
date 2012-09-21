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

#define PI          3.14159265358979323846f
#define TWO_PI      6.28318530717958647693f
#define ACCEL_1G    9.80665f
#define RAD2DEG     (180.0f/ PI)
#define DEG2RAD     (PI /180.0f) 

#define ROLL            0
#define PITCH           1
#define YAW             2
#define THROTTLE        3
#define AUX1            4
#define AUX2            5
#define AUX3            6
#define AUX4            7
#define AUX_CHANNELS    4

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

#define X       0
#define Y       1
#define Z       2

#define MINCOMMAND  1000
#define MIDCOMMAND  1500
#define MAXCOMMAND  2000

//#define THESIS

//#define SONAR

//#define DEBUG

extern uint32_t cycleTime;
extern uint16_t debug[4];
