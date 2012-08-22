/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define PI          3.14159265358979323846f
#define TWO_PI      6.28318530717958647693f
#define ACCEL_1G    9.80665f
#define RAD2DEG     (180.0f/ PI)
#define DEG2RAD     (PI /180.0f) 

///////////////////////////////////////////////////////////////////////////////

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

#define MINCOMMAND  1000
#define MIDCOMMAND  1500
#define MAXCOMMAND  2000

//#define THESIS

///////////////////////////////////////////////////////////////////////////////

extern uint32_t cycleTime;

///////////////////////////////////////////////////////////////////////////////