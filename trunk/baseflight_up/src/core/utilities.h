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

// Constrain

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define map(amt, in_min, in_max, out_min, out_max) (amt - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

char *itoa(int i, char *a, int r);
char *ftoa(float x, char *a);
unsigned dbl2stri(double dbl, char *outbfr, unsigned dec_digits);
float stringToFloat(const char *p);

float standardRadianFormat(float angle);

void Quaternion2RPY(const float q[4], float *roll, float *pitch, float *yaw);

