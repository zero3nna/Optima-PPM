/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

///////////////////////////////////////////////////////////////////////////////
// Constrain
///////////////////////////////////////////////////////////////////////////////

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define map(amt, in_min, in_max, out_min, out_max) (amt - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

///////////////////////////////////////////////////////////////////////////////
// ITOA
///////////////////////////////////////////////////////////////////////////////

char *itoa(int i, char *a, int r);

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////

char *ftoa(float x, char *a);
unsigned dbl2stri(double dbl, char *outbfr, unsigned dec_digits);

///////////////////////////////////////////////////////////////////////////////
//  Standard Radian Format Limiter
////////////////////////////////////////////////////////////////////////////////

float standardRadianFormat(float angle);

void Quaternion2RPY(const float q[4], float rpy[3]);

// ****** find quaternion from roll, pitch, yaw ********
void RPY2Quaternion(const float rpy[3], float q[4]);

float stringToFloat(const char *p);