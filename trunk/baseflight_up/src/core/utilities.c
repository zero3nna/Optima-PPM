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

#ifndef HAVE_ITOA_FUNCTION
/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0) {
        *a = '-';
        *i2a(-(unsigned) i, a + 1, r) = 0;
    } else
        *i2a(i, a, r) = 0;

    return a;
}

#endif

#ifndef HAVE_FTOA_FUNCTION

char *ftoa(float x, char *floatString)
{
    dbl2stri(x, floatString, 6);

    return floatString;
}

#endif

static const double round_nums[8] = {
   0.5,
   0.05,
   0.005,
   0.0005,
   0.00005,
   0.000005,
   0.0000005,
   0.00000005
} ;

unsigned dbl2stri(double dbl, char *outbfr, unsigned dec_digits)
{
    static char local_bfr[32] ;
    char *output = (outbfr == 0) ? local_bfr : outbfr ;

    //*******************************************
    //  extract negative info
    //*******************************************
    if (dbl < 0.0) {
        *output++ = '-' ;
        dbl *= -1.0 ;
    }

    //  handling rounding by adding .5LSB to the floating-point data
    if (dec_digits < 8) {
        if(dbl > 0)
            dbl += round_nums[dec_digits];
        else
            dbl -= round_nums[dec_digits];
    }

    //**************************************************************************
    //  construct fractional multiplier for specified number of digits.
    //**************************************************************************
    uint mult = 1 ;
    uint idx ;
    for (idx=0; idx < dec_digits; idx++)
        mult *= 10 ;

    // printf("mult=%u\n", mult) ;
    uint wholeNum = (uint) dbl ;
    uint decimalNum = (uint) ((dbl - wholeNum) * mult);

    //*******************************************
    //  convert integer portion
    //*******************************************
    char tbfr[12] ;
    idx = 0 ;
    while (wholeNum != 0) {
        tbfr[idx++] = '0' + (wholeNum % 10) ;
        wholeNum /= 10 ;
    }
    if (idx == 0) {
        *output++ = '0' ;
    } else {
        while (idx > 0) {
            *output++ = tbfr[idx-1] ;  //lint !e771
            idx-- ;
        }
    }
    if (dec_digits > 0) {
        *output++ = '.' ;

    //*******************************************
    //  convert fractional portion
    //*******************************************
        idx = 0 ;
        while (decimalNum != 0) {
            tbfr[idx++] = '0' + (decimalNum % 10) ;
            decimalNum /= 10 ;
        }
    //  pad the decimal portion with 0s as necessary;
    //  We wouldn't want to report 3.093 as 3.93, would we??
        while (idx < dec_digits) {
            tbfr[idx++] = '0' ;
        }

        if (idx == 0) {
            *output++ = '0' ;
        } else {
            while (idx > 0) {
                *output++ = tbfr[idx-1] ;
                idx-- ;
            }
        }
    }
    *output = 0 ;

    //  prepare output
    output = (outbfr == 0) ? local_bfr : outbfr ;
    return 0 ;
}

float standardRadianFormat(float angle)
{
    while(angle > PI)
        angle -= TWO_PI;
        
    return (angle);
}

// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//

#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')

float stringToFloat(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.

    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.

    sign = 1.0;
    if (*p == '-') {
        sign = -1.0;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.

    value = 0.0;
    while (valid_digit(*p)) {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.

    if (*p == '.') {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.

    scale = 1.0;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.

        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.

        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.

        while (expon >= 50) { scale *= 1E50; expon -= 50; }
        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
        while (expon >   0) { scale *= 10.0; expon -=  1; }
    }

    // Return signed and scaled floating point result.

    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
//  Quaternion Conversion from Openpilot
////////////////////////////////////////////////////////////////////////////////

// ****** find roll, pitch, yaw from quaternion ********
void Quaternion2RPY(const float q[4], float *roll, float *pitch, float *yaw)
{	
	float R13, R11, R12, R23, R33;
	float q0s = q[0] * q[0];
	float q1s = q[1] * q[1];
	float q2s = q[2] * q[2];
	float q3s = q[3] * q[3];

	R13 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	R11 = q0s + q1s - q2s - q3s;
	R12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
	R23 = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	R33 = q0s - q1s - q2s + q3s;

	*pitch = -asinf(-R13);	// pitch always between -pi/2 to pi/2
	*yaw = atan2f(R12, R11);
	*roll = atan2f(R23, R33);
}