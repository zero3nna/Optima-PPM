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
    
    Adapted from AeroQuad
    http://code.google.com/p/aeroquad/source/browse/trunk/AeroQuad
 */

#include "core/filters.h"

// Low pass filter, kept as regular C function for speed

float filterSmooth(float currentData, float previousData, float smoothFactor) 
{
  if (smoothFactor != 1.0) //only apply time compensated filter if smoothFactor is applied
  {
    return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor)); 
  }
  return currentData; //if smoothFactor == 1.0, do not calculate, just bypass!
}

//  4th Order IIR Filter

// [b, a] = cheby2(4,60,12.5/100) cheby2(order, stopband_ripple, Wst)
// Wst is cutoff frequency from 0.0 to 1.0 (1.0 corresponds to half the sampling rate)

float fourthOrderFilter(float input, fourthOrderData_t *filterData, float *A, float *B)
{
    float output;

    output = B[0] * input + B[1] * filterData->input[1-1] + B[2] * filterData->input[2-1]
                    + B[3] * filterData->input[3-1] + B[4] * filterData->input[4-1] - A[1-1] * filterData->output[1-1]
                    - A[2-1] * filterData->output[2-1] - A[3-1] * filterData->output[3-1] - A[4-1] * filterData->output[4-1];

    filterData->input[4-1] = filterData->input[3-1];
    filterData->input[3-1] = filterData->input[2-1];
    filterData->input[2-1] = filterData->input[1-1];
    filterData->input[1-1] = input;

    filterData->output[4-1] = filterData->output[3-1];
    filterData->output[3-1] = filterData->output[2-1];
    filterData->output[2-1] = filterData->output[1-1];
    filterData->output[1-1] = output;

    return output;
}


