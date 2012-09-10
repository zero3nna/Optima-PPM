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

///////////////////////////////////////////////////////////////////////////////
// Filter Typedefs
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    float input[4];
    float output[4];
} fourthOrderData_t;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

float fourthOrderFilter(float input, fourthOrderData_t *filterData, float *A, float *B);

float filterSmooth(float currentData, float previousData, float smoothFactor);

///////////////////////////////////////////////////////////////////////////////