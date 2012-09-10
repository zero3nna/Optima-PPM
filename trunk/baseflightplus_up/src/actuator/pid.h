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
// PID Definitions
///////////////////////////////////////////////////////////////////////////////

#define ROLL_RATE_PID       0
#define PITCH_RATE_PID      1
#define YAW_RATE_PID        2

#define ROLL_LEVEL_PID      3
#define PITCH_LEVEL_PID     4
#define HEADING_PID         5

#define ALTITUDE_PID        6

#define NUM_PIDS            7

///////////////////////////////////////////////////////////////////////////////
// PID Types
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    float p;
    float i;
    float d;
    float iLim;
    float iAccum;
    float lastErr;
    float dTerm1, dTerm2;
} pidData;

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

extern pidData pids[NUM_PIDS];

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void initPIDs(void);

void zeroPIDs(void);

void zeroPID(pidData *pid);

float applyPID(pidData *pid, const float err, float dT);

///////////////////////////////////////////////////////////////////////////////



