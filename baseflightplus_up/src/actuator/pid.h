/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

#define ROLL_RATE_PID       0
#define PITCH_RATE_PID      1
#define YAW_RATE_PID        2

#define ROLL_LEVEL_PID      3
#define PITCH_LEVEL_PID     4
#define HEADING_PID         5

#define NUM_PIDS            6

///////////////////////////////////////////////////////////////////////////////

typedef struct {
    float p;
    float i;
    float d;
    float iLim;
    float iAccum;
    float lastErr;
} pidData;

extern pidData pids[NUM_PIDS];

///////////////////////////////////////////////////////////////////////////////

void initPIDs(void);

void zeroPIDs(void);

void zeroPID(pidData *pid);

float applyPID(pidData *pid, const float err, float dT);



