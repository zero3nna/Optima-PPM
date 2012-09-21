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
#include "actuator/pid.h"

pidData pids[NUM_PIDS];

// Load PIDs from cfg
void initPIDs(void)
{
    uint8_t i;
    for(i = 0; i < NUM_PIDS; ++i) {
        pids[i].p = cfg.pids[i].p;
        pids[i].i = cfg.pids[i].i;
        pids[i].d = cfg.pids[i].d;
        pids[i].iLim = cfg.pids[i].iLim;
    }
    zeroPIDs();
}

// Reset integrators and error for all PIDs
void zeroPIDs(void)
{
    uint8_t i;
    for (i = 0; i < NUM_PIDS; ++i) {
        pids[i].iAccum = 0.0f;
        pids[i].lastErr = 0.0f;
        pids[i].lastDer = 0.0f;
    }
}

// Reset integrators and error for a specified PID data structure
void zeroPID(pidData *pid)
{
    pid->iAccum = 0.0f;
    pid->lastErr = 0.0f;
    pids->lastDer = 0.0f;
}

#define F_CUT   20.0f
#define RC      1.0f / (TWO_PI * F_CUT)

// Pretty much the Openpilot PID code
float applyPID(pidData *pid, const float err, float dT)
{
    float diff = (err - pid->lastErr);
    float dTerm = 0.0f;
    pid->lastErr = err;
    
    if(pid->i != 0.0f) {
        // Scale by 1000 during calculation to avoid loss of precision
        pid->iAccum += err * (pid->i * dT * 1000.0f);
        // Stop the integrators from winding up
        pid->iAccum = constrain(pid->iAccum, -pid->iLim * 1000.0f, pid->iLim * 1000.0f);
    }
    
    // Calculate DT1 term, fixed T1 timeconstant
    if(pid->d != 0.0f) {
        // Discrete low pass filter, cuts out the
        // high frequency noise that can drive controller crazy
        dTerm = pid->lastDer + (( dT / ( dT + 7.9577e-3)) * ((diff * pid->d / dT) - pid->lastDer));
        pid->lastDer = dTerm;
    }
    
    return ((pid->p * err) + pid->iAccum / 1000.0f + dTerm);
}


