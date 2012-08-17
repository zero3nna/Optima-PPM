/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"
#include "actuator/pid.h"

///////////////////////////////////////////////////////////////////////////////

pidData pids[NUM_PIDS];

///////////////////////////////////////////////////////////////////////////////
// initPIDs
//
// Load PIDs from cfg
///////////////////////////////////////////////////////////////////////////////

void initPIDs(void)
{
    uint8_t i;
    for(i = 0; i < NUM_PIDS; ++i) {
        pids[i].p = cfg.pids[i].p;
        pids[i].i = cfg.pids[i].i;
        pids[i].d = cfg.pids[i].d;
        pids[i].iLim = cfg.pids[i].iLim;
    }
}

///////////////////////////////////////////////////////////////////////////////
// zeroPIDs
//
// Reset integrators and error for all PIDs
///////////////////////////////////////////////////////////////////////////////

void zeroPIDs(void)
{
    uint8_t i;
    for (i = 0; i < NUM_PIDS; ++i) {
        pids[i].iAccum = 0.0f;
        pids[i].lastErr = 0.0f;
    }
}

///////////////////////////////////////////////////////////////////////////////
// zeroPID
//
// Reset integrators and error for a specified PID data structure
///////////////////////////////////////////////////////////////////////////////

void zeroPID(pidData *pid)
{
    pid->iAccum = 0.0f;
    pid->lastErr = 0.0f;
}

/* Pretty much the Openpilot PID code */
float applyPID(pidData *pid, const float err, float dT)
{
    float diff = (err - pid->lastErr);
    pid->lastErr = err;
    
    // Scale by 1000 during calculation to avoid loss of precision
    pid->iAccum += err * (pid->i * dT * 1000.0f);
    // Stop the integrators from winding up
    pid->iAccum = constrain(pid->iAccum, -pid->iLim * 1000.0f, pid->iLim * 1000.0f);
    
    return ((err * pid->p) + pid->iAccum / 1000.0f + (diff * pid->d / dT));
}


