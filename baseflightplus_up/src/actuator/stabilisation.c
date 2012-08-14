/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

#define RATE_SCALING     0.005  // Stick to rate scaling (5 radians/sec)/(500 RX PWM Steps) = 0.005
#define ATTITUDE_SCALING 0.001  // Stick to att scaling (1 radian)/(500 RX PWM Steps) = 0.001

float axisPID[3];

///////////////////////////////////////////////////////////////////////////////
// Stabilisation
///////////////////////////////////////////////////////////////////////////////
/*
 * First we get the scaled desired commands
 * Using the desired command we apply relevant higher level PIDs onto the each axis.
 * Rate PID is always applied, this is nothing novel
 * - Need a stable rate mode before you even think of stabilising attitude/altitude
 */

void stabilisation(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    static uint32_t now, last;
    now = micros();
    float dT = (float)(now - last) * 1e-6f;
    last = now;
    ///////////////////////////////////////////////////////////////////////////////
    
    float desired[3];
    
    ///////////////////////////////////////////////////////////////////////////////
    // Apply outer PID loops
    ///////////////////////////////////////////////////////////////////////////////
    
    // First look at Roll and Pitch
    if (mode.LEVEL_MODE) {
        desired[ROLL]    = command[ROLL] * ATTITUDE_SCALING;
        desired[PITCH]   = command[PITCH] * ATTITUDE_SCALING;
        desired[ROLL]  = standardRadianFormat(applyPID(&pids[ROLL_LEVEL_PID], standardRadianFormat(desired[ROLL] - sensors.attitude[ROLL]), dT));
        desired[PITCH] = standardRadianFormat(applyPID(&pids[PITCH_LEVEL_PID], standardRadianFormat(desired[PITCH] - sensors.attitude[PITCH]), dT));
    } else if(mode.HEADFREE_MODE) {
        float radDiff       = sensors.attitude[YAW] - headfreeReference;
        float cosDiff       = cosf(radDiff);
        float sinDiff       = sinf(radDiff);
        desired[ROLL]       = command[ROLL] * cosDiff - command[PITCH] * sinDiff;
        desired[PITCH]      = command[PITCH] * cosDiff + command[ROLL] * sinDiff;
    } else { // Default to rates
        desired[ROLL]   = command[ROLL] * RATE_SCALING;
        desired[PITCH]  = command[PITCH] * RATE_SCALING;
    }
    
    // Now Yaw
    if(mode.HEADING_MODE && commandInDetent[YAW]) {
        if(!lastCommandInDetent[YAW]) {
            zeroPID(&pids[HEADING_PID]); // We have a new heading zero integrators
        }
        desired[YAW] = standardRadianFormat(applyPID(&pids[HEADING_PID], standardRadianFormat(headingHold - sensors.attitude[YAW]), dT));
    } else { // Default to rates
        desired[YAW]    = command[YAW] * RATE_SCALING;
        headingHold     = sensors.attitude[YAW]; // Get our new heading
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // Apply inner PID loops
    ///////////////////////////////////////////////////////////////////////////////
    
    // Rate PID - Always
    axisPID[ROLL]   = applyPID(&pids[ROLL_RATE_PID], desired[ROLL] - sensors.gyro[ROLL], dT);
    axisPID[PITCH]  = applyPID(&pids[PITCH_RATE_PID], desired[PITCH] - sensors.gyro[PITCH], dT);
    axisPID[YAW]    = applyPID(&pids[YAW_RATE_PID], desired[YAW] - sensors.gyro[YAW], dT);
}

///////////////////////////////////////////////////////////////////////////////