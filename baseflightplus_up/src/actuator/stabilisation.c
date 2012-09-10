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
#include "core/command.h"

///////////////////////////////////////////////////////////////////////////////

#define RATE_SCALING     0.005  // Stick to rate scaling (5 radians/sec)/(500 RX PWM Steps) = 0.005
#define ATTITUDE_SCALING 0.002  // Stick to att scaling (1 radian)/(500 RX PWM Steps) = 0.001

float axisPID[4];

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
        desired[ROLL]  = standardRadianFormat(applyPID(&pids[ROLL_LEVEL_PID], 
                            standardRadianFormat(desired[ROLL] - sensors.attitude[ROLL] - cfg.angleTrim[ROLL]), 
                            dT));
        desired[PITCH] = standardRadianFormat(applyPID(&pids[PITCH_LEVEL_PID], 
                            standardRadianFormat(desired[PITCH] - sensors.attitude[PITCH] - cfg.angleTrim[PITCH]), 
                            dT));
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
    
    // And Throttle
    if(mode.ALTITUDE_MODE) {
        axisPID[THROTTLE]   = applyPID(&pids[ALTITUDE_PID], (altitudeHold - sensors.altitude) / 10.0f, dT); // 0.1m
        axisPID[THROTTLE]   = constrain(axisPID[THROTTLE], -200.0f, 200.0f);
    } else {
        axisPID[THROTTLE]   = 0.0f;
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