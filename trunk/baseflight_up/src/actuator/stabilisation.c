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
#include "core/filters.h"

#define RATE_SCALING     0.005  // Stick to rate scaling (5 radians/sec)/(500 RX PWM Steps) = 0.005
#define ATTITUDE_SCALING 0.002  // Stick to att scaling (1 radian)/(500 RX PWM Steps) = 0.001

float axisPID[4];

/*
 * First we get the scaled desired commands
 * Using the desired command we apply relevant higher level PIDs onto the each axis.
 * Rate PID is always applied, this is nothing novel
 * - Need a stable rate mode before you even think of stabilising attitude/altitude
 */

void stabilisation(void)
{
    static uint32_t now, last;
    static float gyroFiltered[3] = {0.0f, 0.0f, 0.0f};
    now = micros();
    float dT = (float)(now - last) * 1e-6f;
    last = now;
    
    float desired[3];
    
    // Apply outer PID loops
    
    // First look at Roll and Pitch
    if (mode.LEVEL_MODE) {
        desired[ROLL]    = command[ROLL] * ATTITUDE_SCALING;
        desired[PITCH]   = command[PITCH] * ATTITUDE_SCALING;
        desired[ROLL]  = standardRadianFormat(applyPID(&pids[ROLL_LEVEL_PID], 
                            standardRadianFormat(desired[ROLL] - stateData.roll - cfg.angleTrim[ROLL]), 
                            dT));
        desired[PITCH] = standardRadianFormat(applyPID(&pids[PITCH_LEVEL_PID], 
                            standardRadianFormat(desired[PITCH] - stateData.pitch - cfg.angleTrim[PITCH]), 
                            dT));
    } else if(mode.HEADFREE_MODE) {
        float radDiff       = stateData.heading - headfreeReference;
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
        desired[YAW] = standardRadianFormat(applyPID(&pids[HEADING_PID], standardRadianFormat(headingHold - stateData.heading), dT));
    } else { // Default to rates
        desired[YAW]    = command[YAW] * RATE_SCALING;
        headingHold     = stateData.heading; // Get our new heading
    }
    
    // And Throttle
    if(mode.ALTITUDE_MODE) {
        axisPID[THROTTLE]   = applyPID(&pids[ALTITUDE_PID], (altitudeHold - stateData.altitude) / 10.0f, dT); // 0.1m
        axisPID[THROTTLE]   = constrain(axisPID[THROTTLE], -200.0f, 200.0f);
    } else {
        axisPID[THROTTLE]   = 0.0f;
    }
    
    // Apply inner PID loops
    
    // Filter the gyros
    gyroFiltered[ROLL]  = filterSmooth(stateData.gyro[ROLL], gyroFiltered[ROLL], cfg.gyroSmoothFactor);
    gyroFiltered[PITCH] = filterSmooth(stateData.gyro[PITCH], gyroFiltered[PITCH], cfg.gyroSmoothFactor);
    gyroFiltered[YAW]   = filterSmooth(stateData.gyro[YAW], gyroFiltered[YAW], cfg.gyroSmoothFactor);
    
    // Rate PID - Always
    axisPID[ROLL]   = applyPID(&pids[ROLL_RATE_PID], desired[ROLL] - stateData.gyro[ROLL], dT);
    axisPID[PITCH]  = applyPID(&pids[PITCH_RATE_PID], desired[PITCH] - stateData.gyro[PITCH], dT);
    axisPID[YAW]    = applyPID(&pids[YAW_RATE_PID], desired[YAW] - stateData.gyro[YAW], dT);
}