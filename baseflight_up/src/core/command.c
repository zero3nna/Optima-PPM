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

#include "actuator/mixer.h"

#include "core/command.h"

#define THROTTLE_HOLD_DEADBAND  20

readRawRCFuncPtr readRawRC;

int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
int16_t failsafeCnt;

uint8_t auxOptions[AUX_OPTIONS];
modeFlags_t mode;

int16_t command[4] = {0.0f, 0.0f, 0.0f, 1000.0f};
uint8_t commandInDetent[3] = {true, true, true};
uint8_t lastCommandInDetent[3] = {true, true, true};

float headfreeReference;
float headingHold;
float altitudeThrottleHold;
float altitudeHold;

// From Multiwii
static void computeRC(void)
{
    static int16_t rcData4Values[8][4], rcDataMean[8];
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
        rcData4Values[chan][rc4ValuesIndex % 4] = readRawRC(chan);
        rcDataMean[chan] = 0;
        for (a = 0; a < 4; a++)
            rcDataMean[chan] += rcData4Values[chan][a];

        rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
        if (rcDataMean[chan] < rcData[chan] - 3)
            rcData[chan] = rcDataMean[chan] + 2;
        if (rcDataMean[chan] > rcData[chan] + 3)
            rcData[chan] = rcDataMean[chan] - 2;
    }
}

void updateCommands(void)
{
    uint8_t i;
    static uint8_t commandDelay;

    if(!featureGet(FEATURE_SPEKTRUM) || spektrumFrameComplete())
        computeRC();
    
    // Ground Routines
    if(rcData[THROTTLE] < cfg.minCheck) {
        zeroPIDs(); // Stops integrators from exploding on the ground
        
        if(cfg.auxActivate[OPT_ARM] > 0) {
            if(auxOptions[OPT_ARM] && mode.OK_TO_ARM) { // AUX Arming
                mode.ARMED = 1;
                headfreeReference = stateData.heading;
            } else if(mode.ARMED){ // AUX Disarming
                mode.ARMED = 0;
            }
        } else if(rcData[YAW] > cfg.maxCheck && !mode.ARMED) { // Stick Arming
            if(commandDelay++ == 20) {
                mode.ARMED = 1;
                headfreeReference = stateData.heading;
            }
        } else if(rcData[YAW] < cfg.minCheck && mode.ARMED) { // Stick Disarming
            if(commandDelay++ == 20) {
                mode.ARMED = 0;
            }
        } else if(rcData[YAW] < cfg.minCheck && rcData[PITCH] > cfg.minCheck && !mode.ARMED) {
            if(commandDelay++ == 20) {
                computeGyroRTBias();
        		//pulseMotors(3);
        		// GPS Reset
            }
        } else {
            commandDelay = 0;
        }
    } else if(rcData[THROTTLE] > cfg.maxCheck && !mode.ARMED) {
        if(rcData[YAW] > cfg.maxCheck && rcData[PITCH] < cfg.minCheck) {
            if(commandDelay++ == 20) {
                magCalibration();
            }
        } else if(rcData[YAW] < cfg.minCheck && rcData[PITCH] < cfg.minCheck) {
            if(commandDelay++ == 20) {
                accelCalibration();
                pulseMotors(3);
            }
        } else if (rcData[PITCH] > cfg.maxCheck) {
                cfg.angleTrim[PITCH] += 0.01;
                writeParams();
        } else if (rcData[PITCH] < cfg.minCheck) {
                cfg.angleTrim[PITCH] -= 0.01;
                writeParams();
        } else if (rcData[ROLL] > cfg.maxCheck) {
                cfg.angleTrim[ROLL] += 0.01;
                writeParams();
        } else if (rcData[ROLL] < cfg.minCheck) {
                cfg.angleTrim[ROLL] -= 0.01;
                writeParams();
        } else {
            commandDelay = 0;
        }
    }
    
    // Failsafe
    if(featureGet(FEATURE_FAILSAFE)) {
        if(failsafeCnt > cfg.failsafeOnDelay && mode.ARMED) {
            // Stabilise and set Throttle to failsafe level
            for(i = 0; i < 3; ++i) {
                rcData[i] = cfg.midCommand;
            }
            rcData[THROTTLE] = cfg.failsafeThrottle;
            mode.FAILSAFE = 1;
            if(failsafeCnt > cfg.failsafeOffDelay + cfg.failsafeOnDelay) {
                // Disarm
                mode.ARMED = 0;
                // you will have to switch off first to rearm
                mode.OK_TO_ARM = 0;  
            }
            if(failsafeCnt > cfg.failsafeOnDelay && !mode.ARMED) {
                mode.ARMED = 0;
                // you will have to switch off first to rearm
                mode.OK_TO_ARM = 0;
            }
            ++failsafeCnt;
        } else {
            mode.FAILSAFE = 0;
        }
    }

    // Aux Options
    uint16_t auxOptionMask = 0;
    
    for(i = 0; i < AUX_CHANNELS; ++i) {
        auxOptionMask |= (rcData[AUX1 + i] < cfg.minCheck) << (3 * i) |
                        (rcData[AUX1 + i] > cfg.minCheck && rcData[i] < cfg.minCheck) << (3 * i + 1) |
                        (rcData[AUX1 + i] > cfg.maxCheck) << (3 * i + 2);
    }
    
    for(i = 0; i < AUX_OPTIONS; ++i) {
        auxOptions[i] = (auxOptionMask & cfg.auxActivate[i]) > 0;
    }
    
    if(auxOptions[OPT_ARM] == 0) {
        mode.OK_TO_ARM = 1;
    }
    
    // Passthrough
    if(auxOptions[OPT_PASSTHRU]) {
        mode.PASSTHRU_MODE = 1;
    } else {
        mode.PASSTHRU_MODE = 0;
    }
    
    // Level - TODO Add failsafe and ACC check
    if(auxOptions[OPT_LEVEL] || mode.FAILSAFE) { // 
        if(!mode.LEVEL_MODE) {
            zeroPID(&pids[ROLL_LEVEL_PID]);
            zeroPID(&pids[PITCH_LEVEL_PID]);
            mode.LEVEL_MODE = 1;
        }
    } else {
        mode.LEVEL_MODE = 0;
    }
    
    // Heading 
    if(auxOptions[OPT_HEADING]) {
        if(!mode.HEADING_MODE) {
            mode.HEADING_MODE = 1;
            headingHold = stateData.heading;
        }
    } else {
        mode.HEADING_MODE = 0;
    }
    
    // Headfree
    if(auxOptions[OPT_HEADFREE]) {
        if(!mode.HEADFREE_MODE) {
            mode.HEADFREE_MODE = 1;
            headfreeReference = stateData.heading;
        }
    } else {
        mode.HEADFREE_MODE = 0;
    }
    
    if(auxOptions[OPT_HEADFREE_REF]) {
        headfreeReference = stateData.heading;
    }
    
    // GPS GOES HERE
    
    uint8_t axis;
    uint16_t tmp, tmp2;
    
    // Apply deadbands, rates and expo    
    for (axis = 0; axis < 3; axis++) {
        lastCommandInDetent[axis] = commandInDetent[axis];
        tmp = min(abs(rcData[axis] - cfg.midCommand), 500);
        
        if (tmp > cfg.deadBand[axis]) {
            tmp -= cfg.deadBand[axis];
            commandInDetent[axis] = false;
        } else {
            tmp = 0;
            commandInDetent[axis] = true;
        }
    
        if(axis != 2) { // Roll and Pitch
            tmp2 = tmp / 100;
            command[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
        } else { // Yaw
            command[axis] = tmp;
        }
        
        if (rcData[axis] < cfg.midCommand)
            command[axis] = -command[axis];
    }

    tmp = constrain(rcData[THROTTLE], cfg.minCheck, 2000);
    tmp = (uint32_t) (tmp - cfg.minCheck) * 1000 / (2000 - cfg.minCheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    command[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    // This will force a reset
    if(fabs(command[THROTTLE] - altitudeThrottleHold) > THROTTLE_HOLD_DEADBAND)
        mode.ALTITUDE_MODE = 0;
    
    // Altitude TODO Add barometer check
    if(auxOptions[OPT_ALTITUDE]) {
        if(!mode.ALTITUDE_MODE) {
            mode.ALTITUDE_MODE = 1;
            altitudeThrottleHold = command[THROTTLE];
            altitudeHold = stateData.altitude;
            zeroPID(&pids[ALTITUDE_PID]);
        }
    } else {
        mode.ALTITUDE_MODE = 0;
    }
}




