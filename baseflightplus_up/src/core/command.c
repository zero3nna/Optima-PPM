/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

int16_t rcData[8];

uint8_t auxOptions[AUX_OPTIONS];
modeFlags_t mode;

float command[4] = {0.0f, 0.0f, 0.0f, 1000.0f};
uint8_t commandInDetent[4] = {true, true, true, true};
uint8_t lastCommandInDetent[4] = {true, true, true, true};

float headfreeReference;
float headingHold;

///////////////////////////////////////////////////////////////////////////////
// Read Flight Commands
///////////////////////////////////////////////////////////////////////////////


// From Multiwii
uint16_t readRawRC(uint8_t chan)
{
    uint16_t data;

    data = pwmRead(cfg.rcMap[chan]);
    if (data < 750 || data > 2250)
        data = cfg.midCommand;

    return data;
}

// From Multiwii
void computeRC(void)
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

    computeRC();
    
    // TODO Add spektrum support
    // TODO fix multiwii serial protocol RC injection
    // Can choose to inject RCData here if needed
    
    ///////////////////////////////////////////////////////////////////////////////
    // Ground Routines
    ///////////////////////////////////////////////////////////////////////////////
    
    if(rcData[THROTTLE] < cfg.minCheck) {
        zeroPIDs(); // Stops integrators from exploding on the ground
        
        if(cfg.auxActivate[OPT_ARM] > 0) {
            if(auxOptions[OPT_ARM] && mode.OK_TO_ARM) { // AUX Arming
                mode.ARMED = 1;
                headfreeReference = sensors.attitude[YAW];
            } else if(mode.ARMED){ // AUX Disarming
                mode.ARMED = 0;
            }
        } else if(rcData[YAW] > cfg.maxCheck && !mode.ARMED) { // Stick Arming
            if(commandDelay++ == 20) {
                mode.ARMED = 1;
                headfreeReference = sensors.attitude[YAW];
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
        }else if(rcData[YAW] < cfg.minCheck && rcData[PITCH] < cfg.minCheck) {
            if(commandDelay++ == 20) {
                accelCalibration();
                pulseMotors(3);
            }
        } else {
            commandDelay = 0;
        }
        // Add angle trimming?
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // Failsafe
    ///////////////////////////////////////////////////////////////////////////////
    
    if(cfg.failsafe) {
        if(failsafeCount > cfg.failsafeOnDelay && mode.ARMED) {
            // Stabilise and set Throttle to failsafe level
            for(i = 0; i < 3; ++i) {
                rcData[i] = cfg.midCommand;
            }
            rcData[THROTTLE] = cfg.failsafeThrottle;
            mode.FAILSAFE = 1;
            if(failsafeCount > cfg.failsafeOffDelay + cfg.failsafeOnDelay) {
                // Disarm
                mode.ARMED = 0;
                // you will have to switch off first to rearm
                mode.OK_TO_ARM = 0;  
            }
            if(failsafeCount > cfg.failsafeOnDelay && !mode.ARMED) {
                mode.ARMED = 0;
                // you will have to switch off first to rearm
                mode.OK_TO_ARM = 0;
            }
            ++failsafeCount;
        } else {
            mode.FAILSAFE = 0;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Aux Options
    ///////////////////////////////////////////////////////////////////////////////
    	
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
    
    // Altitude TODO Add barometer check
    if(auxOptions[OPT_ALTITUDE]) {
        if(!mode.ALTITUDE_MODE) {
            mode.ALTITUDE_MODE = 1;
            //TODO
            //AltHold = EstAlt;
            //initialThrottleHold = rcCommand[THROTTLE];
            //errorAltitudeI = 0;
            //BaroPID = 0;
        }
    } else {
        mode.ALTITUDE_MODE = 0;
    }
    
    // Heading 
    if(auxOptions[OPT_HEADING]) {
        if(!mode.HEADING_MODE) {
            mode.HEADING_MODE = 1;
            headingHold = sensors.attitude[YAW];
        }
    } else {
        mode.HEADING_MODE = 0;
    }
    
    // Headfree
    if(auxOptions[OPT_HEADFREE]) {
        if(!mode.HEADFREE_MODE) {
            mode.HEADFREE_MODE = 1;
            headfreeReference = sensors.attitude[YAW];
        }
    } else {
        mode.HEADFREE_MODE = 0;
    }
    
    if(auxOptions[OPT_HEADFREE_REF]) {
        headfreeReference = sensors.attitude[YAW];
    }
    
    /////////// ENDTODO
    
    // GPS GOES HERE
    
    ///////////////////////////////////////////////////////////////////////////////
    
    // Scale receiver commands - apply deadband
    for (i = 0; i < 4; ++i) {
        command[i] = (float)rcData[i];
    }

    command[ROLL]       -= cfg.midCommand;                 // Roll Range    -500:500
    command[PITCH]      -= cfg.midCommand;                 // Pitch Range   -500:500
    command[YAW]        -= cfg.midCommand;                 // Yaw Range     -500:500
    command[THROTTLE]   -= cfg.midCommand - MIDCOMMAND;    // Throttle Range 1000:2000
    
    // Apply deadbands
    for(i = 0; i < 3; ++i) {
        
        lastCommandInDetent[i] = commandInDetent[i];
    
    	if ((command[i] <= cfg.deadBand[i]) && (command[i] >= -cfg.deadBand[i])) {
            command[i] = 0;
            commandInDetent[i] = true;
  	    } else {
            commandInDetent[i] = false;
  	        if (command[i] > 0) {
  		        command[i] = command[i] - cfg.deadBand[i];
  	        } else {
  	            command[i] = command[i] + cfg.deadBand[i];
  	        }
        }
        
        // TODO - Do we need dynamic PID?
    }

}

///////////////////////////////////////////////////////////////////////////////




