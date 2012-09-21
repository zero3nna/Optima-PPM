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

// Command Definitions

#define OPT_LEVEL           0
#define OPT_ALTITUDE        1
#define OPT_HEADING         2
#define OPT_CAMSTAB         3
#define OPT_CAMTRIG         4
#define OPT_ARM             5
#define OPT_GPSHOME         6
#define OPT_GPSHOLD         7
#define OPT_PASSTHRU        8
#define OPT_HEADFREE        9
#define OPT_HEADFREE_REF    10 // acquire heading for HEADFREE mode

#define AUX_OPTIONS         11

// Command Typedefs

typedef struct {
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ACC_CALIBRATED;
    uint8_t LEVEL_MODE;
    uint8_t HEADING_MODE;
    uint8_t ALTITUDE_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t FAILSAFE;
} modeFlags_t;

typedef uint16_t (* readRawRCFuncPtr)(uint8_t chan);

// External Variables

extern int16_t rcData[8];
extern int16_t failsafeCnt;

extern uint8_t auxOptions[AUX_OPTIONS];
extern modeFlags_t mode;

extern int16_t command[4];
extern uint8_t commandInDetent[3];
extern uint8_t lastCommandInDetent[3];

extern float headfreeReference;;
extern float headingHold;
extern float altitudeThrottleHold;
extern float altitudeHold;

extern readRawRCFuncPtr readRawRC;

// Functions

void updateCommands(void);