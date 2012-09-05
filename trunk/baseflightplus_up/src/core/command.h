/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

///////////////////////////////////////////////////////////////////////////////
// Command Definitions
///////////////////////////////////////////////////////////////////////////////

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
#define OPT_BEEPERON        10
#define OPT_LEDMAX          11 // we want maximum illumination
#define OPT_LANDING_LIGHTS  12 // enable landing lights at any altitude
#define OPT_HEADFREE_REF    13 // acquire heading for HEADFREE mode

#define AUX_OPTIONS         14

///////////////////////////////////////////////////////////////////////////////
// Command Typedefs
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t I2C_INIT_DONE; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
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
    uint8_t SMALL_ANGLES_25;
    uint8_t CALIBRATE_MAG;
    uint8_t FAILSAFE;
} modeFlags_t;

typedef uint16_t (* readRawRCFuncPtr)(uint8_t chan);

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

extern int16_t rcData[8];
extern int16_t failsafeCnt;

extern uint8_t auxOptions[AUX_OPTIONS];
extern modeFlags_t mode;

extern float command[4];
extern uint8_t commandInDetent[3];
extern uint8_t lastCommandInDetent[3];

extern float headfreeReference;;
extern float headingHold;
extern float altitudeThrottleHold;
extern float altitudeHold;

extern readRawRCFuncPtr readRawRC;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void updateCommands(void);

///////////////////////////////////////////////////////////////////////////////
