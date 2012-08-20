/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

#include "core/command.h"
#include "actuator/pid.h"

///////////////////////////////////////////////////////////////////////////////
// Config Typedefs
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    float p;
    float i;
    float d;
    float iLim;
} pidConfig;

typedef struct {
    uint8_t version;

    uint8_t usePPM;
    
    uint8_t failsafe;
    uint16_t failsafeOnDelay;
    uint16_t failsafeOffDelay;
    uint16_t failsafeThrottle;

    uint8_t rcMap[8];

    uint16_t escPwmRate;
    uint16_t servoPwmRate;
    
    uint16_t auxActivate[AUX_OPTIONS];

    uint8_t mixerConfiguration;

    uint16_t minCommand;
    uint16_t midCommand;
    uint16_t maxCommand;
    uint16_t minCheck;
    uint16_t maxCheck;
    uint16_t minThrottle;
    uint16_t maxThrottle;
    uint8_t motorStop;
    
    uint16_t deadBand[3];
    
    pidConfig pids[NUM_PIDS];

    uint16_t biLeftServoMin;
    uint16_t biLeftServoMid;
    uint16_t biLeftServoMax;

    uint16_t biRightServoMin;
    uint16_t biRightServoMid;
    uint16_t biRightServoMax;

    int8_t yawDirection;
    uint16_t triYawServoMin;
    uint16_t triYawServoMid;
    uint16_t triYawServoMax;

    uint16_t gimbalRollServoMin;
    uint16_t gimbalRollServoMid;
    uint16_t gimbalRollServoMax;
    float gimbalRollServoGain;

    uint16_t gimbalPitchServoMin;
    uint16_t gimbalPitchServoMid;
    uint16_t gimbalPitchServoMax;
    float gimbalPitchServoGain;

    int8_t rollDirectionLeft;
    int8_t rollDirectionRight;
    int8_t pitchDirectionLeft;
    int8_t pitchDirectionRight;
    
    uint16_t wingLeftMinimum;
    uint16_t wingLeftMaximum;
    uint16_t wingRightMinimum;
    uint16_t wingRightMaximum;
    
    uint8_t accelLPF;
    float accelLPF_Factor;
    uint8_t accelCalibrated;
    float accelBias[3];

    uint16_t gyroLPF;
    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    uint8_t magCalibrated;
    float magBias[3];

    // For Mahony AHRS
    
    uint8_t magDriftCompensation;
    float magDeclination;

    float twoKp;

    float twoKi;
    
    uint8_t battery;
    float batScale;
    float batMinCellVoltage;
    float batMaxCellVoltage;

} cfg_t;

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

extern cfg_t cfg;

extern const char rcChannelLetters[8];

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input);

void readEEPROM(void);

void writeParams(void);

void checkFirstTime(bool reset);

///////////////////////////////////////////////////////////////////////////////
