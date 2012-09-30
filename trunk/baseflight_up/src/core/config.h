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

#include "actuator/mixer.h"
#include "actuator/pid.h"

#include "core/command.h"

// Config Typedefs

typedef struct {
    float p;
    float i;
    float d;
    float iLim;
} pidConfig;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_MOTOR_STOP = 1 << 2,
    FEATURE_SERVO_TILT = 1 << 3,
    FEATURE_FAILSAFE = 1 << 4,
    FEATURE_SONAR = 1 << 5,
    FEATURE_SPEKTRUM = 1 << 6,
} AvailableFeatures;

typedef enum {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_TILTONLY = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX = 1 << 3,
} GimbalFlags;

typedef struct {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;       // magic number, should be 0xBE
    
    uint32_t enabledFeatures;
    
    uint16_t failsafeOnDelay;
    uint16_t failsafeOffDelay;
    uint16_t failsafeThrottle;
    
    uint8_t commandRate;
    uint8_t commandExpo;
    uint8_t throttleMid;
    uint8_t throttleExpo;
    uint8_t rollPitchRate;
    uint8_t yawRate;

    //uint8_t dynThrPID;

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
    uint8_t spektrumHiRes;
    
    uint16_t deadBand[3];
    
    int16_t servotrim[8];                   // Adjust Servo MID Offset & Swash angles
    int8_t servoreverse[8];
    
    int8_t yawDirection;
    uint16_t triYawServoMin;
    uint16_t triYawServoMid;
    uint16_t triYawServoMax;

    uint16_t biLeftServoMin;
    uint16_t biLeftServoMid;
    uint16_t biLeftServoMax;

    uint16_t biRightServoMin;
    uint16_t biRightServoMid;
    uint16_t biRightServoMax;
    
    uint16_t wingLeftMin;
    uint16_t wingLeftMid;
    uint16_t wingLeftMax;
    uint16_t wingRightMin;
    uint16_t wingRightMid;
    uint16_t wingRightMax;
    int8_t pitchDirectionLeft;
    int8_t pitchDirectionRight;
    int8_t rollDirectionLeft;
    int8_t rollDirectionRight;
    
    uint8_t gimbalFlags;
    float gimbalSmoothFactor;
    
    uint16_t gimbalRollServoMin;
    uint16_t gimbalRollServoMid;
    uint16_t gimbalRollServoMax;
    float gimbalRollServoGain;

    uint16_t gimbalPitchServoMin;
    uint16_t gimbalPitchServoMid;
    uint16_t gimbalPitchServoMax;
    float gimbalPitchServoGain;
    
    pidConfig pids[NUM_PIDS];
    
    float angleTrim[2];
    
    uint8_t accelLPF;
    //float accelCutout;
    float accelSmoothFactor;
    uint8_t accelCalibrated;
    int32_t accelBias[3];

    uint8_t gyroBiasOnStartup;
    float gyroSmoothFactor;
    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    uint8_t magCalibrated;
    int32_t magBias[3];
    
    uint8_t mpu6050Scale;

    // For Mahony AHRS
    
    float accelKp;
    float accelKi;
    
    float magKp;
    float magKi;

    uint8_t magDriftCompensation;
    
    float magDeclination;
    
    float batScale;
    float batMinCellVoltage;
    float batMaxCellVoltage;
    
    uint16_t startupDelay;
    
    motorMixer_t customMixer[MAX_MOTORS];   // custom mixtable
    
    uint8_t magic_ef;        // magic number, should be 0xEF
    uint8_t chk;             // XOR checksum
} config_t;

// External Variables

extern config_t cfg;
extern int16_t lookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[11];   // lookup table for expo & mid THROTTLE
extern const char rcChannelLetters[8];

// Functions

void parseRcChannels(const char *input);
void readEEPROM(void);
void writeParams(void);
void checkFirstTime(bool reset);
bool featureGet(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);
void featureClearAll(void);
uint32_t featureMask(void);


