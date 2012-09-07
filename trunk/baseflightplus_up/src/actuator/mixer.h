/*
 * From Timecop's Original Baseflight
 */

#pragma once

#include "drivers/pwm_ppm.h"

///////////////////////////////////////////////////////////////////////////////
// Mixer Types
///////////////////////////////////////////////////////////////////////////////

// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,      // UNSUPPORTED, do not select!
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,          // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,       // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,       // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,        // airplane / singlecopter / dualcopter
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,          // no current GUI displays this
    MULTITYPE_LAST = 19
} MultiType;

typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

//extern uint8_t numberMotor;

extern int16_t motor[MAX_MOTORS];

extern int16_t servo[MAX_SERVOS];

extern uint8_t useServo;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void mixerInit(void);

void mixerLoadMix(int index);

void mixTable(void);

void writeServos(void);

void writeMotors(void);

void writeAllMotors(int16_t mc);

void pulseMotors(uint8_t quantity);

///////////////////////////////////////////////////////////////////////////////
