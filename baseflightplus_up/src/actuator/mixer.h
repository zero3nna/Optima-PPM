/*
 * From Timecop's Original Baseflight
 */

#pragma once

#include "drivers/pwm_ppm.h"

///////////////////////////////////////////////////////////////////////////////
// Mixer Types
///////////////////////////////////////////////////////////////////////////////

typedef enum {
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6P = 7,
    MULTITYPE_FLYING_WING = 8, // UNSUPPORTED
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,
    MULTITYPE_OCTOFLATP = 12,
    MULTITYPE_OCTOFLATX = 13,
    MULTITYPE_AIRPLANE = 14, // Airplane, single copter, dual copter
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,
    MULTITYPE_LAST = 19
} MultiType;

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

//extern uint8_t numberMotor;

extern int16_t motor[MAX_MOTORS];

extern int16_t servo[MAX_SERVOS];

extern uint8_t useServos;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void mixerInit(void);

void mixTable(void);

void writeServos(void);

void writeMotors(void);

void writeAllMotors(int16_t mc);

void pulseMotors(uint8_t quantity);

///////////////////////////////////////////////////////////////////////////////
