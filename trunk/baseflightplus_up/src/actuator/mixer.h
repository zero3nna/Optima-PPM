/*
 * From Timecop's Original Baseflight
 */

#pragma once

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
    MULTITYPE_FLYING_WING = 8,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,
    MULTITYPE_OCTOFLATP = 12,
    MULTITYPE_OCTOFLATX = 13,
    MULTITYPE_AIRPLANE = 14,
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_FREEMIX = 18,
} MultiType;

///////////////////////////////////////////////////////////////////////////////
// External Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t numberMotor;

extern uint16_t motor[8];

extern uint16_t servo[8];

extern bool useServos;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void initMixer(void);

void mixTable(void);

void writeServos(void);

void writeMotors(void);

void writeAllMotors(uint16_t mc);

void pulseMotors(uint8_t quantity);

///////////////////////////////////////////////////////////////////////////////
