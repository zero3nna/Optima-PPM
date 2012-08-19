/*
 * From Timecop's Original Baseflight
 */

#include "board.h"

#include "actuator/mixer.h"
#include "actuator/stabilisation.h"

#include "core/command.h"

#include "drivers/pwm_ppm.h"

///////////////////////////////////////////////////////////////////////////////

uint8_t numberMotor;

uint16_t motor[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

uint16_t servo[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

bool useServos = false;

///////////////////////////////////////////////////////////////////////////////
// Initialize Mixer
///////////////////////////////////////////////////////////////////////////////

void initMixer(void)
{
    // enable servos for mixes that require them. note, this shifts motor counts.
    if ( cfg.mixerConfiguration == MULTITYPE_GIMBAL    ||
         cfg.mixerConfiguration == MULTITYPE_BI        ||
         cfg.mixerConfiguration == MULTITYPE_TRI       ||
         cfg.mixerConfiguration == MULTITYPE_QUADP     ||
         cfg.mixerConfiguration == MULTITYPE_QUADX     ||
         cfg.mixerConfiguration == MULTITYPE_Y4        ||
         cfg.mixerConfiguration == MULTITYPE_VTAIL4    ||
        (cfg.mixerConfiguration == MULTITYPE_FREEMIX   &&
         cfg.freeMixMotors < 5)                        ||
         cfg.mixerConfiguration == MULTITYPE_FLYING_WING ) {
             useServos = true;
    } else {
        useServos = false;
    }

    switch (cfg.mixerConfiguration)
    {
        case MULTITYPE_GIMBAL:
            numberMotor = 0;
            break;

        case MULTITYPE_BI:
            numberMotor = 2;
            break;

        case MULTITYPE_TRI:
            numberMotor = 3;
            break;

        case MULTITYPE_QUADP:
        case MULTITYPE_QUADX:
        case MULTITYPE_Y4:
        case MULTITYPE_VTAIL4:
            numberMotor = 4;
            break;

        case MULTITYPE_Y6:
        case MULTITYPE_HEX6P:
        case MULTITYPE_HEX6X:
            numberMotor = 6;
            break;

        case MULTITYPE_FREEMIX:
        	numberMotor = cfg.freeMixMotors;
        	break;

        case MULTITYPE_FLYING_WING:
           numberMotor = 1;
           break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Write to Servos
///////////////////////////////////////////////////////////////////////////////

void writeServos(void)
{
    if (!useServos)
        return;

    if (cfg.mixerConfiguration == MULTITYPE_TRI || cfg.mixerConfiguration == MULTITYPE_BI) {
        /* One servo on Motor #4 */
        pwmWriteServo(0, servo[4]);
        if (cfg.mixerConfiguration == MULTITYPE_BI)
            pwmWriteServo(1, servo[5]);
    } else {
        /* Two servos for camstab or FLYING_WING */
        pwmWriteServo(0, servo[0]);
        pwmWriteServo(1, servo[1]);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Write to Motors
///////////////////////////////////////////////////////////////////////////////

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < numberMotor; i++)
        pwmWriteMotor(i, motor[i]);
}

///////////////////////////////////////////////////////////////////////////////
// Write to All Motors
///////////////////////////////////////////////////////////////////////////////

void writeAllMotors(uint16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

///////////////////////////////////////////////////////////////////////////////
// Pulse Motors
///////////////////////////////////////////////////////////////////////////////

void pulseMotors(uint8_t quantity)
{
    uint8_t i;

    for ( i = 0; i < quantity; i++ )
    {
        writeAllMotors( cfg.minThrottle );
        delay(250);
        writeAllMotors(MINCOMMAND);
        delay(250);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Mixer
///////////////////////////////////////////////////////////////////////////////

#define PIDMIX(X,Y,Z) command[THROTTLE] + axisPID[THROTTLE] + axisPID[THROTTLE] + axisPID[ROLL] * X + axisPID[PITCH] * Y + cfg.yawDirection * axisPID[YAW] * Z

void mixTable(void)
{
    int16_t maxMotor;
    uint8_t i;

    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(command[YAW]), +100 + abs(command[YAW]));
    }

    switch (cfg.mixerConfiguration) {

        case MULTITYPE_BI:
            motor[0] = PIDMIX(+1, 0, 0);        //LEFT
            motor[1] = PIDMIX(-1, 0, 0);        //RIGHT        
            servo[4] = constrain(1500 + (cfg.yawDirection * axisPID[YAW]) + axisPID[PITCH], 1020, 2000);   //LEFT
            servo[5] = constrain(1500 + (cfg.yawDirection * axisPID[YAW]) - axisPID[PITCH], 1020, 2000);   //RIGHT
            break;

        case MULTITYPE_TRI:
            motor[0] = PIDMIX(0, +4 / 3, 0);    //REAR
            motor[1] = PIDMIX(-1, -2 / 3, 0);   //RIGHT
            motor[2] = PIDMIX(+1, -2 / 3, 0);   //LEFT
            servo[4] = constrain(cfg.triYawServoMid + cfg.yawDirection * axisPID[YAW], cfg.triYawServoMin, cfg.triYawServoMax); //REAR
            break;

        case MULTITYPE_QUADP:
            motor[0] = PIDMIX(0, +1, -1);       //REAR
            motor[1] = PIDMIX(-1, 0, +1);       //RIGHT
            motor[2] = PIDMIX(+1, 0, +1);       //LEFT
            motor[3] = PIDMIX(0, -1, -1);       //FRONT
            break;

        case MULTITYPE_QUADX:
            motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
            motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
            motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
            motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
            break;

        case MULTITYPE_Y4:
            motor[0] = PIDMIX(+0, +1, -1);      //REAR_1 CW
            motor[1] = PIDMIX(-1, -1, 0);       //FRONT_R CCW
            motor[2] = PIDMIX(+0, +1, +1);      //REAR_2 CCW
            motor[3] = PIDMIX(+1, -1, 0);       //FRONT_L CW
            break;

        case MULTITYPE_Y6:
            motor[0] = PIDMIX(+0, +4 / 3, +1);  //REAR
            motor[1] = PIDMIX(-1, -2 / 3, -1);  //RIGHT
            motor[2] = PIDMIX(+1, -2 / 3, -1);  //LEFT
            motor[3] = PIDMIX(+0, +4 / 3, -1);  //UNDER_REAR
            motor[4] = PIDMIX(-1, -2 / 3, +1);  //UNDER_RIGHT
            motor[5] = PIDMIX(+1, -2 / 3, +1);  //UNDER_LEFT    
            break;

        case MULTITYPE_HEX6P:
            motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
            motor[1] = PIDMIX(-1 / 2, -1 / 2, -1);      //FRONT_R
            motor[2] = PIDMIX(+1 / 2, +1 / 2, +1);      //REAR_L
            motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
            motor[4] = PIDMIX(+0, -1, +1);      //FRONT
            motor[5] = PIDMIX(+0, +1, -1);      //REAR
            break;

        case MULTITYPE_HEX6X:
#if 0
            motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
            motor[1] = PIDMIX(-1 / 2, -1 / 2, +1);      //FRONT_R
            motor[2] = PIDMIX(+1 / 2, +1 / 2, -1);      //REAR_L
            motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
            motor[4] = PIDMIX(-1, +0, -1);      //RIGHT
            motor[5] = PIDMIX(+1, +0, +1);      //LEFT
#else
            motor[0] = PIDMIX(-4/5,+9/10,+1); //REAR_R 
            motor[1] = PIDMIX(-4/5,-9/10,+1); //FRONT_R 
            motor[2] = PIDMIX(+4/5,+9/10,-1); //REAR_L 
            motor[3] = PIDMIX(+4/5,-9/10,-1); //FRONT_L 
            motor[4] = PIDMIX(-4/5 ,+0 ,-1); //RIGHT 
            motor[5] = PIDMIX(+4/5 ,+0 ,+1); //LEFT
#endif
            break;

        case MULTITYPE_OCTOX8:
            motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
            motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
            motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
            motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
            motor[4] = PIDMIX(-1, +1, +1);      //UNDER_REAR_R
            motor[5] = PIDMIX(-1, -1, -1);      //UNDER_FRONT_R
            motor[6] = PIDMIX(+1, +1, -1);      //UNDER_REAR_L
            motor[7] = PIDMIX(+1, -1, +1);      //UNDER_FRONT_L
            break;

        case MULTITYPE_OCTOFLATP:
            motor[0] = PIDMIX(+7 / 10, -7 / 10, +1);    //FRONT_L
            motor[1] = PIDMIX(-7 / 10, -7 / 10, +1);    //FRONT_R
            motor[2] = PIDMIX(-7 / 10, +7 / 10, +1);    //REAR_R
            motor[3] = PIDMIX(+7 / 10, +7 / 10, +1);    //REAR_L
            motor[4] = PIDMIX(+0, -1, -1);      //FRONT
            motor[5] = PIDMIX(-1, +0, -1);      //RIGHT
            motor[6] = PIDMIX(+0, +1, -1);      //REAR
            motor[7] = PIDMIX(+1, +0, -1);      //LEFT 
            break;

        case MULTITYPE_OCTOFLATX:
            motor[0] = PIDMIX(+1, -1 / 2, +1);  //MIDFRONT_L
            motor[1] = PIDMIX(-1 / 2, -1, +1);  //FRONT_R
            motor[2] = PIDMIX(-1, +1 / 2, +1);  //MIDREAR_R
            motor[3] = PIDMIX(+1 / 2, +1, +1);  //REAR_L
            motor[4] = PIDMIX(+1 / 2, -1, -1);  //FRONT_L
            motor[5] = PIDMIX(-1, -1 / 2, -1);  //MIDFRONT_R
            motor[6] = PIDMIX(-1 / 2, +1, -1);  //REAR_R
            motor[7] = PIDMIX(+1, +1 / 2, -1);  //MIDREAR_L 
            break;

        case MULTITYPE_VTAIL4:
            motor[0] = PIDMIX(+0, +1, +1);      //REAR_R 
            motor[1] = PIDMIX(-1, -1, +0);      //FRONT_R 
            motor[2] = PIDMIX(+0, +1, -1);      //REAR_L 
            motor[3] = PIDMIX(+1, -1, -0);      //FRONT_L
            break;

        case MULTITYPE_GIMBAL:
            servo[0] = constrain(cfg.gimbalPitchServoMid + cfg.gimbalPitchServoGain * + command[PITCH], cfg.gimbalPitchServoMin, cfg.gimbalPitchServoMax);
            servo[1] = constrain(cfg.gimbalRollServoMid + cfg.gimbalRollServoGain * + command[ROLL], cfg.gimbalRollServoMin, cfg.gimbalRollServoMax);
            break;
        
        case MULTITYPE_FREEMIX:
        	for ( i = 0; i < numberMotor; i++ )
        	{
        		motor[i] = PIDMIX ( cfg.freeMix[i][ROLL],
        				            cfg.freeMix[i][PITCH],
        				            cfg.freeMix[i][YAW] );
        	}
        	break;
    }

    ///////////////////////////////////
    
    // TODO Add Gimble

    // this is a way to still have good gyro corrections if any motor reaches its max.

    maxMotor = motor[0];

    for (i = 1; i < numberMotor; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];

    for (i = 0; i < numberMotor; i++)
    {
        if (maxMotor > cfg.maxThrottle)
            motor[i] -= maxMotor - cfg.maxThrottle;

        motor[i] = constrain(motor[i], cfg.minThrottle, cfg.maxThrottle);

        if ((rcData[THROTTLE]) < cfg.minCheck)
        {
            if(cfg.motorStop) {
                motor[i] = cfg.minCommand;
            } else {
                motor[i] = cfg.minThrottle;
            }
        }

        if (!mode.ARMED)
            motor[i] = MINCOMMAND;
    }
}

///////////////////////////////////////////////////////////////////////////////
