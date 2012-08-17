/*
 * From Timecop's Original Baseflight
 */

#include "board.h"

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

    pwmWrite(0, servo[0]);
    pwmWrite(1, servo[1]);
}

///////////////////////////////////////////////////////////////////////////////
// Write to Motors
///////////////////////////////////////////////////////////////////////////////

void writeMotors(void)
{
    uint8_t i;
    uint8_t offset = 0;

    // when servos are enabled, pwm outputs 1 and 2 are for servos only
    if (useServos)
        offset = 2;

    for (i = 0; i < numberMotor; i++)
        pwmWrite(i + offset, motor[i]);
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

#define PIDMIX(X,Y,Z) command[THROTTLE] + axisPID[THROTTLE] + axisPID[ROLL] * X + axisPID[PITCH] * Y + cfg.yawDirection * axisPID[YAW] * Z

void mixTable(void)
{
    int16_t maxMotor;
    uint8_t i;

    ///////////////////////////////////
    /*
    if ( numberMotor > 3 )
    {
        //prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - fabs(rcCommand[YAW]), +100 + fabs(rcCommand[YAW]));
    }
    */
    ///////////////////////////////////

    switch ( cfg.mixerConfiguration )
    {
        case MULTITYPE_BI:
            motor[0] = PIDMIX(  1.0f, 0.0f, 0.0f );        // Left Motor
            motor[1] = PIDMIX( -1.0f, 0.0f, 0.0f );        // Right Motor

            servo[0] = constrain( cfg.biLeftServoMid + (cfg.yawDirection * axisPID[YAW]) + axisPID[PITCH],
                                  cfg.biLeftServoMin, cfg.biLeftServoMax );   // Left Servo
            servo[1] = constrain( cfg.biRightServoMid + (cfg.yawDirection * axisPID[YAW]) - axisPID[PITCH],
                                  cfg.biRightServoMin, cfg.biRightServoMax );   // Right Servo
            break;

        case MULTITYPE_TRI:
            motor[0] = PIDMIX(  1.0f, -2.0f/3.0f, 0.0f );  // Left  CW
            motor[1] = PIDMIX( -1.0f, -2.0f/3.0f, 0.0f );  // Right CCW
            motor[2] = PIDMIX(  0.0f,  4.0f/3.0f, 0.0f );  // Rear  CW or CCW

            servo[0] = constrain( cfg.triYawServoMid + cfg.yawDirection * axisPID[YAW],
                                  cfg.triYawServoMin, cfg.triYawServoMax ); // Tail Servo
            break;

        case MULTITYPE_QUADP:
            motor[0] = PIDMIX(  0.0f, -1.0f, -1.0f );      // Front CW
            motor[1] = PIDMIX( -1.0f,  0.0f,  1.0f );      // Right CCW
            motor[2] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Rear  CW
            motor[3] = PIDMIX(  1.0f,  0.0f,  1.0f );      // Left  CCW
            break;

        case MULTITYPE_QUADX:
            motor[0] = PIDMIX(  1.0f, -1.0f, -1.0f );      // Front Left  CW
            motor[1] = PIDMIX( -1.0f, -1.0f,  1.0f );      // Front Right CCW
            motor[2] = PIDMIX( -1.0f,  1.0f, -1.0f );      // Rear Right  CW
            motor[3] = PIDMIX(  1.0f,  1.0f,  1.0f );      // Rear Left   CCW
            break;

        case MULTITYPE_Y4:
            motor[0] = PIDMIX(  1.0f, -1.0f,  0.0f );      // Front Left  CW
            motor[1] = PIDMIX( -1.0f, -1.0f,  0.0f );      // Front Right CCW
            motor[2] = PIDMIX(  0.0f,  1.0f, -1.0f );      // Top Rear    CW
            motor[3] = PIDMIX(  0.0f,  1.0f,  1.0f );      // Bottom Rear CCW
            break;

        case MULTITYPE_Y6:
            motor[0] = PIDMIX(  1.0f, -2.0f/3.0f, -1.0f );  // Top Left     CW
            motor[1] = PIDMIX( -1.0f, -2.0f/3.0f, -1.0f );  // Top Right    CW
            motor[2] = PIDMIX(  0.0f,  4.0f/3.0f,  1.0f );  // Top Rear     CCW
            motor[3] = PIDMIX(  1.0f, -2.0f/3.0f,  1.0f );  // Bottom Left  CCW
            motor[4] = PIDMIX( -1.0f, -2.0f/3.0f,  1.0f );  // Bottom Right CCW
            motor[5] = PIDMIX(  0.0f,  4.0f/3.0f, -1.0f );  // Bottom Rear  CW
            break;

        case MULTITYPE_HEX6P:
            motor[0] = PIDMIX(  0.0f, -7.0f/8.0f, -1.0f ); // Front       CW
            motor[1] = PIDMIX( -1.0f, -7.0f/8.0f,  1.0f ); // Front Right CCW
            motor[2] = PIDMIX( -1.0f,  7.0f/8.0f, -1.0f ); // Rear Right  CW
            motor[3] = PIDMIX(  0.0f,  7.0f/8.0f,  1.0f ); // Rear        CCW
            motor[4] = PIDMIX(  1.0f,  7.0f/8.0f, -1.0f ); // Rear Left   CW
            motor[5] = PIDMIX(  1.0f, -7.0f/8.0f,  1.0f ); // Front Left  CCW
            break;

        case MULTITYPE_HEX6X:
            motor[0] = PIDMIX(  7.0f/8.0f, -1.0f, -1.0f ); // Front Left  CW
            motor[1] = PIDMIX( -7.0f/8.0f, -1.0f,  1.0f ); // Front Right CCW
            motor[2] = PIDMIX( -7.0f/8.0f,  0.0f, -1.0f ); // Right       CW
            motor[3] = PIDMIX( -7.0f/8.0f,  1.0f,  1.0f ); // Rear Right  CCW
            motor[4] = PIDMIX(  7.0f/8.0f,  1.0f, -1.0f ); // Rear Left   CW
            motor[5] = PIDMIX(  7.0f/8.0f,  0.0f,  1.0f ); // Left        CCW
            break;

        case MULTITYPE_VTAIL4:
            motor[0] = PIDMIX(  1.0f, -1.0f,  0.0f );   // Front Left  CCW - NOTE rotation difference from all other configurations
            motor[1] = PIDMIX( -1.0f, -1.0f,  0.0f );   // Front Right CW  - NOTE rotation difference from all other configurations
            motor[2] = PIDMIX(  0.0f,  1.0f,  1.0f );   // Rear Right  CCW - NOTE rotation difference from all other configurations
            motor[3] = PIDMIX(  0.0f,  1.0f, -1.0f );   // Rear Left   CW  - NOTE rotation difference from all other configurations
            break;

        case MULTITYPE_FREEMIX:
        	for ( i = 0; i < numberMotor; i++ )
        	{
        		motor[i] = PIDMIX ( cfg.freeMix[i][ROLL],
        				            cfg.freeMix[i][PITCH],
        				            cfg.freeMix[i][YAW] );
        	}
        	break;

        case MULTITYPE_GIMBAL:
            servo[0] = constrain( cfg.gimbalRollServoMid + cfg.gimbalRollServoGain * sensors.attitude[ROLL] + command[ROLL],
                                  cfg.gimbalRollServoMin, cfg.gimbalRollServoMax );

            servo[1] = constrain( cfg.gimbalPitchServoMid + cfg.gimbalPitchServoGain * sensors.attitude[PITCH] + command[PITCH],
                                  cfg.gimbalPitchServoMin, cfg.gimbalPitchServoMax );
            break;

        case MULTITYPE_FLYING_WING:
            motor[0] = command[THROTTLE];
            if (!mode.LEVEL_MODE)
            { // do not use sensors for correction, simple 2 channel mixing
            	servo[0] = cfg.pitchDirectionLeft  * (command[PITCH] - cfg.midCommand) +
            			   cfg.rollDirectionLeft   * (command[ROLL ] - cfg.midCommand);
            	servo[1] = cfg.pitchDirectionRight * (command[PITCH] - cfg.midCommand) +
            			   cfg.rollDirectionRight  * (command[ROLL]  - cfg.midCommand);
            }
            else
            { // use sensors to correct (attitude only)
            	servo[0] = cfg.pitchDirectionLeft  * axisPID[PITCH] +
            			   cfg.rollDirectionLeft   * axisPID[ROLL];
            	servo[1] = cfg.pitchDirectionRight * axisPID[PITCH] +
            			   cfg.rollDirectionRight  * axisPID[ROLL];
            }
            servo[0] = constrain(servo[0] + cfg.midCommand, cfg.wingLeftMinimum,
            		                                                 cfg.wingLeftMaximum);
            servo[1] = constrain(servo[1] + cfg.midCommand, cfg.wingRightMinimum,
            		                                                 cfg.wingRightMaximum);
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
