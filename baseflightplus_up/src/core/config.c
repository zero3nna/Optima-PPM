/*
 * From Timecop's Original Baseflight
 * With inspiration from Baseflight Plus
 *
 * Modified for Baseflight U.P. support - Scott Driessens (August 2012)
 */

#include "board.h"
#include "core/command.h"

///////////////////////////////////////////////////////////////////////////////

#define FLASH_PAGE_COUNT 128

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)

// use the last KB for sensor config storage
#define FLASH_WRITE_SENSOR_CONFIG_ADDR  (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))

// use the 2nd to last KB for system config storage
#define FLASH_WRITE_SYSTEM_CONFIG_ADDR  (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 2))

sensorConfig_t sensorConfig;

systemConfig_t systemConfig;

const char rcChannelLetters[] = "AERT1234";

static uint8_t checkNewSensorConf = 1;
static uint8_t checkNewSystemConf = 1;

///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            systemConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////

void readEEPROM(void)
{
    // Read flash

	memcpy(&sensorConfig, (char *)FLASH_WRITE_SENSOR_CONFIG_ADDR, sizeof(sensorConfig_t));

    memcpy(&systemConfig, (char *)FLASH_WRITE_SYSTEM_CONFIG_ADDR, sizeof(systemConfig_t));
    
}

///////////////////////////////////////////////////////////////////////////////

void writeSensorParams(void)
{
    FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_SENSOR_CONFIG_ADDR) == FLASH_COMPLETE)
    {
        for (i = 0; i < sizeof(sensorConfig_t); i += 4)
        {
            status = FLASH_ProgramWord(FLASH_WRITE_SENSOR_CONFIG_ADDR + i, *(uint32_t *)((char *)&sensorConfig + i));
            if (status != FLASH_COMPLETE)
                while(1);
                //break; // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
}

///////////////////////////////////////////////////////////////////////////////

void writeSystemParams(void)
{
    FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_SYSTEM_CONFIG_ADDR) == FLASH_COMPLETE)
    {
        for (i = 0; i < sizeof(systemConfig_t); i += 4)
        {
            status = FLASH_ProgramWord(FLASH_WRITE_SYSTEM_CONFIG_ADDR + i, *(uint32_t *)((char *)&systemConfig + i));
            if (status != FLASH_COMPLETE)
                while(1);//break; // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
}

///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool sensorReset, bool systemReset)
{
    uint8_t test_val, i;

    test_val = *(uint8_t *)FLASH_WRITE_SENSOR_CONFIG_ADDR;

    if (sensorReset || test_val != checkNewSensorConf)
    {
		// Default settings
        sensorConfig.version = checkNewSensorConf;

	    ///////////////////////////////

        sensorConfig.accelCalibrated =      false;
        sensorConfig.accelBias[XAXIS] =     0.0f;
        sensorConfig.accelBias[YAXIS] =     0.0f;
        sensorConfig.accelBias[ZAXIS] =     0.0f;

        ///////////////////////////////

        sensorConfig.gyroTCBiasSlope[ROLL ] =   0.0f;
        sensorConfig.gyroTCBiasSlope[PITCH] =   0.0f;
        sensorConfig.gyroTCBiasSlope[YAW  ] =   0.0f;

	    ///////////////////////////////

	    sensorConfig.gyroTCBiasIntercept[ROLL ] =   0.0f;
	    sensorConfig.gyroTCBiasIntercept[PITCH] =   0.0f;
	    sensorConfig.gyroTCBiasIntercept[YAW  ] =   0.0f;

	    ///////////////////////////////

        sensorConfig.magCalibrated = false;
	    sensorConfig.magBias[XAXIS] =   0.0f;
	    sensorConfig.magBias[YAXIS] =   0.0f;
	    sensorConfig.magBias[ZAXIS] =   0.0f;

        ///////////////////////////////
        
        sensorConfig.magDriftCompensation = true;

        sensorConfig.twoKp =            2.0f; // 2 * proportional gain
        sensorConfig.twoKi =            0.1f; // 2 * integral gain

	    ///////////////////////////////
	    
        sensorConfig.battery                 = false;
	    sensorConfig.batScale                = 11.0f;
        sensorConfig.batMinCellVoltage       = 3.3f;
        sensorConfig.batMaxCellVoltage       = 4.2f;
        
        ///////////////////////////////

	    writeSensorParams();
	}

    ///////////////////////////////////

    if (systemReset || test_val != checkNewSystemConf)
    {
		// Default settings
        systemConfig.version = checkNewSystemConf;
        
        parseRcChannels("AETR1234");
        
        systemConfig.usePPM                         = true;
        systemConfig.escPwmRate                     = 400;
        systemConfig.servoPwmRate                   = 50;
        
        systemConfig.failsafe                       = false;
        systemConfig.failsafeOnDelay                = 50; // Number of command loops (50Hz) until failsafe kicks in
        systemConfig.failsafeOffDelay               = 20000; // Number of command loops until failsafe stops
        systemConfig.failsafeThrottle               = 1200;
        
        for(i = 0; i < AUX_OPTIONS; ++i)
            systemConfig.auxActivate[i] = 0;

        systemConfig.mixerConfiguration                 = MULTITYPE_QUADX;

        systemConfig.minCommand                         = 1000;
        systemConfig.midCommand                         = 1500;
        systemConfig.maxCommand                         = 2000;
        systemConfig.minCheck                           = 1100;
        systemConfig.maxCheck                           = 1900;
        systemConfig.minThrottle                        = 1150;
        systemConfig.maxThrottle                        = 1850;
        systemConfig.motorStop                          = true;
        
        systemConfig.deadBand[ROLL]                     = 12;
        systemConfig.deadBand[PITCH]                    = 12;
        systemConfig.deadBand[YAW]                      = 12;
        
        systemConfig.biLeftServoMin                     = 1000;
        systemConfig.biLeftServoMid                     = 1500;
        systemConfig.biLeftServoMax                     = 2000;

        systemConfig.biRightServoMin                    = 1000;
        systemConfig.biRightServoMid                    = 1500;
        systemConfig.biRightServoMax                    = 2000;

        systemConfig.yawDirection                       = 1;
        systemConfig.triYawServoMin                     = 1000;
        systemConfig.triYawServoMid                     = 1500;
        systemConfig.triYawServoMax                     = 2000;

        systemConfig.gimbalRollServoMin                 = 1000;
		systemConfig.gimbalRollServoMid                 = 1500;
		systemConfig.gimbalRollServoMax                 = 2000;
		systemConfig.gimbalRollServoGain                = 1.0f;

		systemConfig.gimbalPitchServoMin                = 1000;
		systemConfig.gimbalPitchServoMid                = 1500;
		systemConfig.gimbalPitchServoMax                = 2000;
		systemConfig.gimbalPitchServoGain               = 1.0f;

        // Free Mix Defaults to Quad X
		systemConfig.freeMixMotors = 4;

		systemConfig.freeMix[0][ROLL]                   =  1.0f;
        systemConfig.freeMix[0][PITCH]                  = -1.0f;
        systemConfig.freeMix[0][YAW]                    = -1.0f;

        systemConfig.freeMix[1][ROLL]                   = -1.0f;
        systemConfig.freeMix[1][PITCH]                  = -1.0f;
        systemConfig.freeMix[1][YAW]                    =  1.0f;

        systemConfig.freeMix[2][ROLL]                   = -1.0f;
        systemConfig.freeMix[2][PITCH]                  =  1.0f;
        systemConfig.freeMix[2][YAW]                    = -1.0f;

        systemConfig.freeMix[3][ROLL]                   =  1.0f;
        systemConfig.freeMix[3][PITCH]                  =  1.0f;
        systemConfig.freeMix[3][YAW]                    =  1.0f;

        systemConfig.freeMix[4][ROLL]                   =  0.0f;
        systemConfig.freeMix[4][PITCH]                  =  0.0f;
        systemConfig.freeMix[4][YAW]                    =  0.0f;

        systemConfig.freeMix[5][ROLL]                   =  0.0f;
        systemConfig.freeMix[5][PITCH]                  =  0.0f;
        systemConfig.freeMix[5][YAW]                    =  0.0f;

        systemConfig.rollDirectionLeft                  = -1;
        systemConfig.rollDirectionRight                 =  1;
        systemConfig.pitchDirectionLeft                 = -1;
        systemConfig.pitchDirectionRight                =  1;
        
        systemConfig.wingLeftMinimum                    =  1000;
        systemConfig.wingLeftMaximum                    =  2000;
        systemConfig.wingRightMinimum                   =  1000;
        systemConfig.wingRightMaximum                   =  2000;

        systemConfig.pids[ROLL_RATE_PID].p               = 100.0f;
        systemConfig.pids[ROLL_RATE_PID].i               =   0.0f;
        systemConfig.pids[ROLL_RATE_PID].d               =   0.0f;
        systemConfig.pids[ROLL_RATE_PID].iLim            = 100.0f;  // PWMs

        systemConfig.pids[PITCH_RATE_PID].p              = 100.0f;
        systemConfig.pids[PITCH_RATE_PID].i              =   0.0f;
        systemConfig.pids[PITCH_RATE_PID].d              =   0.0f;
        systemConfig.pids[PITCH_RATE_PID].iLim           = 100.0f;  // PWMs

        systemConfig.pids[YAW_RATE_PID].p                = 200.0f;
        systemConfig.pids[YAW_RATE_PID].i                =   0.0f;
        systemConfig.pids[YAW_RATE_PID].d                =   0.0f;
        systemConfig.pids[YAW_RATE_PID].iLim             = 100.0f;  // PWMs

        systemConfig.pids[ROLL_LEVEL_PID].p                =   2.0f;
        systemConfig.pids[ROLL_LEVEL_PID].i                =   0.0f;
        systemConfig.pids[ROLL_LEVEL_PID].d                =   0.0f;
        systemConfig.pids[ROLL_LEVEL_PID].iLim             =   0.5f;  // radians/sec

        systemConfig.pids[PITCH_LEVEL_PID].p               =   2.0f;
        systemConfig.pids[PITCH_LEVEL_PID].i               =   0.0f;
        systemConfig.pids[PITCH_LEVEL_PID].d               =   0.0f;
        systemConfig.pids[PITCH_LEVEL_PID].iLim           =   0.5f;  // radians/sec

        systemConfig.pids[HEADING_PID].p                 =   1.5f;
        systemConfig.pids[HEADING_PID].i                 =   0.0f;
        systemConfig.pids[HEADING_PID].d                 =   0.0f;
        systemConfig.pids[HEADING_PID].iLim             =   0.5f;  // radians/sec

        writeSystemParams();
	}
}

///////////////////////////////////////////////////////////////////////////////
