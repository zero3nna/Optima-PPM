/*
 * From Timecop's Original Baseflight
 *
 * Modified for Baseflight U.P. support - Scott Driessens (August 2012)
 */

#include "board.h"
#include "core/command.h"
#include "actuator/mixer.h"

///////////////////////////////////////////////////////////////////////////////

#define FLASH_PAGE_COUNT 128

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)

// use the last KB for config storage
#define FLASH_WRITE_SYSTEM_CONFIG_ADDR  (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))

cfg_t cfg;

const char rcChannelLetters[] = "AERT1234";

static uint8_t checkNewSystemConf = 2;

///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            cfg.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////

void readEEPROM(void)
{
    // Read flash

    memcpy(&cfg, (char *)FLASH_WRITE_SYSTEM_CONFIG_ADDR, sizeof(cfg_t));
    
}

///////////////////////////////////////////////////////////////////////////////

void writeParams(void)
{
    FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_SYSTEM_CONFIG_ADDR) == FLASH_COMPLETE)
    {
        for (i = 0; i < sizeof(cfg_t); i += 4)
        {
            status = FLASH_ProgramWord(FLASH_WRITE_SYSTEM_CONFIG_ADDR + i, *(uint32_t *)((char *)&cfg + i));
            if (status != FLASH_COMPLETE)
                while(1);//break; // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
}

///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool reset)
{
    uint8_t test_val, i;

    test_val = *(uint8_t *)FLASH_WRITE_SYSTEM_CONFIG_ADDR;

    ///////////////////////////////////

    if (reset || test_val != checkNewSystemConf)
    {
		// Default settings
        cfg.version = checkNewSystemConf;
        
        parseRcChannels("AETR1234");
        
        cfg.usePPM                         = true;
        cfg.escPwmRate                     = 400;
        cfg.servoPwmRate                   = 50;
        
        cfg.failsafe                       = false;
        cfg.failsafeOnDelay                = 50; // Number of command loops (50Hz) until failsafe kicks in
        cfg.failsafeOffDelay               = 20000; // Number of command loops until failsafe stops
        cfg.failsafeThrottle               = 1200;
        
        for(i = 0; i < AUX_OPTIONS; ++i)
            cfg.auxActivate[i] = 0;

        cfg.mixerConfiguration                 = MULTITYPE_QUADX;

        cfg.minCommand                         = 1000;
        cfg.midCommand                         = 1500;
        cfg.maxCommand                         = 2000;
        cfg.minCheck                           = 1100;
        cfg.maxCheck                           = 1900;
        cfg.minThrottle                        = 1150;
        cfg.maxThrottle                        = 1850;
        cfg.motorStop                          = true;
        
        cfg.deadBand[ROLL]                     = 12;
        cfg.deadBand[PITCH]                    = 12;
        cfg.deadBand[YAW]                      = 12;
        
        cfg.biLeftServoMin                     = 1000;
        cfg.biLeftServoMid                     = 1500;
        cfg.biLeftServoMax                     = 2000;

        cfg.biRightServoMin                    = 1000;
        cfg.biRightServoMid                    = 1500;
        cfg.biRightServoMax                    = 2000;

        cfg.yawDirection                       = 1;
        cfg.triYawServoMin                     = 1000;
        cfg.triYawServoMid                     = 1500;
        cfg.triYawServoMax                     = 2000;

        cfg.gimbalRollServoMin                 = 1000;
		cfg.gimbalRollServoMid                 = 1500;
		cfg.gimbalRollServoMax                 = 2000;
		cfg.gimbalRollServoGain                = 1.0f;

		cfg.gimbalPitchServoMin                = 1000;
		cfg.gimbalPitchServoMid                = 1500;
		cfg.gimbalPitchServoMax                = 2000;
		cfg.gimbalPitchServoGain               = 1.0f;

        cfg.rollDirectionLeft                  = -1;
        cfg.rollDirectionRight                 =  1;
        cfg.pitchDirectionLeft                 = -1;
        cfg.pitchDirectionRight                =  1;
        
        cfg.wingLeftMinimum                    =  1000;
        cfg.wingLeftMaximum                    =  2000;
        cfg.wingRightMinimum                   =  1000;
        cfg.wingRightMaximum                   =  2000;

        cfg.pids[ROLL_RATE_PID].p               = 100.0f;
        cfg.pids[ROLL_RATE_PID].i               =   0.0f;
        cfg.pids[ROLL_RATE_PID].d               =   0.0f;
        cfg.pids[ROLL_RATE_PID].iLim            = 100.0f;  // PWMs

        cfg.pids[PITCH_RATE_PID].p              = 100.0f;
        cfg.pids[PITCH_RATE_PID].i              =   0.0f;
        cfg.pids[PITCH_RATE_PID].d              =   0.0f;
        cfg.pids[PITCH_RATE_PID].iLim           = 100.0f;  // PWMs

        cfg.pids[YAW_RATE_PID].p                = 200.0f;
        cfg.pids[YAW_RATE_PID].i                =   0.0f;
        cfg.pids[YAW_RATE_PID].d                =   0.0f;
        cfg.pids[YAW_RATE_PID].iLim             = 100.0f;  // PWMs

        cfg.pids[ROLL_LEVEL_PID].p                =   2.0f;
        cfg.pids[ROLL_LEVEL_PID].i                =   0.0f;
        cfg.pids[ROLL_LEVEL_PID].d                =   0.0f;
        cfg.pids[ROLL_LEVEL_PID].iLim             =   0.5f;  // radians/sec

        cfg.pids[PITCH_LEVEL_PID].p               =   2.0f;
        cfg.pids[PITCH_LEVEL_PID].i               =   0.0f;
        cfg.pids[PITCH_LEVEL_PID].d               =   0.0f;
        cfg.pids[PITCH_LEVEL_PID].iLim            =   0.5f;  // radians/sec

        cfg.pids[HEADING_PID].p                 =   1.5f;
        cfg.pids[HEADING_PID].i                 =   0.0f;
        cfg.pids[HEADING_PID].d                 =   0.0f;
        cfg.pids[HEADING_PID].iLim              =   0.5f;  // radians/sec
        
        cfg.pids[ALTITUDE_PID].p                 =   20.0f;
        cfg.pids[ALTITUDE_PID].i                 =   17.0f;
        cfg.pids[ALTITUDE_PID].d                 =   7.0f;
        cfg.pids[ALTITUDE_PID].iLim              =   30000.0f; // 0.1 m
        
        cfg.accelLPF            = true;
        cfg.accelLPF_Factor     = 0.90f;
        
        cfg.accelCalibrated                 = false;
        cfg.accelBias[XAXIS]                = 0.0f;
        cfg.accelBias[YAXIS]                = 0.0f;
        cfg.accelBias[ZAXIS]                = 0.0f;

        cfg.gyroLPF                    = 42;
        cfg.gyroTCBiasSlope[ROLL]      = 0.0f;
        cfg.gyroTCBiasSlope[PITCH]     = 0.0f;
        cfg.gyroTCBiasSlope[YAW]       = 0.0f;
        cfg.gyroTCBiasIntercept[ROLL]  = 0.0f;
        cfg.gyroTCBiasIntercept[PITCH] = 0.0f;
        cfg.gyroTCBiasIntercept[YAW]   = 0.0f;

        cfg.magCalibrated              = false;
        cfg.magBias[ROLL]              = 0.0f;
        cfg.magBias[PITCH]             = 0.0f;
        cfg.magBias[YAW]               = 0.0f;

        // For Mahony AHRS

        cfg.magDriftCompensation       = true;

        // Get your magnetic decliniation from here : http://magnetic-declination.com/
        // For example, -6deg 37min, = -6.37 Japan, format is [sign]ddd.mm (degreesminutes)
        cfg.magDeclination             = 10.59f; 
        
        cfg.twoKp                      = 1.0f;

        cfg.twoKi                      = 0.0f;

        cfg.battery                    = false;
        cfg.batScale                   = 11.0f;
        cfg.batMinCellVoltage          = 3.3f;
        cfg.batMaxCellVoltage          = 4.2f;

        writeParams();
	}
}

///////////////////////////////////////////////////////////////////////////////
