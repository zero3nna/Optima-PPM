/*
 * From Timecop's Original Baseflight
 *
 * Modified for Baseflight U.P. support - Scott Driessens (August 2012)
 */

#include "board.h"
#include "core/command.h"

#define FLASH_PAGE_COUNT 128

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)

// use the last KB for config storage
#define FLASH_WRITE_ADDR  (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))

config_t cfg;

int16_t lookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];   // lookup table for expo & mid THROTTLE

const char rcChannelLetters[] = "AERT1234";

static uint8_t EEPROM_CONF_VERSION = 1;
static void resetConf(void);

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

static uint8_t validEEPROM(void)
{
    const config_t *temp = (const config_t *)FLASH_WRITE_ADDR;
    const uint8_t *p;
    uint8_t chk = 0;
    
    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return 0;
        
    // check size and magic numbers
    if (temp->size != sizeof(config_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return 0;
    
    for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(config_t)); ++p)
        chk ^= *p;
    
    // checksum failed
    if (chk != 0)
        return 0;
    
    // looks good, let's roll!
    return 1;
}

void readEEPROM(void)
{
    uint8_t i;
    // Read flash
    memcpy(&cfg, (char *)FLASH_WRITE_ADDR, sizeof(config_t));
    
    for (i = 0; i < 6; i++)
        lookupPitchRollRC[i] = (2500 + cfg.commandExpo * (i * i - 25)) * i * (int32_t) cfg.commandRate / 2500;

    for (i = 0; i < 11; i++) {
        int16_t tmp = 10 * i - cfg.throttleMid;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - cfg.throttleMid;
        if (tmp < 0)
            y = cfg.throttleMid;
        lookupThrottleRC[i] = 10 * cfg.throttleMid + tmp * (100 - cfg.throttleExpo + (int32_t) cfg.throttleExpo * (tmp * tmp) / (y * y)) / 10;      // [0;1000]
        lookupThrottleRC[i] = cfg.minThrottle + (int32_t) (cfg.maxThrottle - cfg.minThrottle) * lookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }
}

void writeParams(void)
{
    FLASH_Status status;
    uint32_t i;
    uint8_t chk = 0;
    const uint8_t *p;
    
    cfg.version = EEPROM_CONF_VERSION;
    cfg.size = sizeof(config_t);
    cfg.magic_be = 0xBE;
    cfg.magic_ef = 0xEF;
    cfg.chk = 0;
    // recalculate checksum before writing
    for (p = (const uint8_t *)&cfg; p < ((const uint8_t *)&cfg + sizeof(config_t)); ++p)
        chk ^= *p;
    cfg.chk = chk;
    
    // write it
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
        for (i = 0; i < sizeof(config_t); i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)((char *)&cfg + i));
            if (status != FLASH_COMPLETE)
                while(1);//break; // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
}

void checkFirstTime(bool reset)
{
    // check the EEPROM integrity before resetting the values
    if (!validEEPROM() || reset)
        resetConf();
}

// Default settings
static void resetConf(void)
{
    int32_t i;
    memset(&cfg, 0, sizeof(config_t));
    
    cfg.version = EEPROM_CONF_VERSION;

	// Default settings
    cfg.version = EEPROM_CONF_VERSION;
    cfg.mixerConfiguration                 = MULTITYPE_QUADX;
    
    featureClearAll();
    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_PPM);
    
    parseRcChannels("AETR1234");
    
    // Motor/ESC
    cfg.escPwmRate                     = 400;
    cfg.servoPwmRate                   = 50;
    
    // Failsafe
    cfg.failsafeOnDelay                = 50; // Number of command loops (50Hz) until failsafe kicks in
    cfg.failsafeOffDelay               = 20000; // Number of command loops until failsafe stops
    cfg.failsafeThrottle               = 1200;
    
    cfg.commandRate             = 90;
    cfg.commandExpo             = 65;
    cfg.rollPitchRate           = 0;
    cfg.yawRate                 = 0;
    //cfg.dynThrPID             = 0;
    cfg.throttleMid             = 50;
    cfg.throttleExpo            = 0;
    
    // Command Settings
    for(i = 0; i < AUX_OPTIONS; ++i)
        cfg.auxActivate[i] = 0;

    cfg.minCommand                         = 1000;
    cfg.midCommand                         = 1500;
    cfg.maxCommand                         = 2000;
    cfg.minCheck                           = 1100;
    cfg.maxCheck                           = 1900;
    cfg.minThrottle                        = 1150;
    cfg.maxThrottle                        = 1850;
    cfg.spektrumHiRes                      = false;
    
    cfg.deadBand[ROLL]                     = 12;
    cfg.deadBand[PITCH]                    = 12;
    cfg.deadBand[YAW]                      = 12;
    
    // Servos
    
    // Tricopter
    cfg.yawDirection                       = 1;
    cfg.triYawServoMin                     = 1000;
    cfg.triYawServoMid                     = 1500;
    cfg.triYawServoMax                     = 2000;
    
    // Bicopter
    cfg.biLeftServoMin                     = 1000;
    cfg.biLeftServoMid                     = 1500;
    cfg.biLeftServoMax                     = 2000;

    cfg.biRightServoMin                    = 1000;
    cfg.biRightServoMid                    = 1500;
    cfg.biRightServoMax                    = 2000;
    
    // Flying wing
    cfg.wingLeftMin             = 1020;
    cfg.wingLeftMid             = 1500;
    cfg.wingLeftMax             = 2000;
    cfg.wingRightMin            = 1020;
    cfg.wingRightMid            = 1500;
    cfg.wingRightMax            = 2000;
    cfg.pitchDirectionLeft      = 1;
    cfg.pitchDirectionRight     = -1;
    cfg.rollDirectionLeft       = 1;
    cfg.rollDirectionRight      = 1;
    
    cfg.gimbalFlags = GIMBAL_NORMAL;
    cfg.gimbalSmoothFactor = 0.95f;

    cfg.gimbalRollServoMin                 = 1000;
	cfg.gimbalRollServoMid                 = 1500;
	cfg.gimbalRollServoMax                 = 2000;
	cfg.gimbalRollServoGain                = 1.0f;

	cfg.gimbalPitchServoMin                = 1000;
	cfg.gimbalPitchServoMid                = 1500;
	cfg.gimbalPitchServoMax                = 2000;
	cfg.gimbalPitchServoGain               = 1.0f;

    // PIDs
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
    
    cfg.angleTrim[ROLL]         = 0.0f;
    cfg.angleTrim[PITCH]        = 0.0f;
    
    cfg.accelLPF                = false;
    cfg.accelSmoothFactor       = 0.75f;
    
    cfg.accelCalibrated                 = false;
    cfg.accelBias[XAXIS]                = 0;
    cfg.accelBias[YAXIS]                = 0;
    cfg.accelBias[ZAXIS]                = 0;

    cfg.gyroBiasOnStartup           = false;
    cfg.gyroSmoothFactor            = 0.95f;
    cfg.gyroTCBiasSlope[ROLL]       = 0.0f;
    cfg.gyroTCBiasSlope[PITCH]      = 0.0f;
    cfg.gyroTCBiasSlope[YAW]        = 0.0f;
    cfg.gyroTCBiasIntercept[ROLL]   = 0.0f;
    cfg.gyroTCBiasIntercept[PITCH]  = 0.0f;
    cfg.gyroTCBiasIntercept[YAW]    = 0.0f;

    cfg.magCalibrated               = false;
    cfg.magBias[ROLL]               = 0;
    cfg.magBias[PITCH]              = 0;
    cfg.magBias[YAW]                = 0;
    
    cfg.mpu6050Scale                = false; // Shitty hack

    // For Mahony AHRS
    
    cfg.accelKp                     = 2.0f;
    cfg.accelKi                     = 0.01f;
    
    cfg.magKp                       = 1.0f;
    cfg.magKi                       = 0.01f;
    
    cfg.magDriftCompensation        = false;

    // Get your magnetic decliniation from here : http://magnetic-declination.com/
    // For example, -6deg 37min, = -6.37 Japan, format is [sign]ddd.mm (degreesminutes)
    cfg.magDeclination              = 10.59f; 

    cfg.batScale                    = 11.0f;
    cfg.batMinCellVoltage           = 3.3f;
    cfg.batMaxCellVoltage           = 4.2f;
    
    cfg.startupDelay                = 1000;
    
    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_MOTORS; i++)
        cfg.customMixer[i].throttle = 0.0f;

    writeParams();
}

bool featureGet(uint32_t mask)
{
    return cfg.enabledFeatures & mask;
}

void featureSet(uint32_t mask)
{
    cfg.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    cfg.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    cfg.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return cfg.enabledFeatures;
}
