/*
 * From Timecop's Original Baseflight
 *
 * Added Baseflight U.P. support - Scott Driessens (August 2012)
 */

#include "board.h"

#include "core/printf_min.h"

#include "drivers/i2c.h"

// we unset this on 'exit'
uint8_t cliMode;
static void cliDefaults(char *cmdline);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);
static void cliCalibrate(char *cmdline);
static void telemetryOn(char *cmdline);

static void telemetry(void);
static void calibHelp(void);

///////////////////////////////////////////////////////////////////////////////

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

///////////////////////////////////////////////////////////////////////////////
// buffer
///////////////////////////////////////////////////////////////////////////////
static char cliBuffer[32];
static uint32_t bufferIndex = 0;
static uint8_t telemetryEnabled = false;

///////////////////////////////////////////////////////////////////////////////

// sync this with MultiType enum from mw.h
const char *mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6P",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char *featureNames[] = {
    "PPM", "VBAT", "MOTOR_STOP", "SERVO_TILT", "FAILSAFE", "SONAR",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char *sensorNames[] = {
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

// 
/*
const char *accNames[] = {
    "", "ADXL345", "MPU6050", NULL
};*/

///////////////////////////////////////////////////////////////////////////////

typedef struct {
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    { "calibrate", "sensor calibration", cliCalibrate },
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "exit", "", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank for list", cliSet },
    { "status", "show system status", cliStatus },
    { "telemetry", "", telemetryOn },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

///////////////////////////////////////////////////////////////////////////////
// CLI Calibration
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    char *name;
    char *param;
    void (*func)(void);
} calibcmd_t;

const calibcmd_t calibCmdTable[] = {
    { "accel", "Placed level and fixed", accelCalibration },
    { "gyro", "Temp compensation (15 min)", gyroTempCalibration},
    { "gyroRT", "Placed fixed", computeGyroRTBias} ,
    { "help", "", calibHelp }, 
    { "mag", "Rotate around all axis within 10 s", magCalibration },
};

#define CALIB_CMD_COUNT (sizeof(calibCmdTable) / sizeof(calibCmdTable[0]))

///////////////////////////////////////////////////////////////////////////////
// CLI Value
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "escPwmRate", VAR_UINT16, &cfg.escPwmRate, 50, 498},
    { "servoPwmRate", VAR_UINT16, &cfg.servoPwmRate, 50, 498},
    { "failsafeOnDelay",    VAR_UINT16, &cfg.failsafeOnDelay, 0, 1000 },
    { "dailsafeOffDelay",   VAR_UINT16, &cfg.failsafeOffDelay, 0, 100000 },
    { "failsafeThrottle",   VAR_UINT16, &cfg.failsafeThrottle, 1000, 2000 },
    { "minCommand", VAR_UINT16, &cfg.minCommand, 0, 2000 },
    { "midCommand", VAR_UINT16, &cfg.midCommand, 1200, 1700 },
    { "maxCommand", VAR_UINT16, &cfg.maxCommand, 0, 2000 },
    { "minCheck", VAR_UINT16, &cfg.minCheck, 0, 2000 },
    { "maxCheck", VAR_UINT16, &cfg.maxCheck, 0, 2000 },
    { "minThrottle", VAR_UINT16, &cfg.minThrottle, 0, 2000 },
    { "maxThrottle", VAR_UINT16, &cfg.maxThrottle, 0, 2000 },
    { "rollDeadband",   VAR_UINT8, &cfg.deadBand[ROLL], 0, 32 },
    { "pitchDeadband",  VAR_UINT8, &cfg.deadBand[PITCH], 0, 32 },
    { "yawDeadband",    VAR_UINT8, &cfg.deadBand[YAW], 0, 32 },
    { "biLeftServoMin", VAR_UINT16, &cfg.biLeftServoMin, 0, 2000 },
    { "biLeftServoMid", VAR_UINT16, &cfg.biLeftServoMid, 0, 2000 },
    { "biLeftServoMax", VAR_UINT16, &cfg.biLeftServoMax, 0, 2000 },
    { "biRightServoMin", VAR_UINT16, &cfg.biRightServoMin, 0, 2000 },
    { "biRightServoMid", VAR_UINT16, &cfg.biRightServoMid, 0, 2000 },
    { "biRightServoMax", VAR_UINT16, &cfg.biRightServoMax, 0, 2000 },
    { "yawDirection", VAR_INT8, &cfg.yawDirection, -1, 1 },
    { "triYawServoMin", VAR_UINT16, &cfg.triYawServoMin, 0, 2000 },
    { "triYawServoMid", VAR_UINT16, &cfg.triYawServoMid, 0, 2000 },
    { "triYawServoMax", VAR_UINT16, &cfg.triYawServoMax, 0, 2000 },
    { "gimbalRollServoMin", VAR_UINT16, &cfg.gimbalRollServoMin, 0, 2000 },
    { "gimbalRollServoMid", VAR_UINT16, &cfg.gimbalRollServoMid, 0, 2000 },
    { "gimbalRollServoMax", VAR_UINT16, &cfg.gimbalRollServoMax, 0, 2000 },
    { "gimbalRollServoGain", VAR_FLOAT, &cfg.gimbalRollServoGain, -100, 100 },
    { "gimbalPitchServoMin", VAR_UINT16, &cfg.gimbalPitchServoMin, 0, 2000 },
    { "gimbalPitchServoMid", VAR_UINT16, &cfg.gimbalPitchServoMid, 0, 2000 },
    { "gimbalPitchServoMax", VAR_UINT16, &cfg.gimbalPitchServoMax, 0, 2000 },
    { "gimbalPitchServoGain", VAR_FLOAT, &cfg.gimbalPitchServoGain, -100, 100 },
    { "rollDirectionLeft",      VAR_INT8, &cfg.rollDirectionLeft, -1, 1},
    { "rollDirectionRight",     VAR_INT8, &cfg.rollDirectionRight, -1, 1},
    { "pitchDirectionLeft",     VAR_INT8, &cfg.pitchDirectionLeft, -1, 1},
    { "pitchDirectionRight",    VAR_INT8, &cfg.pitchDirectionRight, -1, 1},
    { "wingLeftMinimum",    VAR_UINT16, &cfg.wingLeftMinimum, 0, 2000 },
    { "wingLeftMaximum",    VAR_UINT16, &cfg.wingLeftMaximum, 0, 2000 },
    { "wingRightMinimum",   VAR_UINT16, &cfg.wingRightMinimum, 0, 2000 },
    { "wingRightMaximum",   VAR_UINT16, &cfg.wingRightMaximum, 0, 2000 },
    { "p_roll_rate",        VAR_FLOAT, &cfg.pids[ROLL_RATE_PID].p,     0, 400 },
    { "i_roll_rate",        VAR_FLOAT, &cfg.pids[ROLL_RATE_PID].i,     0, 400 },
    { "d_roll_rate",        VAR_FLOAT, &cfg.pids[ROLL_RATE_PID].d,     0, 400 },
    { "ilim_roll_rate",     VAR_FLOAT, &cfg.pids[ROLL_RATE_PID].iLim,   0, 200},
    { "p_pitch_rate",       VAR_FLOAT, &cfg.pids[PITCH_RATE_PID].p,    0, 400 },
    { "i_pitch_rate",       VAR_FLOAT, &cfg.pids[PITCH_RATE_PID].i,    0, 400 },
    { "d_pitch_rate",       VAR_FLOAT, &cfg.pids[PITCH_RATE_PID].d,    0, 400 },
    { "ilim_pitch_rate",    VAR_FLOAT, &cfg.pids[PITCH_RATE_PID].iLim, 0, 200},
    { "p_yaw_rate",         VAR_FLOAT, &cfg.pids[YAW_RATE_PID].p,      0, 400 },
    { "i_yaw_rate",         VAR_FLOAT, &cfg.pids[YAW_RATE_PID].i,      0, 400 },
    { "d_yaw_rate",         VAR_FLOAT, &cfg.pids[YAW_RATE_PID].d,      0, 400 },
    { "ilim_yaw_rate",      VAR_FLOAT, &cfg.pids[YAW_RATE_PID].iLim,    0, 200},
    { "p_roll_level",       VAR_FLOAT, &cfg.pids[ROLL_LEVEL_PID].p,      0, 400 },
    { "i_roll_level",       VAR_FLOAT, &cfg.pids[ROLL_LEVEL_PID].i,      0, 400 },
    { "d_roll_level",       VAR_FLOAT, &cfg.pids[ROLL_LEVEL_PID].d,      0, 400 },
    { "ilim_roll_level",    VAR_FLOAT, &cfg.pids[ROLL_LEVEL_PID].iLim,    0, 200},
    { "p_pitch_level",      VAR_FLOAT, &cfg.pids[PITCH_LEVEL_PID].p,      0, 400 },
    { "i_pitch_level",      VAR_FLOAT, &cfg.pids[PITCH_LEVEL_PID].i,      0, 400 },
    { "d_pitch_level",      VAR_FLOAT, &cfg.pids[PITCH_LEVEL_PID].d,      0, 400 },
    { "ilim_pitch_level",   VAR_FLOAT, &cfg.pids[PITCH_LEVEL_PID].iLim,    0, 200},
    { "p_heading",     VAR_FLOAT, &cfg.pids[HEADING_PID].p,      0, 400 },
    { "i_heading",     VAR_FLOAT, &cfg.pids[HEADING_PID].i,      0, 400 },
    { "d_heading",     VAR_FLOAT, &cfg.pids[HEADING_PID].d,      0, 400 },
    { "ilim_heading",  VAR_FLOAT, &cfg.pids[HEADING_PID].iLim,    0, 200},
    { "p_altitude",     VAR_FLOAT, &cfg.pids[ALTITUDE_PID].p ,      0, 400 },
    { "i_altitude",     VAR_FLOAT, &cfg.pids[ALTITUDE_PID].i,      0, 400 },
    { "d_altitude",     VAR_FLOAT, &cfg.pids[ALTITUDE_PID].d,      0, 400 },
    { "ilim_altitude",  VAR_FLOAT, &cfg.pids[ALTITUDE_PID].iLim,    0, 50000},
    { "mpu6050Scale", VAR_UINT8, &cfg.mpu6050Scale, 0, 1 },   
    { "imuKp",  VAR_FLOAT, &cfg.imuKp,    0, 50},
    { "imuKi",  VAR_FLOAT, &cfg.imuKi,    0, 50},
    { "accelInfluenceCutoff",  VAR_FLOAT, &cfg.accelInfluenceCutoff,    0, 50},
    { "magDriftCompensation",  VAR_UINT8, &cfg.magDriftCompensation,    0, 1},
    { "magDeclination",  VAR_FLOAT, &cfg.magDeclination,    -18000, 18000},
    { "accelLPF", VAR_UINT8, &cfg.accelLPF, 0, 1},
    { "accelLPF_Factor",  VAR_FLOAT, &cfg.accelLPF_Factor,    -100, 100},
    { "gyroLPF", VAR_UINT16, &cfg.gyroLPF, 10, 256},
    { "batScale",  VAR_FLOAT, &cfg.batScale,    0, 50},
    { "batMinCellVoltage",  VAR_FLOAT, &cfg.batMinCellVoltage,    0, 5},
    { "batMaxCellVoltage",  VAR_FLOAT, &cfg.batMaxCellVoltage,    0, 5},
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

///////////////////////////////////////////////////////////////////////////////

static void cliSetVar(const clivalue_t *var, const char *value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

///////////////////////////////////////////////////////////////////////////////

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

///////////////////////////////////////////////////////////////////////////////

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

///////////////////////////////////////////////////////////////////////////////

static void cliDefaults(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    delay(10);
    systemReset(false);
}

///////////////////////////////////////////////////////////////////////////////

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode...\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
    // save and reboot... I think this makes the most sense
    cliSave(cmdline);
}

///////////////////////////////////////////////////////////////////////////////

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (len == 0) {
        uartPrint("Enabled features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                uartPrint((char *)featureNames[i]);
            uartWrite(' ');
        }
        uartPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            uartPrint((char *)featureNames[i]);
            uartWrite(' ');
        }
        uartPrint("\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                uartPrint("Invalid feature name...\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0) {
                if (remove) {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                } else {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                uartPrint((char *)featureNames[i]);
                uartPrint("\r\n");
                break;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    uartPrint("Available commands:\r\n");    

    for (i = 0; i < CMD_COUNT; i++) {
        uartPrint(cmdTable[i].name);
        uartWrite('\t');
        uartPrint(cmdTable[i].param);
        uartPrint("\r\n");
        while (!uartTransmitEmpty());
    }
}

///////////////////////////////////////////////////////////////////////////////

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        // uppercase it
        for (i = 0; i < 8; i++)
            cmdline[i] = toupper(cmdline[i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++)
        out[cfg.rcMap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    uartPrint(out);
    uartPrint("\r\n");
}

///////////////////////////////////////////////////////////////////////////////

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;
    
    len = strlen(cmdline);

    if (len == 0) {
        uartPrint("Current mixer: ");
        uartPrint((char *)mixerNames[cfg.mixerConfiguration - 1]);
        uartPrint("\r\n");
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            uartPrint((char *)mixerNames[i]);
            uartWrite(' ');
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            cfg.mixerConfiguration = i + 1;
            uartPrint("Mixer set to ");
            uartPrint((char *)mixerNames[i]);
            uartPrint("\r\n");
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

static void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams();
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);
}

///////////////////////////////////////////////////////////////////////////////

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    char buf[16];

    switch (var->type) {
        case VAR_UINT8:
            sprintf_min(buf, "%u", *(uint8_t *)var->ptr);
            break;
        
        case VAR_INT8:
            sprintf_min(buf, "%d", *(int8_t *)var->ptr);
            break;

        case VAR_UINT16:
            sprintf_min(buf, "%u", *(uint16_t *)var->ptr);
            break;

        case VAR_INT16:
            sprintf_min(buf, "%d", *(int16_t *)var->ptr);
            break;

        case VAR_UINT32:
            sprintf_min(buf, "%u", *(uint32_t *)var->ptr);
            break;
        case VAR_FLOAT:
            sprintf_min(buf, "%f", *(float *)var->ptr);
            break;
    }
    
    uartPrint(buf);

    if (full) {
        uartPrint(" ");
        itoa(var->min, buf, 10);
        uartPrint(buf);
        uartPrint(" ");
        itoa(var->max, buf, 10);
        uartPrint(buf);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Set
///////////////////////////////////////////////////////////////////////////////

static void cliSetVar(const clivalue_t *var, const char *eqptr)
{   
    int32_t value = 0;
    float fvalue = 0.0f;
    
    if(var->type == VAR_FLOAT) {
        fvalue = stringToFloat(eqptr);
    } else {
        value = atoi(eqptr);
    }
    
    if ((var->type != VAR_FLOAT && (value < var->min || value > var->max)) ||
        (var->type == VAR_FLOAT && (fvalue < var->min || fvalue > var->max))) {
        uartPrint("ERR: Value assignment out of range\r\n");
        return;
    }
    
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(uint8_t *)var->ptr = (uint8_t)value;
            break;
            
        case VAR_UINT16:
        case VAR_INT16:
            *(uint16_t *)var->ptr = (uint16_t)value;
            break;
            
        case VAR_UINT32:
            *(uint32_t *)var->ptr = (uint32_t)value;
            break;
            
        case VAR_FLOAT:
            *(float *)var->ptr = fvalue;
            break;
    }
    
    uartPrint((char *)var->name);
    uartPrint(" set to ");
    cliPrintVar(var, 0);
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            uartPrint((char *)valueTable[i].name);
            uartPrint(" = ");
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
            while (!uartTransmitEmpty());
        }
    } else if ((eqptr = strstr(cmdline, "="))) {
        // has equal, set var
        eqptr++;
        len--;
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                // found
                cliSetVar(val, eqptr);
                return;
            }
        }
        uartPrint("ERR: Unknown variable name\r\n");
    }
}

///////////////////////////////////////////////////////////////////////////////

static void cliStatus(char *cmdline)
{
    //uint8_t i;
    //uint32_t mask;

    printf_min("System Uptime: %u", millis() / 1000);
    uartPrint(" seconds, Voltage: ");
    printf_min("0.2f", sensors.batteryVoltage);
    printf_min(", %u", sensors.batteryCellCount);
    uartPrint("S battery)\r\n");

    //mask = sensorsMask();

    /*uartPrint("Detected sensors: ");
    for (i = 0; ; i++) {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            uartPrint((char *)sensorNames[i]);
        uartWrite(' ');
    }
    if (sensors(SENSOR_ACC)) {
        uartPrint("ACCHW: ");
        uartPrint((char *)accNames[accHardware]);
    }
    uartPrint("\r\n");*/

    printf_min("Cycle Time: %u", cycleTime);
    printf_min(", i2c Errors: %u", i2cGetErrorCounter());
    uartPrint("\r\n");
}

///////////////////////////////////////////////////////////////////////////////

static void calibHelp(void)
{
    uint8_t i;
    uartPrint("Available calibration commands: \r\n");
    for(i = 0; i < CALIB_CMD_COUNT ; ++i) {
        uartPrint(calibCmdTable[i].name);
        uartWrite('\t');
        uartPrint(calibCmdTable[i].param);
        uartPrint("\r\n");
        while (!uartTransmitEmpty());
    }
}

static void cliCalibrate(char *cmdline)
{
    uint8_t i;
    uint32_t len;
    
    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        ///////////////////////////////
        uartPrint("Sensor calibration data: \r\n");
        ///////////////////////////////
        printf_min("ACCEL BIAS:\r\n%f, %f, %f\r\n", cfg.accelBias[XAXIS], 
            cfg.accelBias[YAXIS], cfg.accelBias[ZAXIS]);
        ///////////////////////////////
        printf_min("GYRO TC BIAS:\r\n%f, %f, %f\r\n", cfg.gyroTCBiasSlope[ROLL], 
            cfg.gyroTCBiasSlope[PITCH], cfg.gyroTCBiasSlope[YAW]);
        printf_min("GYRO TC BIAS INTERCEPT:\r\n%f, %f, %f\r\n", cfg.gyroTCBiasIntercept[ROLL], 
            cfg.gyroTCBiasIntercept[PITCH], cfg.gyroTCBiasIntercept[YAW]);
        printf_min("GYRO RT BIAS:\r\n%f, %f, %f\r\n", sensors.gyroRTBias[ROLL], 
            sensors.gyroRTBias[PITCH], sensors.gyroRTBias[YAW]);
	    ///////////////////////////////
	    printf_min("MAG BIAS:\r\n%f, %f, %f\r\n", cfg.magBias[XAXIS], cfg.magBias[YAXIS],
            cfg.magBias[ZAXIS]);
        ///////////////////////////////
    } else {
        for(i = 0; i < CALIB_CMD_COUNT; ++i) {
            if (strncasecmp(cmdline, calibCmdTable[i].name, len) == 0) {
                calibCmdTable[i].func();
                break;
            }
        }
        if(i >= CALIB_CMD_COUNT) {
            uartPrint("Invalid calibration command\r\n");
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

static void cliVersion(char *cmdline)
{
    uartPrint("Baseflight U.P. CLI version 1.0 " __DATE__ " / " __TIME__);
}

///////////////////////////////////////////////////////////////////////////////

void cliProcess(void)
{
    if (!cliMode) {
        cliMode = 1;
        uartPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
        cliPrompt();
    }
    
    if(telemetryEnabled) {
        telemetry();
    }

    while (uartAvailable()) {
        uint8_t c = uartRead();
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex]) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                uartPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    uartPrint(cmd->name);
                    uartWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                uartWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {
            // clear screen
            uartPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            uartPrint("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate
            
            target.name = cliBuffer;
            target.param = NULL;
            
            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                uartPrint("ERR: Unknown command, try 'help'");

            memset(cliBuffer, 0, sizeof(cliBuffer));
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;
            cliPrompt();
        } else if (c == 127 || c == 8) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                uartPrint("\010 \010");
                //uartWrite(8);
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == 32)
                continue;
            cliBuffer[bufferIndex++] = c;
            uartWrite(c);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// AeroQuad style telemetry
///////////////////////////////////////////////////////////////////////////////

static void telemetry(void)
{
    static uint8_t query;
    
    if (uartAvailable()) query = uartRead();
    
    switch (query) {
        case '#':
            telemetryEnabled = false;
            cliPrompt();
            break;
        case 'a':
            printf_min("%f,%f,%f\n", sensors.accel[ROLL], sensors.accel[PITCH], sensors.accel[YAW]);
            break;
        case 'b':
            printf_min("%f\n", sensors.baroAltitude);
            break;
        case 'g':
            printf_min("%f,%f,%f\n", sensors.gyro[ROLL], sensors.gyro[PITCH], sensors.gyro[YAW]);
            break;
        case 'm':
            printf_min("%f,%f,%f\n", sensors.mag[ROLL], sensors.mag[PITCH], sensors.mag[YAW]);
            break;
        case 'q':
            printf_min("%f,%f,%f,%f\n", q[0], q[1], q[2], q[3]);
            break;
        case 't':
            printEventDeltas();
            break;
        case 'w':
            printf_min("%f,%f,%f\n", sensors.attitude[ROLL] * RAD2DEG, sensors.attitude[PITCH] * RAD2DEG, sensors.attitude[YAW] * RAD2DEG);
            break;
        case 'x':
        default:
            break;
    }
}

static void telemetryOn(char *cmdline)
{
    telemetryEnabled = true;
}