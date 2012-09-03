/*
 * From Timecop's Original Baseflight
 */

#include "board.h"

#include "actuator/mixer.h"

#include "core/cli.h"
#include "core/printf_min.h"

#include "drivers/i2c.h"

// Multiwii Serial Protocol 0 
#define VERSION                  210
#define MSP_VERSION              0
#define PLATFORM_32BIT           0x80000000

#define MSP_IDENT                100    //out message         multitype + version
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         1 altitude
#define MSP_BAT                  110    //out message         vbat, powermetersum
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113    //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114    //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203    //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_WP_SET               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

#define INBUF_SIZE 64

static const char boxnames[] =
    "LEVEL;"
    "ALTITUDE;"
    "HEADING;"
    "CAMSTAB;"
    "CAMTRIG;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "PASSTHRU;"
    "HEADFREE;"
    "BEEPER;"
    "LEDMAX;"
    "LLIGHTS;"
    "HEADADJ;";

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

static uint8_t checksum, indRX, inBuf[INBUF_SIZE];
static uint8_t cmdMSP;
static bool guiConnected = false;

void serialize32(uint32_t a)
{
    static uint8_t t;
    t = a;
    uartWrite(t);
    checksum ^= t;
    t = a >> 8;
    uartWrite(t);
    checksum ^= t;
    t = a >> 16;
    uartWrite(t);
    checksum ^= t;
    t = a >> 24;
    uartWrite(t);
    checksum ^= t;
}

void serialize16(int16_t a)
{
    static uint8_t t;
    t = a;
    uartWrite(t);
    checksum ^= t;
    t = a >> 8 & 0xff;
    uartWrite(t);
    checksum ^= t;
}

void serialize8(uint8_t a)
{
    uartWrite(a);
    checksum ^= a;
}

uint8_t read8(void)
{
    return inBuf[indRX++] & 0xff;
}

uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t) read8() << 8;
    return t;
}

uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t) read16() << 16;
    return t;
}

void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(cmdMSP);
}

void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void tailSerialReply(void)
{
    serialize8(checksum);
}

void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
}

void serialInit(uint32_t baudrate)
{
    uartInit(baudrate);
}

static void evaluateCommand(void)
{
    uint32_t i;
    uint8_t wp_no;

    switch (cmdMSP) {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcData[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RAW_GPS:
        read8();
        read8();
        read32();
        read32();
        read16();
        read16();
        
        //f.GPS_FIX = read8();
        //GPS_numSat = read8();
        //GPS_coord[LAT] = read32();
        //GPS_coord[LON] = read32();
        //GPS_altitude = read16();
        //GPS_speed = read16();
        //GPS_update |= 2;        // New data signalisation to GPS functions
        headSerialReply(0);
        break;
    case MSP_SET_PID:
        for(i = 0; i < 3; ++i) {
            cfg.pids[i].p = read8();
            //cfg.pids[i].i = read8() / 1000.0f;
            read8();
            cfg.pids[i].d = read8();
        }
        
        cfg.pids[ALTITUDE_PID].p = read8();
        //cfg.pids[ALTITUDE_PID].i = read8() / 1000.0f;
        read8();
        cfg.pids[ALTITUDE_PID].d = read8();
        
        // POS, POSR, NAVR
        for(i = 0; i < 3; ++i) {
            read8();
            read8();
            read8();
        }
        cfg.pids[ROLL_LEVEL_PID].p = read8();
        //cfg.pids[ROLL_LEVEL_PID].i = read8() / 1000.0f;
        read8();
        cfg.pids[ROLL_LEVEL_PID].d = read8();
        
        cfg.pids[PITCH_LEVEL_PID].p = cfg.pids[ROLL_LEVEL_PID].p;
        //cfg.pids[PITCH_LEVEL_PID].i = cfg.pids[ROLL_LEVEL_PID].i;
        read8();
        cfg.pids[PITCH_LEVEL_PID].d = cfg.pids[ROLL_LEVEL_PID].d;
    
        cfg.pids[HEADING_PID].p = read8();
        //cfg.pids[HEADING_PID].i = read8() / 1000.0f;
        read8();
        cfg.pids[HEADING_PID].d = read8();
    
        // Velocity
        for(i = 0; i < 3; ++i)
            read8();
            
        initPIDs();
        headSerialReply(0);
        break;
    case MSP_SET_BOX:
        for (i = 0; i < AUX_OPTIONS; ++i)
            cfg.auxActivate[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RC_TUNING:
        read8();
        read8();
        read8();
        read8();
        read8();
        read8();
        read8();
        /*
        cfg.rcRate8 = read8();
        cfg.rcExpo8 = read8();
        cfg.rollPitchRate = read8();
        cfg.yawRate = read8();
        cfg.dynThrPID = read8();
        cfg.thrMid8 = read8();
        cfg.thrExpo8 = read8(); */
        headSerialReply(0);
        break;
    case MSP_SET_MISC:
        headSerialReply(0);
        break;
    case MSP_IDENT:
        headSerialReply(7);
        serialize8(VERSION);                // multiwii version
        serialize8(cfg.mixerConfiguration); // type of multicopter
        serialize8(MSP_VERSION);            // MultiWii Serial Protocol Version
        serialize32(PLATFORM_32BIT);        // "capability"
        break;
    case MSP_STATUS:
        headSerialReply(10);
        serialize16(cycleTime);
        serialize16(i2cGetErrorCounter());
        serialize16(sensorsAvailable);
        serialize32(mode.LEVEL_MODE << OPT_LEVEL | mode.ALTITUDE_MODE << OPT_ALTITUDE | mode.HEADING_MODE << OPT_HEADING | mode.ARMED << OPT_ARM | auxOptions[OPT_CAMSTAB] << OPT_CAMSTAB | auxOptions[OPT_CAMTRIG] << OPT_CAMTRIG | 
                    mode.GPS_HOME_MODE << OPT_GPSHOME | mode.GPS_HOLD_MODE << OPT_GPSHOLD | mode.HEADFREE_MODE << OPT_HEADFREE | mode.PASSTHRU_MODE << OPT_PASSTHRU | 
                    auxOptions[OPT_BEEPERON] << OPT_BEEPERON | auxOptions[OPT_LEDMAX] << OPT_LEDMAX | auxOptions[OPT_LANDING_LIGHTS] << OPT_LANDING_LIGHTS | auxOptions[OPT_HEADFREE_REF] << OPT_HEADFREE_REF);
        break;
    case MSP_RAW_IMU:
        headSerialReply(18);
        for (i = 0; i < 3; i++)
            serialize16(sensors.accel[i] * 10.0f);
        for (i = 0; i < 3; i++)
            serialize16(sensors.gyro[i] * 100.0f);
        for (i = 0; i < 3; i++)
            serialize16(sensors.mag[i]);
        break;
    case MSP_SERVO:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(servo[i]);
        break;
    case MSP_MOTOR:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(motor[i]);
        break;
    case MSP_RC:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(rcData[i]);
        break;
    case MSP_RAW_GPS:
        headSerialReply(0);
        /*headSerialReply(14);
        serialize8(f.GPS_FIX);
        serialize8(GPS_numSat);
        serialize32(GPS_coord[LAT]);
        serialize32(GPS_coord[LON]);
        serialize16(GPS_altitude);
        serialize16(GPS_speed);*/
        break;
    case MSP_COMP_GPS:
        headSerialReply(0);
        /*
        headSerialReply(5);
        serialize16(GPS_distanceToHome);
        serialize16(GPS_directionToHome);
        serialize8(GPS_update & 1); */
        break;
    case MSP_ATTITUDE:
        headSerialReply(8);
        serialize16(sensors.attitude[ROLL] * RAD2DEG * 10);
        serialize16(sensors.attitude[PITCH] * RAD2DEG * 10);
        serialize16(sensors.attitude[YAW] * RAD2DEG);
        serialize16(headfreeReference);
        break;
    case MSP_ALTITUDE:
        headSerialReply(4);
        serialize32(sensors.altitude);
        break;
    case MSP_BAT:
        headSerialReply(3);
        serialize8(sensors.batteryVoltage * 10);
        serialize16(0); // power meter trash
        break;
    case MSP_RC_TUNING:
    /*
        headSerialReply(7);
        serialize8(cfg.rcRate8);
        serialize8(cfg.rcExpo8);
        serialize8(cfg.rollPitchRate);
        serialize8(cfg.yawRate);
        serialize8(cfg.dynThrPID);
        serialize8(cfg.thrMid8);
        serialize8(cfg.thrExpo8); */
        headSerialReply(0);
        break;
    case MSP_PID:
        headSerialReply(30);
        for(i = 0; i < 3; ++i) {
            serialize8(pids[i].p);
            serialize8(pids[i].i * 1000.0f);
            serialize8(pids[i].d);
        }
        
        serialize8(pids[ALTITUDE_PID].p);
        serialize8(pids[ALTITUDE_PID].i * 1000.0f);
        serialize8(pids[ALTITUDE_PID].d);
        
        // POS, POSR, NAVR
        for(i = 0; i < 3; ++i) {
            serialize8(0);
            serialize8(0);
            serialize8(0);
        }
        serialize8(pids[ROLL_LEVEL_PID].p);
        serialize8(pids[ROLL_LEVEL_PID].i * 1000.0f);
        serialize8(pids[ROLL_LEVEL_PID].d);
        
        serialize8(pids[HEADING_PID].p);
        serialize8(pids[HEADING_PID].i * 1000.0f);
        serialize8(pids[HEADING_PID].d);
        
        // Velocity
        serialize8(0);
        serialize8(0);
        serialize8(0);
        break;
    case MSP_BOX:
        headSerialReply(2 * AUX_OPTIONS);
        for (i = 0; i < AUX_OPTIONS; i++)
            serialize16(cfg.auxActivate[i]);
        break;
    case MSP_BOXNAMES:
        headSerialReply(sizeof(boxnames) - 1);
        serializeNames(boxnames);
        break;
    case MSP_PIDNAMES:
        headSerialReply(sizeof(pidnames) - 1);
        serializeNames(pidnames);
        break;
    case MSP_MISC:
        headSerialReply(2);
        serialize16(0); // intPowerTrigger1
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(8);
        for (i = 0; i < 8; i++)
            serialize8(i + 1);
        break;
    case MSP_WP:
        wp_no = read8();    // get the wp number
        headSerialReply(0);
        /*
        headSerialReply(12);
        if (wp_no == 0) {
            serialize8(0);                   // wp0
            serialize32(GPS_home[LAT]);
            serialize32(GPS_home[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        } else if (wp_no == 16) {
            serialize8(16);                  // wp16
            serialize32(GPS_hold[LAT]);
            serialize32(GPS_hold[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        }
        */
        break;
    case MSP_RESET_CONF:
        headSerialReply(0);
        checkFirstTime(true);
        systemReset(false);
        break;
    case MSP_ACC_CALIBRATION:
        accelCalibration();
        computeGyroRTBias();
        headSerialReply(0);
        break;
    case MSP_MAG_CALIBRATION:
        magCalibration();
        headSerialReply(0);
        break;
    case MSP_EEPROM_WRITE:
        writeParams();
        headSerialReply(0);
        break;
    case MSP_DEBUG:
        headSerialReply(0);
        /*headSerialReply(8);
        for (i = 0; i < 4; i++)
            serialize16(debug[i]);      // 4 variables are here for general monitoring purpose
            */
        break;
    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        headSerialError(0);
        break;
    }
    tailSerialReply();
}

#ifdef THESIS

static uint8_t highSpeedTelemetry = false;

#define CURRENT_SCALE_FACTOR    5.0f/(1023.0f * 0.133f)

static float current; // A

// UART2 Receive ISR callback
void currentDataReceive(uint16_t c)
{
    static char data[5];
    static uint8_t index;
    
    if(c == '\n' && index) {
        data[index] = '\0';
        current = (float)atoi(data);
        current = (current - 102.3) * CURRENT_SCALE_FACTOR;
        index = 0;
    } else if(index < 5){
        data[index++] = (uint8_t)c;
    } else {
        index = 0;
    }
}

static void highSpeedTelemetryCallback(void)
{
    printf_min("%0.2f,%0.2f,%0.2f,%d,%0.2f,%0.2f\n", 
            sensors.attitude[ROLL] * RAD2DEG, sensors.attitude[PITCH] * RAD2DEG, sensors.attitude[YAW] * RAD2DEG, 
            sensors.altitude, sensors.batteryVoltage, current);
    
    if(highSpeedTelemetry)
        singleEvent(highSpeedTelemetryCallback, 5000);
}

#endif

// evaluate all other incoming serial data
static void evaluateOtherData(uint8_t sr)
{
    switch (sr) {
#ifdef THESIS
        case '~':
            highSpeedTelemetry = !highSpeedTelemetry;
            singleEvent(highSpeedTelemetryCallback, 5000);
            break;
#endif
        case '#':
            cliProcess();
            break;
        case 'R':
            systemReset(true);      // reboot to bootloader
            break;
    }
}

void serialCom(void)
{
    uint8_t c;
    static uint8_t offset;
    static uint8_t dataSize;
    static enum _serial_state {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD,
    } c_state = IDLE;

    // in cli mode, all uart stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

    while (uartAvailable()) {
        c = uartRead();

        if (c_state == IDLE) {
            c_state = (c == '$') ? HEADER_START : IDLE;
            if (c_state == IDLE)
                evaluateOtherData(c); // evaluate all other incoming serial data
        } else if (c_state == HEADER_START) {
            c_state = (c == 'M') ? HEADER_M : IDLE;
        } else if (c_state == HEADER_M) {
            c_state = (c == '<') ? HEADER_ARROW : IDLE;
        } else if (c_state == HEADER_ARROW) {
            if (c > INBUF_SIZE) {       // now we are expecting the payload size
                c_state = IDLE;
                continue;
            }
            dataSize = c;
            offset = 0;
            checksum = 0;
            indRX = 0;
            checksum ^= c;
            c_state = HEADER_SIZE;      // the command is to follow
            guiConnected = true;
        } else if (c_state == HEADER_SIZE) {
            cmdMSP = c;
            checksum ^= c;
            c_state = HEADER_CMD;
        } else if (c_state == HEADER_CMD && offset < dataSize) {
            checksum ^= c;
            inBuf[offset++] = c;
        } else if (c_state == HEADER_CMD && offset >= dataSize) {
            if (checksum == c) {        // compare calculated and transferred checksum
                evaluateCommand();      // we got a valid packet, evaluate it
            }
            c_state = IDLE;
        }
    }
}