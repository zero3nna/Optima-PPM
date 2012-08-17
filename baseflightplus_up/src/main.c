/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"

#include "actuator/mixer.h"
#include "actuator/pid.h"
#include "actuator/stabilisation.h"

#include "core/command.h"
#include "core/mavlink.h"
#include "core/serial.h"

///////////////////////////////////////
// Timing Defines
///////////////////////////////////////

#define COUNT_1000HZ    1000
#define COUNT_500HZ     2000
#define COUNT_300HZ     3333
#define COUNT_250HZ     4000
#define COUNT_200HZ     5000
#define COUNT_100HZ     10000
#define COUNT_75HZ      13333
#define COUNT_50HZ      20000
#define COUNT_25HZ      40000
#define COUNT_10HZ      100000
#define COUNT_5HZ       200000
#define COUNT_1HZ       1000000

///////////////////////////////////////////////////////////////////////////////

uint32_t cycleTime;

///////////////////////////////////////////////////////////////////////////////

void highSpeedTelemetry(void);
void updateActuators(void);
void statusLED(void);

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    systemInit();
    sensorsInit();

    initPIDs();
    //mavlinkInit();
    
    // Called every time -> oversampling
    periodicEvent(gyroSample, 0);
    periodicEvent(accelSample, 0);
    
    periodicEvent(updateAttitude, COUNT_300HZ);
    periodicEvent(updateActuators, COUNT_250HZ);
    periodicEvent(updateCommands, COUNT_50HZ);
    if(cfg.magDriftCompensation)
        periodicEvent(magSample, COUNT_75HZ);
    periodicEvent(updateAltitude, COUNT_25HZ);
    periodicEvent(serialCom, COUNT_50HZ);
    periodicEvent(statusLED, COUNT_10HZ);
    if(cfg.battery)
        periodicEvent(batterySample, COUNT_25HZ);

    while (1)
    {
        eventCallbacks();
    }
    
    return 0;
}

///////////////////////////////////////////////////////////////////////////////

void statusLED(void)
{
    if(!cfg.accelCalibrated){
        LED1_TOGGLE();
    } else if (mode.LEVEL_MODE) {
        LED1_ON();
    } else {
        LED1_OFF();
    }
    
    if(mode.ARMED) {
        LED0_ON();
    } else {
        LED0_OFF();
    }
}

///////////////////////////////////////////////////////////////////////////////

void updateActuators(void)
{
    static uint32_t last;
    uint32_t now = micros();
    cycleTime = now - last;
    last = now;
    
    stabilisation();
    mixTable();
    writeServos();
    writeMotors(); 
}

///////////////////////////////////////////////////////////////////////////////
