/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#include "board.h"
#include "core/mavlink.h"
#include "core/command.h"

///////////////////////////////////////
// Timing Defines
///////////////////////////////////////

#define COUNT_1000HZ    1000
#define COUNT_500HZ     2000
#define COUNT_300HZ     3333
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
    
    periodicEvent(updateAttitude, COUNT_300HZ);
    periodicEvent(updateActuators, COUNT_200HZ);
    periodicEvent(updateCommands, COUNT_50HZ);
    if(sensorConfig.magDriftCompensation)
        periodicEvent(magSample, COUNT_75HZ);
    periodicEvent(baroSample, COUNT_50HZ);
    periodicEvent(updateAltitude, COUNT_10HZ);
    periodicEvent(serialCom, COUNT_50HZ);
    periodicEvent(statusLED, COUNT_10HZ);
    if(sensorConfig.battery)
        periodicEvent(batterySample, COUNT_25HZ);

    while (1)
    {
        eventCallbacks();
        gyroSample();
        accelSample();
    }
    
    return 0;
}


void statusLED(void)
{
    if(!sensorConfig.accelCalibrated){
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

