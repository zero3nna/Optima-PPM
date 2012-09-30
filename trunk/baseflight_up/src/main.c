/*
    BaseflightPlus U.P
    Copyright (C) 2012 Scott Driessens

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "board.h"

#include "actuator/mixer.h"
#include "actuator/pid.h"
#include "actuator/stabilisation.h"

#include "core/command.h"
#include "core/serial.h"

#include "drivers/adc.h"
#include "drivers/i2c.h"
#include "drivers/pwm_ppm.h"
#include "drivers/spektrum.h"

uint32_t cycleTime;

void highSpeedTelemetry(void);
void updateActuators(void);
void statusLED(void);

int main(void)
{
    drv_pwm_config_t pwm_params;
    
    systemInit();
    
    checkFirstTime(false);
    readEEPROM();
    
    adcInit();
    i2cInit(I2C2);
    uartInit(115200);
    
    sensorsInit();
    
    mixerInit(); // Must be called before pwmInit
    
    if(featureGet(FEATURE_SPEKTRUM)) {
        readRawRC = spektrumReadRawRC;
        spektrumInit();
    } else {
        // spektrum and GPS are mutually exclusive
        // Optional GPS - available in both PPM and PWM input mode, in PWM input, reduces number of available channels by 2.
        // TODO
        //if (feature(FEATURE_GPS))
        //    gpsInit(cfg.gps_baudrate);
        readRawRC = pwmReadRawRC;
    }
    
    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    else
        pwm_params.airplane = false;
    pwm_params.usePPM = featureGet(FEATURE_PPM);
    pwm_params.useUART = false; featureGet(FEATURE_PPM);
    pwm_params.enableInput = !featureGet(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
    pwm_params.extraServos = cfg.gimbalFlags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = cfg.escPwmRate;
    pwm_params.servoPwmRate = cfg.servoPwmRate;
    
    pwmInit(&pwm_params);

    initPIDs();
    
#ifdef THESIS
    uart2Init(9600, currentDataReceive, true);
#endif
    
    delay(cfg.startupDelay);               // 2 sec delay for sensor stabilisation - probably not long enough.....
    if(cfg.gyroBiasOnStartup)
        computeGyroRTBias();
    
    periodicEvent(gyroSample, 500);
    if(sensorsGet(SENSOR_ACC))
        periodicEvent(accelSample, 2000);
    if(sensorsGet(SENSOR_MAG))
        periodicEvent(magSample, 20000);
    periodicEvent(updateAttitude, 3000);
    periodicEvent(updateActuators, 4000);
    periodicEvent(updateCommands, 20000);
    if(sensorsGet(SENSOR_BARO) || sensorsGet(SENSOR_SONAR))
        periodicEvent(updateAltitude, 40000);
    periodicEvent(serialCom, 20000);
    periodicEvent(statusLED, 100000);
    periodicEvent(computeGyroTCBias, 1000000);
    if(featureGet(FEATURE_VBAT))
        periodicEvent(batterySample, 40000);
        
    stateData.q[0] = 1.0f;
    stateData.q[1] = 0.0f;
    stateData.q[2] = 0.0f;
    stateData.q[3] = 0.0f;

    while (1)
    {
        eventCallbacks();
    }
    
    return 0;
}



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


