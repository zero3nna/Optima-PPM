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
#include "core/serial.h"

#include "drivers/adc.h"
#include "drivers/i2c.h"
#include "drivers/pwm_ppm.h"
#include "drivers/spektrum.h"

///////////////////////////////////////////////////////////////////////////////

uint32_t cycleTime;

///////////////////////////////////////////////////////////////////////////////

void highSpeedTelemetry(void);
void updateActuators(void);
void statusLED(void);

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
    drv_pwm_config_t pwm_params;
    
    systemInit();
    
    adcInit();
    i2cInit(I2C2);
    uartInit(115200);
    
    sensorsInit();
    
    mixerInit(); // Must be called before pwmInit
    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        pwm_params.airplane = true;
    pwm_params.usePPM = featureGet(FEATURE_PPM);
    pwm_params.enableInput = true;//!feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServo;
    pwm_params.extraServos = cfg.gimbalFlags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = cfg.escPwmRate;
    pwm_params.servoPwmRate = cfg.servoPwmRate;
    
    pwmInit(&pwm_params);
    
    if(featureGet(FEATURE_SPEKTRUM)) {
        readRawRC = spektrumReadRawRC;
        spektrumInit();
    } else {
        readRawRC = pwmReadRawRC;
    }

    initPIDs();
    
#ifdef THESIS
    uart2Init(9600, currentDataReceive, true);
#endif
    
    delay(cfg.startupDelay);               // 2 sec delay for sensor stabilisation - probably not long enough.....
    
    if(cfg.gyroBiasOnStartup)
        computeGyroRTBias();
    
    periodicEvent(gyroSample, 500);
    
    if(sensorsAvailable & SENSOR_ACC)
        periodicEvent(accelSample, 500);
    
    if(sensorsAvailable & SENSOR_MAG)
        periodicEvent(magSample, 20000);
    
    periodicEvent(updateAttitude, 3000);
    periodicEvent(updateActuators, 4000);
    periodicEvent(updateCommands, 20000);
    periodicEvent(updateAltitude, 40000);
    periodicEvent(serialCom, 20000);
    periodicEvent(statusLED, 100000);
    
    if(featureGet(FEATURE_VBAT))
        periodicEvent(batterySample, 40000);

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
