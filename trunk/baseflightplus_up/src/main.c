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

///////////////////////////////////////////////////////////////////////////////

uint32_t cycleTime;

///////////////////////////////////////////////////////////////////////////////

void highSpeedTelemetry(void);
void updateActuators(void);
void statusLED(void);

///////////////////////////////////////////////////////////////////////////////

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
    pwm_params.usePPM = cfg.usePPM;
    pwm_params.enableInput = true;//!feature(FEATURE_SPEKTRUM); // disable inputs if using spektrum
    pwm_params.useServos = useServos;
    pwm_params.extraServos = false;//cfg.gimbal_flags & GIMBAL_FORWARDAUX;
    pwm_params.motorPwmRate = cfg.escPwmRate;
    pwm_params.servoPwmRate = cfg.servoPwmRate;
    
    pwmInit(&pwm_params);

    initPIDs();
    
#ifdef THESIS
    uart2Init(9600, currentDataReceive);
#endif
    
    delay(1000);               // 1 sec delay for sensor stabilization - probably not long enough.....
    
    periodicEvent(gyroSample, 500);
    periodicEvent(accelSample, 500);
    
    periodicEvent(updateAttitude, 3000);
    
    periodicEvent(updateActuators, 4000);
    periodicEvent(updateCommands, 20000);
    if(cfg.magDriftCompensation)
        periodicEvent(magSample, 20000);
    periodicEvent(updateAltitude, 40000);
    periodicEvent(serialCom, 20000);
    periodicEvent(statusLED, 100000);
    if(cfg.battery)
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
