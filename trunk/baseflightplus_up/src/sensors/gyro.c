

#include "board.h"

#define CALIBRATION_SAMPLES 2000

///////////////////////////////////////

int16_t rawGyro[3];

int16_t rawGyroTemperature;

gyro_t _gyro;
gyro_t *gyro = &_gyro;

///////////////////////////////////////////////////////////////////////////////
// Read Gyro
///////////////////////////////////////////////////////////////////////////////

void readGyro(void)
{
    gyro->read(rawGyro);
}

void readGyroTemp(void)
{
    gyro->temperature(&rawGyroTemperature);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Gyro Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeGyroTCBias(void)
{
    sensors.gyroTemperature     = (float) (rawGyroTemperature + 13200) / 280.0f + 35.0f;
    
    sensors.gyroTCBias[ROLL]    = cfg.gyroTCBiasSlope[ROLL] * sensors.gyroTemperature + cfg.gyroTCBiasIntercept[ROLL];
    sensors.gyroTCBias[PITCH]   = cfg.gyroTCBiasSlope[PITCH] * sensors.gyroTemperature + cfg.gyroTCBiasIntercept[PITCH];
    sensors.gyroTCBias[YAW]     = cfg.gyroTCBiasSlope[YAW] * sensors.gyroTemperature + cfg.gyroTCBiasIntercept[YAW];
}

///////////////////////////////////////////////////////////////////////////////
// Gyro Temperature Calibration
///////////////////////////////////////////////////////////////////////////////

void gyroTempCalibration(void)
{
    uint16_t i;

    float bias1[3] = { 0.0f, 0.0f, 0.0f };
    float temperature1 = 0.0f;

    float bias2[3] = { 0.0f, 00.f, 0.0f };
    float temperature2 = 0.0f;

    uartPrint("\nGyro Temperature Calibration:\n");

    ///////////////////////////////////
    // Get samples at temperature1
    ///////////////////////////////////
    uartPrint("\nFirst Point: \n");
    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
        readGyro();
        readGyroTemp();
        bias1[ROLL]     += rawGyro[ROLL];
        bias1[PITCH]    += rawGyro[PITCH];
        bias1[YAW]      += rawGyro[YAW];
        temperature1    += ((float) rawGyroTemperature + 13200.0f) / 280.0f + 35.0f;
        delay(1);
    }
    
    for(i = 0; i < 3; ++i)
        bias1[i] /= (float) CALIBRATION_SAMPLES;

    temperature1 /= (float) CALIBRATION_SAMPLES;

    printf_min("R1:%f, P1:%f, Y1:%f, T1:%f\n", bias1[ROLL], bias1[PITCH], bias1[YAW], temperature1);

    ///////////////////////////////////
    // Time delay for temperature
    // Stabilizaiton
    ///////////////////////////////////

    uartPrint("\nWaiting 15 minutes for temp to rise, press a key to break.\n");

    // Delay for 15 minutes
    while (i < 450 || !uartAvailable()) {
        delay(1000);
        LED0_TOGGLE();
        readGyroTemp();
        printf_min("T: %f\n", ((float) rawGyroTemperature + 13200.0f) / 280.0f + 35.0f);
    }
    
    ///////////////////////////////////
    // Get samples at temperature2
    ///////////////////////////////////
    uartPrint("\nSecond Point: \n");
    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
        readGyro();
        bias2[ROLL]     += rawGyro[ROLL];
        bias2[PITCH]    += rawGyro[PITCH];
        bias2[YAW]      += rawGyro[YAW];
        temperature2    += ((float) rawGyroTemperature + 13200.0f) / 280.0f + 35.0f;
        delay(1);
    }

    for(i = 0; i < 3; ++i)
        bias2[i] /= (float) CALIBRATION_SAMPLES;
        
    temperature2 /= (float) CALIBRATION_SAMPLES;

    printf_min("R2:%f, P2:%f, Y2:%f, T2:%f\n", bias2[ROLL], bias2[PITCH], bias2[YAW], temperature2);

    cfg.gyroTCBiasSlope[ROLL]   = (bias2[ROLL] - bias1[ROLL]) / (temperature2 - temperature1);
    cfg.gyroTCBiasSlope[PITCH]  = (bias2[PITCH] - bias1[PITCH]) / (temperature2 - temperature1);
    cfg.gyroTCBiasSlope[YAW]    = (bias2[YAW] - bias1[YAW]) / (temperature2 - temperature1);

    cfg.gyroTCBiasIntercept[ROLL]   = bias2[ROLL] - cfg.gyroTCBiasSlope[ROLL] * temperature2;
    cfg.gyroTCBiasIntercept[PITCH]  = bias2[PITCH] - cfg.gyroTCBiasSlope[PITCH] * temperature2;
    cfg.gyroTCBiasIntercept[YAW]    = bias2[YAW] - cfg.gyroTCBiasSlope[YAW] * temperature2;

    uartPrint("\nTC Bias Slope\n");
    printf_min("R:%f, P:%f, Y:%f\n", cfg.gyroTCBiasSlope[ROLL], cfg.gyroTCBiasSlope[PITCH], cfg.gyroTCBiasSlope[YAW]);

    uartPrint("\nTC Bias Intercept:\n");
    printf_min("R:%f, P:%f, Y:%f\n", cfg.gyroTCBiasIntercept[ROLL], cfg.gyroTCBiasIntercept[PITCH], cfg.gyroTCBiasIntercept[YAW]);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Gyro Runtime Bias
///////////////////////////////////////////////////////////////////////////////

void computeGyroRTBias(void)
{
    uint16_t i;
    float gyroSum[3] = { 0.0f, 0.0f, 0.0f };

    for (i = 0; i < CALIBRATION_SAMPLES; ++i) {
        readGyro();
        readGyroTemp();
        computeGyroTCBias();

        gyroSum[ROLL]   += rawGyro[ROLL] - sensors.gyroTCBias[ROLL];
        gyroSum[PITCH]  += rawGyro[PITCH] - sensors.gyroTCBias[PITCH];
        gyroSum[YAW]    += rawGyro[YAW] - sensors.gyroTCBias[YAW];
        
        if(!(i % 250))
            LED0_TOGGLE();

        delay(1);
    }

    for (i = ROLL; i < 3; ++i)
        sensors.gyroRTBias[i] = (float) gyroSum[i] / (float)CALIBRATION_SAMPLES;
}

void initGyro(void)
{
    gyro->init();
}