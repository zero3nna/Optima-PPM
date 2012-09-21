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
#include "sensors/sensors.h"

#define CALIBRATION_SAMPLES 2000

void computeGyroTCBias(void)
{   
    gyro.temperature(&sensorData.gyroTemperature);
    sensorParams.gyroTCBias[ROLL]    = cfg.gyroTCBiasSlope[ROLL] * sensorData.gyroTemperature + cfg.gyroTCBiasIntercept[ROLL];
    sensorParams.gyroTCBias[PITCH]   = cfg.gyroTCBiasSlope[PITCH] * sensorData.gyroTemperature + cfg.gyroTCBiasIntercept[PITCH];
    sensorParams.gyroTCBias[YAW]     = cfg.gyroTCBiasSlope[YAW] * sensorData.gyroTemperature + cfg.gyroTCBiasIntercept[YAW];
}

// Gyro Temperature Calibration
//
// From Aeroquad
// http://code.google.com/p/aeroquad/source/browse/trunk/AeroQuad

void gyroTempCalibration(void)
{
    uint16_t i;

    float bias1[3] = { 0.0f, 0.0f, 0.0f };
    float temperature1 = 0.0f;

    float bias2[3] = { 0.0f, 00.f, 0.0f };
    float temperature2 = 0.0f;

    uartPrint("\nGyro Temperature Calibration:\n");

    // Get samples at temperature1
    uartPrint("\nFirst Point: \n");
    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
        gyro.read(sensorData.gyro);
        gyro.temperature(&sensorData.gyroTemperature);
        bias1[ROLL]     += sensorData.gyro[ROLL];
        bias1[PITCH]    += sensorData.gyro[PITCH];
        bias1[YAW]      += sensorData.gyro[YAW];
        temperature1    += sensorData.gyroTemperature;
        delay(1);
    }
    
    for(i = 0; i < 3; ++i)
        bias1[i] /= (float) CALIBRATION_SAMPLES;

    temperature1 /= (float) CALIBRATION_SAMPLES;

    printf_min("R1:%f, P1:%f, Y1:%f, T1:%f\n", bias1[ROLL], bias1[PITCH], bias1[YAW], temperature1);

    // Time delay for temperature

    uartPrint("\nWaiting 15 minutes for temp to rise, press a key to break.\n");

    // Delay for 15 minutes
    while (i++ < 450 && !uartAvailable()) {
        delay(1000);
        LED0_TOGGLE();
        gyro.temperature(&sensorData.gyroTemperature);
        printf_min("T: %f\n", sensorData.gyroTemperature);
    }
    
    // Get samples at temperature2
    uartPrint("\nSecond Point: \n");
    for (i = 0; i < CALIBRATION_SAMPLES; i++) {
        gyro.read(sensorData.gyro);
        gyro.temperature(&sensorData.gyroTemperature);
        bias2[ROLL]     += sensorData.gyro[ROLL];
        bias2[PITCH]    += sensorData.gyro[PITCH];
        bias2[YAW]      += sensorData.gyro[YAW];
        temperature2    += sensorData.gyroTemperature;
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

void computeGyroRTBias(void)
{
    uint16_t i;
    int32_t gyroSum[3] = { 0, 0, 0 };

    for (i = 0; i < CALIBRATION_SAMPLES; ++i) {
        gyro.read(sensorData.gyro);
        computeGyroTCBias();

        gyroSum[ROLL]   += sensorData.gyro[ROLL] - (int32_t)sensorParams.gyroTCBias[ROLL];
        gyroSum[PITCH]  += sensorData.gyro[PITCH] - (int32_t)sensorParams.gyroTCBias[PITCH];
        gyroSum[YAW]    += sensorData.gyro[YAW] - (int32_t)sensorParams.gyroTCBias[YAW];
        
        if(!(i % 250))
            LED0_TOGGLE();

        delay(1);
    }

    for (i = ROLL; i < 3; ++i)
        sensorParams.gyroRTBias[i] = gyroSum[i] / CALIBRATION_SAMPLES;
}

