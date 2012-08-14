/*
  June 2012

  BaseFlightPlus Rev -

  An Open Source STM32 Based Multicopter

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick

  Designed to run on Naze32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "board.h"

///////////////////////////////////

int16_t rawAccel[3];

///////////////////////////////////////////////////////////////////////////////
// Read Accel
///////////////////////////////////////////////////////////////////////////////

void readAccel(void)
{
    if(useMPU6050)
    {
        mpu6050Read(rawAccel, 0, 0);
    }
    else
    {
        adxl345Read(rawAccel);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Compute Accel Runtime Bias
///////////////////////////////////////////////////////////////////////////////

void computeAccelRTBias(void)
{
    uint16_t samples;
    float accelSum[3] = { 0, 0, 0 };

    for (samples = 0; samples < 2000; samples++) {
        readAccel();

        // Don't forget about the precalculated bias..
        accelSum[XAXIS] += rawAccel[XAXIS] - sensorConfig.accelBias[XAXIS];
        accelSum[YAXIS] += rawAccel[YAXIS] - sensorConfig.accelBias[YAXIS];
        accelSum[ZAXIS] += rawAccel[ZAXIS] - sensorConfig.accelBias[ZAXIS];

        delay(1);
    }

    sensors.accelRTBias[XAXIS] = accelSum[XAXIS] / 2000.0f;
    sensors.accelRTBias[YAXIS] = accelSum[YAXIS] / 2000.0f;
    sensors.accelRTBias[ZAXIS] = (accelSum[ZAXIS] / 2000.0f) + (ACCEL_1G / fabs(sensorConfig.accelScaleFactor[ZAXIS]));
}

///////////////////////////////////////////////////////////////////////////////
// Accelerometer Calibration
///////////////////////////////////////////////////////////////////////////////


static void gatherDataAxis(char* explain, float* average, uint8_t axis)
{
    int32_t sum = 0;
    uint32_t i;

    uartPrint("Place accelerometer ");
    uartPrint(explain);
    uartPrint("\n  Send a character when ready to proceed\n\n");

    while (!uartAvailable()) { }   
    uartRead();

    uartPrint("  Gathering Data...\n\n");

    for (i = 0; i < 5000; i++) {
        readAccel();
        sum += rawAccel[axis];
        delayMicroseconds(1000);
    }

    *average = (float)sum / 5000.0f;
    printf_min("%f\n\n", *average);
}

void accelCalibration(void)
{
    float noseUp;
    float noseDown;
    float leftWingDown;
    float rightWingDown;
    float upSideDown;
    float rightSideUp;

    uartPrint("\nAccelerometer Calibration:\n\n");

    ///////////////////////////////////
    gatherDataAxis("right side up", &rightSideUp, ZAXIS);
    gatherDataAxis("up side down", &upSideDown, ZAXIS);

    sensorConfig.accelBias[ZAXIS] = (rightSideUp + upSideDown) / 2.0f;
    sensorConfig.accelScaleFactor[ZAXIS] = (2.0f * ACCEL_1G) / (fabs(rightSideUp) + fabs(upSideDown));

    ///////////////////////////////////
    gatherDataAxis("left edge down", &leftWingDown, YAXIS);
    gatherDataAxis("right edge down", &rightWingDown, YAXIS);

    sensorConfig.accelBias[YAXIS] = (leftWingDown + rightWingDown) / 2.0f;
    sensorConfig.accelScaleFactor[YAXIS] = (2.0f * ACCEL_1G) / (fabs(leftWingDown) + fabs(rightWingDown));

    ///////////////////////////////////
    gatherDataAxis("rear edge down", &noseUp, XAXIS);
    gatherDataAxis("front edge down", &noseDown, XAXIS);

    sensorConfig.accelBias[XAXIS] = (noseUp + noseDown) / 2.0f;
    sensorConfig.accelScaleFactor[XAXIS] = (2.0f * ACCEL_1G) / (fabs(noseUp) + fabs(noseDown));

    ///////////////////////////////////
    printf_min("test:%f\n\n", (noseDown - sensorConfig.accelBias[XAXIS]) * sensorConfig.accelScaleFactor[XAXIS]);

    printf_min("%f, %f, %f, %f, %f, %f\n\n", noseUp, noseDown, sensorConfig.accelScaleFactor[XAXIS], 
                    sensorConfig.accelBias[XAXIS], (noseUp - sensorConfig.accelBias[XAXIS]) * sensorConfig.accelScaleFactor[XAXIS], 
                    (noseDown - sensorConfig.accelBias[XAXIS]) * sensorConfig.accelScaleFactor[XAXIS]);
    
    printf_min("%f, %f, %f, %f, %f, %f\n\n", leftWingDown, rightWingDown, sensorConfig.accelScaleFactor[YAXIS],
                    sensorConfig.accelBias[YAXIS], (leftWingDown - sensorConfig.accelBias[YAXIS]) * sensorConfig.accelScaleFactor[YAXIS],
                    (rightWingDown - sensorConfig.accelBias[YAXIS]) * sensorConfig.accelScaleFactor[YAXIS]);
                
    printf_min("%f, %f, %f, %f, %f, %f\n", upSideDown, rightSideUp, sensorConfig.accelScaleFactor[ZAXIS], sensorConfig.accelBias[ZAXIS],
                    (upSideDown - sensorConfig.accelBias[ZAXIS]) * sensorConfig.accelScaleFactor[ZAXIS],
                    (rightSideUp - sensorConfig.accelBias[ZAXIS]) * sensorConfig.accelScaleFactor[ZAXIS]);

    uartPrint("\nAccel Calibration Complete.\n");
    
    sensorConfig.accelCalibrated = true;
}

///////////////////////////////////////////////////////////////////////////////
// Accel Initialization
///////////////////////////////////////////////////////////////////////////////

void initAccel(void)
{
    if(!useMPU6050)
    {
        adxl345Init();
    }

    //computeAccelRTBias();
    
    sensors.accelRTBias[XAXIS] = 0.0f;
    sensors.accelRTBias[YAXIS] = 0.0f;
    sensors.accelRTBias[ZAXIS] = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////