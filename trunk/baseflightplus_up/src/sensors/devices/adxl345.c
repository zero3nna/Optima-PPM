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
#include "adxl345.h"

///////////////////////////////////////////////////////////////////////////////
// ADXL345 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Address

#define ADXL345_ADDRESS 0x53

// Registers

#define ADXL345_OFSX        0x1E
#define ADXL345_OFSY        0x1F
#define ADXL345_OFSZ        0x20
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32

// Bits

#define MEASURE             0x08

#define FULL_RES            0x08
#define RANGE_4_G           0x01

#define DATA_RATE_1600      0x0E


///////////////////////////////////////////////////////////////////////////////
// Read Accel
///////////////////////////////////////////////////////////////////////////////

void adxl345Read(int16_t values[3])
{
    uint8_t buffer[6];

    i2cRead(ADXL345_ADDRESS, ADXL345_DATAX0, 6, buffer);

    values[YAXIS] = -(buffer[0] + (buffer[1] << 8));
    values[XAXIS] = -(buffer[2] + (buffer[3] << 8));
    values[ZAXIS] = -(buffer[4] + (buffer[5] << 8));
}

///////////////////////////////////////////////////////////////////////////////
// Accel Initialization
///////////////////////////////////////////////////////////////////////////////

void adxl345Init(void)
{
    uint8_t i;
    
    i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, MEASURE);

    delay(10);

    i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, FULL_RES | RANGE_4_G);

    delay(10);

    i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, DATA_RATE_1600);

    delay(100);
    
    if(!sensorConfig.accelCalibrated) {
        for(i = 0; i < 3; ++i) {
            sensorConfig.accelScaleFactor[i] = ACCEL_1G / 256.0f;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////