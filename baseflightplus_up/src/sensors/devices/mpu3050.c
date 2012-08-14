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

///////////////////////////////////////////////////////////////////////////////
// MPU3050 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Address

#define MPU3050_ADDRESS 0x68

// Registers

#define MPU3050_X_OFFS_H      0x0C
#define MPU3050_X_OFFS_L      0x0D
#define MPU3050_Y_OFFS_H      0x0E
#define MPU3050_Y_OFFS_L      0x0F
#define MPU3050_Z_OFFS_H      0x10
#define MPU3050_Z_OFFS_L      0x11
#define MPU3050_SMPLRT_DIV    0x15
#define MPU3050_DLPF_FS_SYNC  0x16
#define MPU3050_INT_CFG       0x17
#define MPU3050_TEMP_OUT      0x1B
#define MPU3050_GYRO_OUT      0x1D
#define MPU3050_USER_CTRL     0x3D
#define MPU3050_PWR_MGM       0x3E

// Bits

#define FS_SEL_2000_DPS       0x18

#define ACTL                  0x00
#define OPEN                  0x00
#define LATCH_INT_EN          0x20
#define INT_ANYRD_2CLEAR      0x10
#define RAW_RDY_EN            0x01

#define H_RESET               0x80
#define INTERNAL_OSC          0x00

///////////////////////////////////////

#define LOW_PASS_FILTER 0x18    // 256 Hz Low pass filter, 8 kHz internal sample rate
//#define LOW_PASS_FILTER 0x19  // 188 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1A  //  98 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1B  //  42 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1C  //  20 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1D  //  10 Hz Low pass filter, 1 kHz internal sample rate
//#define LOW_PASS_FILTER 0x1E  //   5 Hz Low pass filter, 1 kHz internal sample rate

#if (LOW_PASS_FILTER == 0x18)
#define SAMPLE_RATE_DIVISOR 0x07        // 1000 Hz = 8000/(7 + 1)
#else
#define SAMPLE_RATE_DIVISOR 0x00        // 1000 Hz = 1000/(0 + 1)
#endif

///////////////////////////////////////////////////////////////////////////////
// MPU3050/6050 Variables
///////////////////////////////////////////////////////////////////////////////

// MPU3050 14.375 LSBs per dps at Â±2000 Âº/s
// scale factor to get rad/s: (1/14.375*PI/180) = 0.00121414208834388144
#define MPU3050_GYRO_SCALE_FACTOR     0.00121414208834388144f

///////////////////////////////////////////////////////////////////////////////
// Read Gyro
///////////////////////////////////////////////////////////////////////////////

void mpu3050Read(int16_t values[3], int16_t* temperature)
{
    uint8_t buf[8];

    // Get data from device
    i2cRead(MPU3050_ADDRESS, MPU3050_TEMP_OUT, 8, buf);

    *temperature = (buf[0] << 8) | buf[1];

    values[XAXIS] = ((buf[2+0] << 8) | buf[2+1]);
    values[YAXIS] = ((buf[2+2] << 8) | buf[2+3]);
    values[ZAXIS] = -((buf[2+4] << 8) | buf[2+5]);
}

///////////////////////////////////////////////////////////////////////////////
// Gyro Initialization
///////////////////////////////////////////////////////////////////////////////

void mpu3050Init(void)
{
    uint8_t i;
    
    i2cWrite(MPU3050_ADDRESS, MPU3050_PWR_MGM, H_RESET);
    i2cWrite(MPU3050_ADDRESS, MPU3050_PWR_MGM, INTERNAL_OSC);
    i2cWrite(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, LOW_PASS_FILTER | FS_SEL_2000_DPS);
    i2cWrite(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, SAMPLE_RATE_DIVISOR);
    i2cWrite(MPU3050_ADDRESS, MPU3050_INT_CFG, 0);
    
    for(i = 0; i < 3; ++i) {
        sensors.gyroScaleFactor[i] = MPU3050_GYRO_SCALE_FACTOR;
    }
}

///////////////////////////////////////////////////////////////////////////////