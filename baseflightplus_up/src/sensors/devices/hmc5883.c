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
#include "drivers/i2c.h"

///////////////////////////////////////////////////////////////////////////////

#define HMC5883_ADDRESS 0x1E

#define HMC5883_CONFIG_REG_A    0x00
#define HMC5883_CONFIG_REG_B    0x01
#define HMC5883_MODE_REG        0x02
#define HMC5883_DATA_X_MSB_REG  0x03
#define HMC5883_STATUS_REG      0x09

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_CONFIG 0x18  // 1 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x38  // 2 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x58  // 4 Sample average, 75 Hz
#define SENSOR_CONFIG 0x78      // 8 Sample average, 75 Hz

#define NORMAL_MEASUREMENT_CONFIGURATION 0x00
#define POSITIVE_BIAS_CONFIGURATION      0x01

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_GAIN 0x00  // +/- 0.88 Ga
#define SENSOR_GAIN 0x20        // +/- 1.3  Ga (default)
//#define SENSOR_GAIN 0x40  // +/- 1.9  Ga
//#define SENSOR_GAIN 0x60  // +/- 2.5  Ga
//#define SENSOR_GAIN 0x80  // +/- 4.0  Ga
//#define SENSOR_GAIN 0xA0  // +/- 4.7  Ga
//#define SENSOR_GAIN 0xC0  // +/- 5.6  Ga
//#define SENSOR_GAIN 0xE0  // +/- 8.1  Ga

///////////////////////////////////////////////////////////////////////////////

#define OP_MODE_CONTINUOUS 0x00 // Continuous conversion
#define OP_MODE_SINGLE     0x01 // Single converison

#define STATUS_RDY         0x01 // Data Ready

///////////////////////////////////////////////////////////////////////////////
// Read Magnetometer
///////////////////////////////////////////////////////////////////////////////

void hmc5883Read(int16_t *values)
{
    uint8_t buf[6];

    i2cRead(HMC5883_ADDRESS, HMC5883_DATA_X_MSB_REG, 6, buf);

    values[XAXIS] = -(buf[0] << 8 | buf[1]);
    // the Z registers comes before the Y registers in the HMC5883L
    values[ZAXIS] = -(buf[2] << 8 | buf[3]);
    values[YAXIS] = (buf[4] << 8 | buf[5]);
}

bool hmc5883Detect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(HMC5883_ADDRESS, 0x0A, 1, &sig);
    if (!ack || sig != 'H')
        return false;
        
    mag->init = hmc5883Init;
    mag->read = hmc5883Read;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Initialize Magnetometer
///////////////////////////////////////////////////////////////////////////////

void hmc5883Init(void)
{
    /*
       uint8_t I2C2_Buffer_Rx[1] = { 0 };

       i2cWrite( HMC5883_ADDRESS, HMC5883_CONFIG_REG_B, SENSOR_GAIN );
       delay(20);

       i2cWrite( HMC5883_ADDRESS, HMC5883_CONFIG_REG_A, SENSOR_CONFIG | POSITIVE_BIAS_CONFIGURATION );
       delay(20);

       i2cWrite( HMC5883_ADDRESS, HMC5883_MODE_REG, OP_MODE_SINGLE );

       while ( (I2C2_Buffer_Rx[0] & STATUS_RDY) == 0x00 )
       i2cRead ( HMC5883_ADDRESS, HMC5883_STATUS_REG, 1, I2C2_Buffer_Rx );

       readMag();
     */

    sensors.magScaleFactor = 1.0f; // (1.16F * 1090.0F) / (float)rawMag[i];

    i2cWrite(HMC5883_ADDRESS, HMC5883_CONFIG_REG_A, SENSOR_CONFIG | NORMAL_MEASUREMENT_CONFIGURATION);
    delay(20);

    i2cWrite(HMC5883_ADDRESS, HMC5883_MODE_REG, OP_MODE_CONTINUOUS);
    delay(20);

    readMag();

    delay(20);
}

///////////////////////////////////////////////////////////////////////////////