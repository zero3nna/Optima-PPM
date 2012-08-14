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
// BMP085 Variables
///////////////////////////////////////////////////////////////////////////////

#define BMP085_ADDRESS 0x77

///////////////////////////////////

//#define OSS 0  //  4.5 mSec conversion time (222.22 Hz)
#define OSS 1                   //  7.5 mSec conversion time (133.33 Hz)
//#define OSS 2  // 13.5 mSec conversion time ( 74.07 Hz)
//#define OSS 3  // 25.5 mSec conversion time ( 39.22 Hz)

///////////////////////////////////////

#define BMP085_PROM_START_ADDR  0xAA
#define BMP085_PROM_DATA_LEN    22

#define BMP085_T_MEASURE        0x2E    // temperature measurement
#define BMP085_P_MEASURE        0x34 + (OSS<<6) // pressure measurement
#define BMP085_CTRL_MEAS_REG    0xF4
#define BMP085_ADC_OUT_MSB_REG  0xF6

///////////////////////////////////////

float pressureAlt;

int32_t pressureAverage;

int32_t uncompensatedPressure;

int16_t uncompensatedTemperature;

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp085_smd500_calibration_param_t;

bmp085_smd500_calibration_param_t cal;

int32_t x1, x2, x3, b3, b5, b6, p;

uint32_t b4, b7;

///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////
void readTemperatureRequestPressure(void)
{
    uint8_t data[2];

    i2cRead(BMP085_ADDRESS, BMP085_ADC_OUT_MSB_REG, 2, data);

    uncompensatedTemperature = (data[0] << 8) | data[1];

    i2cWrite(BMP085_ADDRESS, BMP085_CTRL_MEAS_REG, BMP085_P_MEASURE);
}

///////////////////////////////////////////////////////////////////////////////
// ReadPressureRequestPressure
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestPressure(void)
{
    uint8_t data[3];

    i2cRead(BMP085_ADDRESS, BMP085_ADC_OUT_MSB_REG, 3, data);

    uncompensatedPressure = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2]) >> (8 - OSS);

    i2cWrite(BMP085_ADDRESS, BMP085_CTRL_MEAS_REG, BMP085_P_MEASURE);
}

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestTemperature(void)
{
    uint8_t data[3];

    i2cRead(BMP085_ADDRESS, BMP085_ADC_OUT_MSB_REG, 3, data);

    uncompensatedPressure = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2]) >> (8 - OSS);

    i2cWrite(BMP085_ADDRESS, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateTemperature(void)
{
    x1 = ((uncompensatedTemperature - (int32_t) cal.ac6) * (int32_t) cal.ac5) >> 15;
    x2 = ((int32_t) cal.mc << 11) / (x1 + cal.md);
    b5 = x1 + x2;
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void)
{
    b6 = b5 - 4000;
    x1 = (cal.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = cal.ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = ((((int32_t) cal.ac1 * 4 + x3) << OSS) + 2) >> 2;
    x1 = (cal.ac3 * b6) >> 13;
    x2 = (cal.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (cal.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = (uint32_t) (pressureAverage - b3) * (50000 >> OSS);
    p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    pressureAlt = (44330.0f * (1.0f - pow((float) p / 101325.0f, 0.190295f)));
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void initPressure(void)
{
    uint8_t data[BMP085_PROM_DATA_LEN];

    delay(15);

    i2cRead(BMP085_ADDRESS, BMP085_PROM_START_ADDR, BMP085_PROM_DATA_LEN, data);

    /*parameters AC1-AC6*/
    cal.ac1 =  (data[0] <<8) | data[1];
    cal.ac2 =  (data[2] <<8) | data[3];
    cal.ac3 =  (data[4] <<8) | data[5];
    cal.ac4 =  (data[6] <<8) | data[7];
    cal.ac5 =  (data[8] <<8) | data[9];
    cal.ac6 = (data[10] <<8) | data[11];

    /*parameters B1,B2*/
    cal.b1 =  (data[12] <<8) | data[13];
    cal.b2 =  (data[14] <<8) | data[15];

    /*parameters MB,MC,MD*/
    cal.mb =  (data[16] <<8) | data[17];
    cal.mc =  (data[18] <<8) | data[19];
    cal.md =  (data[20] <<8) | data[21];

    i2cWrite(BMP085_ADDRESS, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);

    delay(10);

    readTemperatureRequestPressure();

    delay(10);

    readPressureRequestTemperature();

    delay(10);

    calculateTemperature();
    calculatePressureAltitude();
}

///////////////////////////////////////////////////////////////////////////////