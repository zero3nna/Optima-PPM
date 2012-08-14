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
#include "estimator/lp_filter.h"

///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 200 Hz Data
///////////////////////////////////////////////////////////////////////////////

float computeFourthOrder200Hz(float currentInput, fourthOrderData_t *filterParameters)
{
    // cheby2(4,60,12.5/100)
#define B0_200HZ  0.001139392787073f
#define B1_200HZ -0.003386240693441f
#define B2_200HZ  0.004665482032666f
#define B3_200HZ -0.003386240693441f
#define B4_200HZ  0.001139392787073f

#define A1_200HZ -3.692341608388116f
#define A2_200HZ  5.123502002652351f
#define A3_200HZ -3.165946995349404f
#define A4_200HZ  0.734958387305099f

    float output;

    output = B0_200HZ * currentInput + B1_200HZ * filterParameters->inputTm1 + B2_200HZ * filterParameters->inputTm2 + B3_200HZ * filterParameters->inputTm3 + B4_200HZ * filterParameters->inputTm4 - A1_200HZ * filterParameters->outputTm1 - A2_200HZ * filterParameters->outputTm2 - A3_200HZ * filterParameters->outputTm3 - A4_200HZ * filterParameters->outputTm4;

    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////
