/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 */

#pragma once

///////////////////////////////////////////////////////////////////////////////

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware definitions and GPIO

#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_Pin_3
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_Pin_4

#define LED0_TOGGLE() digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF()    digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON()     digitalLo(LED0_GPIO, LED0_PIN);

#define LED1_TOGGLE() digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF()    digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON()     digitalLo(LED1_GPIO, LED1_PIN);

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    GPIO_Pin_12

#define BEEP_TOGGLE() digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF()    digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON()     digitalLo(BEEP_GPIO, BEEP_PIN);

#include "actuator/pid.h"

#include "baseflight_proto.h"

#include "actuator/stabilisation.h"
#include "actuator/mixer.h"

#include "core/cli.h"
#include "core/config.h"
#include "core/command.h"
#include "core/printf_min.h"
#include "core/serial.h"
#include "core/utilities.h"

#include "drivers/adc.h"
#include "drivers/i2c.h"
#include "drivers/pwm_ppm.h"
#include "drivers/system.h"
#include "drivers/uart.h"

#include "sensors/accel.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/mag.h"
#include "sensors/devices/adxl345.h"
#include "sensors/devices/bmp085.h"
#include "sensors/devices/hmc5883.h"
#include "sensors/devices/mpu3050.h"
#include "sensors/devices/mpu6050.h"
#include "estimator/altitude.h"
#include "estimator/attitude.h"
#include "estimator/MahonyAHRS.h"

///////////////////////////////////////////////////////////////////////////////
