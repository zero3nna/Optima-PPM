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

#include "drivers/adc.h"
#include "sensors/sensors.h"
#include "core/config.h"

float batteryAdcToVoltage(float src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 11 = 11:1 voltage divider (10k:1k) for 1V
    return (((src) * 3.3f) / 4095.0f) * cfg.batScale;
}

void batteryInit(void)
{
    uint32_t i;
    float voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++) {
        voltage += adcGet();
        delay(10);
    }

    voltage = batteryAdcToVoltage(voltage / 32.0f);

    // autodetect cell count, going from 2S..6S
    for (i = 2; i < 6; i++) {
        if (voltage < i * cfg.batMaxCellVoltage)
            break;
    }
    sensorData.batteryCellCount = i;
    sensorData.batteryWarningVoltage = i * cfg.batMinCellVoltage; // 3.3V per cell minimum, configurable in CLI
}