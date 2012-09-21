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

#pragma once

#include "board.h"

typedef struct {
	 
	 // Orientation states
	 union {
		  float heading;
		  float yaw;
		  float psi;
	 };
	 union {
		  float pitch;
		  float theta;
	 };
	 union {
		  float roll;
		  float phi;
	 };
	 
	 // Orientation rate states
	 union {
		  float heading_rate;
		  float yaw_rate;
		  float psi_dot;
	 };
	 
	 union {
		  float pitch_rate;
		  float theta_dot;
	 };
	 
	 union {
		  float roll_rate;
		  float phi_dot;
	 };
	 
	 float q[4]; // quaternion of sensor frame relative to auxiliary frame
	 
	 // Entries for storing processed sensor data
	 float gyro[3];
	 
	 float accel[3];
	 	 
	 float mag[3];
	 
	 int32_t altitude;

} AHRS_StateData;

extern AHRS_StateData stateData;

void updateAttitude(void);