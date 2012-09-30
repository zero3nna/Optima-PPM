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
#include "core/filters.h"

AHRS_StateData stateData;

static float accelLPF_A[4] = {-3.878129734998890f, 5.641762572815880f, -3.648875955419103f, 0.885247737995618f};
static float accelLPF_B[5] = {9.877867510385060e-04, -0.003762348901931f, 0.005553744695291f, -0.003762348901931f, 9.877867510385030e-04};
static fourthOrderData_t accelFilter[3];

static void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT);

static void updateSensors(void)
{   
    uint8_t i;
    float temp[3];
    
    if(sensorData.accelSamples) {
        
        for(i = 0; i < 3; ++i)
            temp[i] = stateData.accel[i];
            
        stateData.accel[X] = ((float)sensorData.accelAccum[X] / sensorData.accelSamples) * sensorParams.accelScaleFactor;
        stateData.accel[Y] = ((float)sensorData.accelAccum[Y] / sensorData.accelSamples) * sensorParams.accelScaleFactor;
        stateData.accel[Z] = ((float)sensorData.accelAccum[Z] / sensorData.accelSamples) * sensorParams.accelScaleFactor;
        
        if(cfg.accelSmoothFactor < 1.0f)
            for(i = 0; i < 3; ++i)
                stateData.accel[i] = filterSmooth(stateData.accel[i], temp[i], cfg.accelSmoothFactor);
        
        if(cfg.accelLPF)
            for(i = 0; i < 3; ++i)
                stateData.accel[i] = fourthOrderFilter(stateData.accel[i], &accelFilter[i], accelLPF_A, accelLPF_B);
    }
    
    if(sensorData.gyroSamples) {
        stateData.gyro[X] = ((float)sensorData.gyroAccum[X] / sensorData.gyroSamples - sensorParams.gyroTCBias[X]) * sensorParams.gyroScaleFactor;
        stateData.gyro[Y] = ((float)sensorData.gyroAccum[Y] / sensorData.gyroSamples - sensorParams.gyroTCBias[Y]) * sensorParams.gyroScaleFactor;
        stateData.gyro[Z] = (float)(sensorData.gyroAccum[Z] / sensorData.gyroSamples - sensorParams.gyroTCBias[Z]) * sensorParams.gyroScaleFactor;
    }
    
    if(sensorData.magSamples) {
        stateData.mag[X] = ((float)sensorData.magAccum[X] / sensorData.magSamples) * sensorParams.magScaleFactor;
    	stateData.mag[Y] = ((float)sensorData.magAccum[Y] / sensorData.magSamples) * sensorParams.magScaleFactor;
    	stateData.mag[Z] = ((float)sensorData.magAccum[Z] / sensorData.magSamples) * sensorParams.magScaleFactor;
    }
}

void updateAttitude(void)
{   
    updateSensors();
    
    static uint32_t now, last;
    float dT;
    
    now = micros();
    dT = (float)(now - last) * 1e-6f;
    last = now;
    
    AHRSUpdate( stateData.gyro[ROLL],   -stateData.gyro[PITCH],  stateData.gyro[YAW],
                    stateData.accel[X], stateData.accel[Y], stateData.accel[Z],
                    stateData.mag[X],    stateData.mag[Y],    stateData.mag[Z],
                    dT);
                    
    zeroSensorAccumulators();
    
    Quaternion2RPY(stateData.q, &stateData.roll, &stateData.pitch, &stateData.yaw);
    
    stateData.heading += cfg.magDeclination * DEG2RAD;
}

//=====================================================================================================
// S.O.H. Madgwick + OpenPilot attitude.c + MultiWii
// 25th August 2010
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (b[X] b[Z]) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'dTOn2' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.
//
// Gyroscope units are radians/second, accelerometer and magnetometer units are irrelevant as the
// vector is normalised.
//
//=====================================================================================================

static void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT) {
    static float errInt[3] = { 0.0f, 0.0f, 0.0f };	// integral error terms scaled by Ki
    float *q = stateData.q;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; // auxiliary variables to reduce number of repeated operations
	float h[3], b[3];
	float fluxRot[3], gravRot[3];
    float err[3] = {0.0f, 0.0f, 0.0f};
    float norm;
    float halfT = dT * 0.5f;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(sensorData.accelSamples && !(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
	    
		// Normalise accelerometer measurement
		norm = sqrtf(ax * ax + ay * ay + az * az);
		
		// Sanity check and multiwii style accel cutoff
		// This deals with hard accelerations where the accelerometer is less trusted and 0G cases
		// When we are at large angles, level mode will act like rate mode.
		if(!isinf(norm) && abs(norm - ACCEL_1G) < 0.4f * ACCEL_1G) {    
    		ax /= norm;
    		ay /= norm;
    		az /= norm;
    		
    		// Auxiliary variables to avoid repeated arithmetic
            q0q0 = q[0] * q[0];
            q0q1 = q[0] * q[1];
            q0q2 = q[0] * q[2];
            q0q3 = q[0] * q[3];
            q1q1 = q[1] * q[1];
            q1q2 = q[1] * q[2];
            q1q3 = q[1] * q[3];
            q2q2 = q[2] * q[2];
            q2q3 = q[2] * q[3];
            q3q3 = q[3] * q[3];

    		// Estimated direction of gravity
    		gravRot[X] = 2.0f * (q1q3 - q0q2);
    		gravRot[Y] = 2.0f * (q0q1 + q2q3);
    		gravRot[Z] = q0q0 - q1q1 - q2q2 + q3q3;
	
    		// Error is sum of cross product between estimated and measured direction of gravity
            err[X] = (az * gravRot[Y] - ay * gravRot[Z]) * cfg.accelKp;
            err[Y] = (ax * gravRot[Z] - az * gravRot[X]) * cfg.accelKp;
            err[Z] = (ay * gravRot[X] - ax * gravRot[Y]) * cfg.accelKp;
            
            errInt[X] += cfg.accelKi * err[X];	// integral error scaled by Ki
    		errInt[Y] += cfg.accelKi * err[Y];
    		errInt[Z] += cfg.accelKi * err[Z];
    		
    		// Apply proportional feedback
        	gx += err[X];
        	gy += err[Y];
        	gz += err[Z];

        	// Apply integral feedback
        	gx += errInt[X];
        	gy += errInt[Y];
        	gz += errInt[Z];
		}
	}
	
	if(sensorData.magSamples && cfg.magDriftCompensation && !(mx == 0.0f && my == 0.0f && mz == 0.0f)) {
	    // Normalise magnetometer measurement
		norm = sqrtf(mx * mx + my * my + mz * mz);

		if(!isinf(norm)) {
		    mx /= norm;
    		my /= norm;
    		mz /= norm;

            // Reference direction of Earth's magnetic field        
            h[X] = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            h[Y] = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            h[Z] = b[Z] = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
            b[X] = sqrt(h[X] * h[X] + h[Y] * h[Y]);

            // Estimated direction of vector perpendicular to magnetic flux
            fluxRot[X] = 2.0f * (b[X] * (0.5f - q2q2 - q3q3) + b[Z] * (q1q3 - q0q2));
            fluxRot[Y] = 2.0f * (b[X] * (q1q2 - q0q3) + b[Z] * (q0q1 + q2q3));
            fluxRot[Z] = 2.0f * (b[X] * (q0q2 + q1q3) + b[Z] * (0.5f - q1q1 - q2q2));

		    err[X] = (my * fluxRot[Z] - mz * fluxRot[Y]) * cfg.magKp;
    		err[Y] = (mz * fluxRot[X] - mx * fluxRot[Z]) * cfg.magKp;
    		err[Z] = (mx * fluxRot[Y] - my * fluxRot[X]) * cfg.magKp;
    		
    		errInt[X] += cfg.magKi * err[X] * halfT;	// integral error scaled by Ki
    		errInt[Y] += cfg.magKi * err[Y] * halfT;
    		errInt[Z] += cfg.magKi * err[Z] * halfT;
    		
    		// Apply proportional feedback
        	gx += err[X];
        	gy += err[Y];
        	gz += err[Z];

        	// Apply integral feedback
        	gx += errInt[X];
        	gy += errInt[Y];
        	gz += errInt[Z];
		}
	}
	
	// Integrate rate of change of quaternion
    {	
        float qdot[4];
    	qdot[0] = (-q[1] * gx - q[2] * gy - q[3] * gz) * halfT;
    	qdot[1] = (q[0] * gx + q[2] * gz - q[3] * gy) * halfT;
    	qdot[2] = (q[0] * gy - q[1] * gz + q[3] * gx) * halfT;
    	qdot[3] = (q[0] * gz + q[1] * gy - q[2] * gx) * halfT;
	
    	q[0] += qdot[0];
    	q[1] += qdot[1];
    	q[2] += qdot[2];
    	q[3] += qdot[3];
    	
    	if(q[0] < 0) {
			q[0] = -q[0];
			q[1] = -q[1];
			q[2] = -q[2];
			q[3] = -q[3];
		}
	}
	
	// Normalise quaternion
	norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
	
	// If quaternion has become inappropriately short or is nan reinit.
	// THIS SHOULD NEVER ACTUALLY HAPPEN
	if(fabs(norm) < 1.0e-3f || norm != norm || isinf(norm)) {
   		q[0] = 1.0f;
   		q[1] = 0.0f;
   		q[2] = 0.0f;
   		q[3] = 0.0f;
   	}
}