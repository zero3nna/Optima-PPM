/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 *
 * August 2012 - Modified for use in Baseflight Proto (Scott Driessens)
 */

//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>
#include "board.h"

//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	512.0f			// sample frequency in Hz

//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

// gyro must be in rad/s - mag and accel do no matter as they are normalised

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT) {
	float recipNorm;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc, qd;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dT);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
	    
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		
		if(isinf(recipNorm) || abs(ACCEL_1G - 1.0f / recipNorm) > cfg.accelCutoff)
            return;
            
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		
		if(isinf(recipNorm))
            return;
            
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field        
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3; // Close enough
		//halfvz = (q0q0 - q1q1 - q2q2 + q3q3) * 0.5f;
        
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		/*
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
		*/
		// Corrected directions for MW coordinate system
		halfex = (az * halfvy - ay * halfvz) + (my * halfwz - mz * halfwy);
		halfey = (ax * halfvz - az * halfvx) + (mz * halfwx - mx * halfwz);
		halfez = (ay * halfvx - ax * halfvy) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(cfg.twoKi > 0.0f) {
			integralFBx += cfg.twoKi * halfex * dT;	// integral error scaled by Ki
			integralFBy += cfg.twoKi * halfey * dT;
			integralFBz += cfg.twoKi * halfez * dT;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += cfg.twoKp * halfex;
		gy += cfg.twoKp * halfey;
		gz += cfg.twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dT);		// pre-multiply common factors
	gy *= (0.5f * dT);
	gz *= (0.5f * dT);
	qa = q0;
	qb = q1;
	qc = q2;
    qd = q3;
	q0 += (-qb * gx - qc * gy - qd * gz);
	q1 += (qa * gx + qc * gz - qd * gy);
	q2 += (qa * gy - qb * gz + qd * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	
	if(isinf(recipNorm))
        return;
        
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dT) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc, qd;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		
		if(isinf(recipNorm))
            return;
            
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		
		// Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;       

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		/*
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		*/
		// Corrected directions for MW coordinate system
		halfex = (az * halfvy - ay * halfvz);
		halfey = (ax * halfvz - az * halfvx);
		halfez = (ay * halfvx - ax * halfvy);

		// Compute and apply integral feedback if enabled
		if(cfg.twoKi > 0.0f) {
			integralFBx += cfg.twoKi * halfex * dT;	// integral error scaled by Ki
			integralFBy += cfg.twoKi * halfey * dT;
			integralFBz += cfg.twoKi * halfez * dT;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += cfg.twoKp * halfex;
		gy += cfg.twoKp * halfey;
		gz += cfg.twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dT);		// pre-multiply common factors
	gy *= (0.5f * dT);
	gz *= (0.5f * dT);
	qa = q0;
	qb = q1;
	qc = q2;
    qd = q3;
	q0 += (-qb * gx - qc * gy - qd * gz);
	q1 += (qa * gx + qc * gz - qd * gy);
	q2 += (qa * gy - qb * gz + qd * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	
	if(isinf(recipNorm))
        return;
        
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
