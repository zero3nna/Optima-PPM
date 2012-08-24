/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */

#include "board.h"
#include "core/ring_buffer.h"
#include "core/filters.h"

float q[4]; // quaternion of sensor frame relative to auxiliary frame

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT);

static void updateSensors(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    
    uint8_t numAccelSamples = 0;
    int32_t accelAccum[3] = {0, 0, 0};
    SensorSample accelSample;
    
    uint8_t numGyroSamples = 0;
    int32_t gyroAccum[3] = {0, 0, 0};
    SensorSample gyroSample;
    
    uint8_t numMagSamples = 0;
    int32_t magAccum[3] = {0, 0, 0};
    SensorSample magSample;
    
    float temp[3] = {0.0f, 0.0f, 0.0f};
    
    uint8_t i;
    
    ///////////////////////////////////////////////////////////////////////////////
    
    if(cfg.magDriftCompensation) {
        while(bufferUsed(magSampleBuffer)) {
            bufferRead(magSampleBuffer, &magSample);
            magAccum[XAXIS] += magSample.x;
            magAccum[YAXIS] += magSample.y;
            magAccum[ZAXIS] += magSample.z;
            ++numMagSamples;
        }
    
        if(numMagSamples) {
            sensors.mag[XAXIS] = ((float) magAccum[XAXIS] / numMagSamples - cfg.magBias[XAXIS]) * sensors.magScaleFactor;
        	sensors.mag[YAXIS] = ((float) magAccum[YAXIS] / numMagSamples - cfg.magBias[YAXIS]) * sensors.magScaleFactor;
        	sensors.mag[ZAXIS] = ((float) magAccum[ZAXIS] / numMagSamples - cfg.magBias[ZAXIS]) * sensors.magScaleFactor; 
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    while(bufferUsed(accelSampleBuffer)) {
        bufferRead(accelSampleBuffer, &accelSample);
        accelAccum[XAXIS] += accelSample.x;
        accelAccum[YAXIS] += accelSample.y;
        accelAccum[ZAXIS] += accelSample.z;
        ++numAccelSamples;
    }
    
    if(numAccelSamples) {
        for(i = 0; i < 3; ++i)
            temp[i] = sensors.accel[i];
        sensors.accel[XAXIS] = ((float) accelAccum[XAXIS] / numAccelSamples - cfg.accelBias[XAXIS]) * sensors.accelScaleFactor;
        sensors.accel[YAXIS] = ((float) accelAccum[YAXIS] / numAccelSamples - cfg.accelBias[YAXIS]) * sensors.accelScaleFactor;
        sensors.accel[ZAXIS] = ((float) accelAccum[ZAXIS] / numAccelSamples - cfg.accelBias[ZAXIS]) * sensors.accelScaleFactor;
        if(cfg.accelLPF) {
            for(i = 0; i < 3; ++i)
                sensors.accel[i] = filterSmooth(sensors.accel[i], temp[i], cfg.accelLPF_Factor);
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    while(bufferUsed(gyroSampleBuffer)) {
        bufferRead(gyroSampleBuffer, &gyroSample);
        gyroAccum[XAXIS] += gyroSample.x;
        gyroAccum[YAXIS] += gyroSample.y;
        gyroAccum[ZAXIS] += gyroSample.z;
        ++numGyroSamples;
    }

    if(numGyroSamples) {
        readGyroTemp();
        computeGyroTCBias();
        sensors.gyro[XAXIS] = ((float) gyroAccum[XAXIS]  / numGyroSamples - sensors.gyroRTBias[XAXIS] - sensors.gyroTCBias[XAXIS]) * sensors.gyroScaleFactor;
        sensors.gyro[YAXIS] = ((float) gyroAccum[YAXIS]  / numGyroSamples - sensors.gyroRTBias[YAXIS] - sensors.gyroTCBias[YAXIS]) * sensors.gyroScaleFactor;
        sensors.gyro[ZAXIS] = ((float) gyroAccum[ZAXIS]  / numGyroSamples - sensors.gyroRTBias[ZAXIS] - sensors.gyroTCBias[ZAXIS]) * sensors.gyroScaleFactor;
    }
}

void updateAttitude(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    
    updateSensors();
    
    ///////////////////////////////////////////////////////////////////////////////
    
    static uint32_t now, last;
    float dT;
    
    now = micros();
    dT = (float)(now - last) * 1e-6f;
    last = now;
    
    ///////////////////////////////////////////////////////////////////////////////
    
    MahonyAHRSupdate( sensors.gyro[ROLL],   -sensors.gyro[PITCH],  sensors.gyro[YAW],
                    sensors.accel[XAXIS], sensors.accel[YAXIS], sensors.accel[ZAXIS],
                    sensors.mag[XAXIS],    sensors.mag[YAXIS],    sensors.mag[ZAXIS],
                    dT);
    
    ///////////////////////////////////////////////////////////////////////////////
    
    Quaternion2RPY(q, sensors.attitude);
    
    sensors.attitude[YAW] += cfg.magDeclination * DEG2RAD;
    
    ///////////////////////////////////////////////////////////////////////////////

}

//=====================================================================================================
// MahonyAHRS + Openpilot attitude.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT) { 
    static float integralFB[3] = { 0.0f, 0.0f, 0.0f };	// integral error terms scaled by Ki
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float norm;
	float hx, hy, bx, bz;
	float halfMagRot[3];
    float halfAccelRot[3];
    float halfErr[3];

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
	    
		// Normalise accelerometer measurement
		norm = sqrtf(ax * ax + ay * ay + az * az);
		
		if(!isinf(norm) && norm > 1.0e-3f && norm < cfg.accelCutoff) {    
    		ax /= norm;
    		ay /= norm;
    		az /= norm;     

    		// Estimated direction of gravity and vector perpendicular to magnetic flux
    		halfAccelRot[XAXIS] = q[1] * q[3] - q[0] * q[2];
    		halfAccelRot[YAXIS] = q[0] * q[1] + q[2] * q[3];
    		halfAccelRot[ZAXIS] = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 0.5f;
	
    		// Error is sum of cross product between estimated and measured direction of gravity
    		halfErr[XAXIS] = (az * halfAccelRot[YAXIS] - ay * halfAccelRot[ZAXIS]);
    		halfErr[YAXIS] = (ax * halfAccelRot[ZAXIS] - az * halfAccelRot[XAXIS]);
    		halfErr[ZAXIS] = (ay * halfAccelRot[XAXIS] - ax * halfAccelRot[YAXIS]);      

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
    		
    		if(cfg.magDriftCompensation && !(mx == 0.0f && my == 0.0f && mz == 0.0f)) {
    		    // Normalise magnetometer measurement
        		norm = sqrtf(mx * mx + my * my + mz * mz);

        		if(!isinf(norm)) {
        		    mx /= norm;
            		my /= norm;
            		mz /= norm;

                    // Reference direction of Earth's magnetic field        
                    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
                    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
                    bx = sqrt(hx * hx + hy * hy);
                    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

                    halfMagRot[XAXIS] = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
                    halfMagRot[YAXIS] = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
                    halfMagRot[ZAXIS] = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        		    halfErr[XAXIS] += (my * halfMagRot[ZAXIS] - mz * halfMagRot[YAXIS]);
            		halfErr[YAXIS] += (mz * halfMagRot[XAXIS] - mx * halfMagRot[ZAXIS]);
            		halfErr[ZAXIS] += (mx * halfMagRot[YAXIS] - my * halfMagRot[XAXIS]);
        		}
    		}

    		// Compute and apply integral feedback if enabled
    		if(cfg.imuKi > 0.0f) {
    			integralFB[XAXIS] += cfg.imuKi * halfErr[XAXIS] * dT * 0.5f;	// integral error scaled by Ki
    			integralFB[YAXIS] += cfg.imuKi * halfErr[YAXIS] * dT * 0.5f;
    			integralFB[ZAXIS] += cfg.imuKi * halfErr[ZAXIS] * dT * 0.5f;
    			gx += integralFB[XAXIS];	// apply integral feedback
    			gy += integralFB[YAXIS];
    			gz += integralFB[ZAXIS];
    		}

    		// Apply proportional feedback
    		gx += cfg.imuKp * halfErr[XAXIS];
    		gy += cfg.imuKp * halfErr[YAXIS];
    		gz += cfg.imuKp * halfErr[ZAXIS];
		}
	}
	
	// Integrate rate of change of quaternion
    {	
        float qdot[4];
        gx *= (0.5f * dT);		// pre-multiply common factors
    	gy *= (0.5f * dT);
    	gz *= (0.5f * dT);
    	qdot[0] = (-q[1] * gx - q[2] * gy - q[3] * gz);
    	qdot[1] = (q[0] * gx + q[2] * gz - q[3] * gy);
    	qdot[2] = (q[0] * gy - q[1] * gz + q[3] * gx);
    	qdot[3] = (q[0] * gz + q[1] * gy - q[2] * gx);
	
    	q[0] += qdot[0];
    	q[1] += qdot[1];
    	q[2] += qdot[2];
    	q[3] += qdot[3];
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
   		q[0] = -1.0f;
   		q[1] = 0.0f;
   		q[2] = 0.0f;
   		q[3] = 0.0f;
   	}
}