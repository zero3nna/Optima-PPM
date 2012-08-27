/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */

#include "board.h"
#include "core/ring_buffer.h"
#include "core/filters.h"

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // quaternion of sensor frame relative to auxiliary frame

void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT);

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
    
    AHRSUpdate( sensors.gyro[ROLL],   -sensors.gyro[PITCH],  sensors.gyro[YAW],
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

void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT) { 
    static float errInt[3] = { 0.0f, 0.0f, 0.0f };	// integral error terms scaled by Ki
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; // auxiliary variables to reduce number of repeated operations
	float hx, hy, hz, bx, bz;
	float fluxRot[3], gravRot[3];
    float err[3];
    float norm;
    float halfT = dT * 0.5f;
    //float angleNorm;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
	    
		// Normalise accelerometer measurement
		norm = sqrtf(ax * ax + ay * ay + az * az);
		//angleNorm = sqrtf(sensors.attitude[ROLL] * sensors.attitude[ROLL] + sensors.attitude[PITCH] * sensors.attitude[PITCH]) * RAD2DEG;
		
		// Sanity check and multiwii style accel cutoff
		// This deals with hard accelerations where the accelerometer is less trusted and 0G cases
		// When we are at large angles, level mode will act like rate mode.
		// Maybe we could think of scaling it so it doesn't turn off suddenly? This works with multiwii though...
		if(!isinf(norm) && norm > 0.6f * ACCEL_1G && norm < 1.4f * ACCEL_1G/* && angleNorm < 25*/) {    
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
    		gravRot[XAXIS] = 2.0f * (q1q3 - q0q2);
    		gravRot[YAXIS] = 2.0f * (q0q1 + q2q3);
    		gravRot[ZAXIS] = q0q0 - q1q1 - q2q2 + q3q3;
	
    		// Error is sum of cross product between estimated and measured direction of gravity
    		err[XAXIS] = (az * gravRot[YAXIS] - ay * gravRot[ZAXIS]);
    		err[YAXIS] = (ax * gravRot[ZAXIS] - az * gravRot[XAXIS]);
    		err[ZAXIS] = (ay * gravRot[XAXIS] - ax * gravRot[YAXIS]);      
    		
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
                    hz = bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
                    bx = sqrt(hx * hx + hy * hy);

                    // Estimated direction of vector perpendicular to magnetic flux
                    fluxRot[XAXIS] = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
                    fluxRot[YAXIS] = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
                    fluxRot[ZAXIS] = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

        		    err[XAXIS] += (my * fluxRot[ZAXIS] - mz * fluxRot[YAXIS]);
            		err[YAXIS] += (mz * fluxRot[XAXIS] - mx * fluxRot[ZAXIS]);
            		err[ZAXIS] += (mx * fluxRot[YAXIS] - my * fluxRot[XAXIS]);
        		}
    		}
    		
    		// Apply proportional feedback
    		gx += cfg.imuKp * err[XAXIS];
    		gy += cfg.imuKp * err[YAXIS];
    		gz += cfg.imuKp * err[ZAXIS];

    		// Compute and apply integral feedback if enabled
    		if(cfg.imuKi > 0.0f) {
    			errInt[XAXIS] += cfg.imuKi * err[XAXIS] * halfT;	// integral error scaled by Ki
    			errInt[YAXIS] += cfg.imuKi * err[YAXIS] * halfT;
    			errInt[ZAXIS] += cfg.imuKi * err[ZAXIS] * halfT;
    			gx += errInt[XAXIS];	// apply integral feedback
    			gy += errInt[YAXIS];
    			gz += errInt[ZAXIS];
    		}
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