/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */

#include "board.h"
#include "core/filters.h"

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // quaternion of sensor frame relative to auxiliary frame
static uint8_t AHRSInitialised = false;

static void AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT);
//static float calculateAccConfidence(float accNorm);

static void updateSensors(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    
    uint8_t i;
    float temp[3];
    
    ///////////////////////////////////////////////////////////////////////////////
    
    if(magSamples.numSamples) {
        sensors.mag[XAXIS] = (magSamples.accum[XAXIS] / magSamples.numSamples - cfg.magBias[XAXIS]) * sensors.magScaleFactor;
    	sensors.mag[YAXIS] = (magSamples.accum[YAXIS] / magSamples.numSamples - cfg.magBias[YAXIS]) * sensors.magScaleFactor;
    	sensors.mag[ZAXIS] = (magSamples.accum[ZAXIS] / magSamples.numSamples - cfg.magBias[ZAXIS]) * sensors.magScaleFactor;
        zeroSensorSamples(&magSamples);
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    if(accelSamples.numSamples) {
        if(cfg.accelLPF) {
            for(i = 0; i < 3; ++i)
                temp[i] = sensors.accel[i];
        }
        sensors.accel[XAXIS] = (accelSamples.accum[XAXIS] / accelSamples.numSamples - cfg.accelBias[XAXIS]) * sensors.accelScaleFactor;
        sensors.accel[YAXIS] = (accelSamples.accum[YAXIS] / accelSamples.numSamples - cfg.accelBias[YAXIS]) * sensors.accelScaleFactor;
        sensors.accel[ZAXIS] = (accelSamples.accum[ZAXIS] / accelSamples.numSamples - cfg.accelBias[ZAXIS]) * sensors.accelScaleFactor;
        zeroSensorSamples(&accelSamples);
        if(cfg.accelLPF) {
            for(i = 0; i < 3; ++i)
                sensors.accel[i] = filterSmooth(sensors.accel[i], temp[i], cfg.accelLPF_Factor);
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    
    if(gyroSamples.numSamples) {
        readGyroTemp();
        computeGyroTCBias();
        sensors.gyro[XAXIS] = (gyroSamples.accum[XAXIS] / gyroSamples.numSamples - sensors.gyroRTBias[XAXIS] - sensors.gyroTCBias[XAXIS]) * sensors.gyroScaleFactor;
        sensors.gyro[YAXIS] = (gyroSamples.accum[YAXIS] / gyroSamples.numSamples - sensors.gyroRTBias[YAXIS] - sensors.gyroTCBias[YAXIS]) * sensors.gyroScaleFactor;
        sensors.gyro[ZAXIS] = (gyroSamples.accum[ZAXIS] / gyroSamples.numSamples - sensors.gyroRTBias[ZAXIS] - sensors.gyroTCBias[ZAXIS]) * sensors.gyroScaleFactor;
        zeroSensorSamples(&gyroSamples);
        for(i = 0; i < 3; ++i)
            sensors.gyro[i] = filterSmooth(sensors.gyro[i], 0.0f, cfg.gyroWeakZeroFactor);
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

/*
#define CONFIDENCE_DECAY            1.0f
#define CONFIDENCE_FILTER_FACTOR    0.75f

static float calculateAccConfidence(float accNorm) {
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity
	static float accNormPrev = 1.0f;

	accNorm = filterSmooth(accNormPrev, accNorm, CONFIDENCE_FILTER_FACTOR);
	accNormPrev = accNorm;

	return constrain(1.0f - (CONFIDENCE_DECAY * sqrtf(abs(accNorm - 1.0f))), 0.0f, 1.0f);
} // calculateAccConfidence
*/

//=====================================================================================================
// S.O.H. Madgwick + OpenPilot attitude.c
// 25th August 2010
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
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
// adapted from John Ihlein's AQ version using Aerospace Coordinates
// gain scaling with accelerometer magnitude above 1G Greg Egan
//
//=====================================================================================================

static void AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q[2] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    
    if(q[0] < 0) {
		q[0] = -q[0];
		q[1] = -q[1];
		q[2] = -q[2];
		q[3] = -q[3];
	}
}

static void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dT) { 
    static float errInt[3] = { 0.0f, 0.0f, 0.0f };	// integral error terms scaled by Ki
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; // auxiliary variables to reduce number of repeated operations
	float hx, hy, hz, bx, bz;
	float fluxRot[3], gravRot[3];
    float err[3];
    float norm;
    float halfT = dT * 0.5f;
    //float accConfidence;
    //float angleNorm;
    
    if(!AHRSInitialised) {
        AHRSinit(ax, ay, az, mx, my, mz);
        AHRSInitialised = true;
    }

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
    		
            //accConfidence = calculateAccConfidence(norm);
	
    		// Error is sum of cross product between estimated and measured direction of gravity
            err[XAXIS] = (az * gravRot[YAXIS] - ay * gravRot[ZAXIS]);// * accConfidence;
            err[YAXIS] = (ax * gravRot[ZAXIS] - az * gravRot[XAXIS]);// * accConfidence;
            err[ZAXIS] = (ay * gravRot[XAXIS] - ax * gravRot[YAXIS]);// * accConfidence;      
    		
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