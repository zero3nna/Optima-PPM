Copyright (c) 2012 Baseflight U.P.
Licensed under the MIT License
@author  Scott Driessens v0.1 (August 2012)

This is a combination of many flight controller projects that would not be 
possible without Timecop's work. Thank-you for making a decent codebase and 
flight controller so that I could experiment.

Ideas and source code have been derived from:
	- AeroQuad
	- Baseflight/Baseflight Plus/Baseflight Plus Dev
	- Multiwii
	- Madgwick/Mahony (AHRS)
	
This project attempts to make Baseflight more modular so that it can 
be ported to other platforms or used with a range of sensors. All sensor readings 
are expressed in SI units as floating points, support for new sensors should be as 
simple as providing the Init method and Read methods, which are responsible for updating
the scale factor.

There are still many things to finish porting and many things that may not work. 
At the moment it is flyable in rate mode and most of the interfaces are working.

What still needs to be done:
	- Finish multiwii serial protocol
		-Fix PID scaling for modes other than rate mode 
			- This should be easy to do once dynamic PID is implemented in Multiwii GUIs
	- Add Spektrum support
	- Add GPS support
	- Add FRSky telemetry
	- Add all LED modes
	- Write dynamic sensor detection - at the moment it selects between 
		MPU3050/ADXL345 and MPU6050 only
	- Get MPU6050 hardware working...just need to fix up sensor directions and scale factors.
	- Gimbal stabilisation
	- Dynamic feature selection - only have ppm and failsafe at the moment
	- Add altitude hold mode
	- Add Rates/Expo
	
THE SENSORS MUST BE PROPERLY CALIBRATED IN ORDER TO GET THE BEST RESULTS
- Gyro (RATE MODE) is calibrated the same way Multiwii handles it.
	- DISARMED, THROTTLE MIN, YAW LEFT, PITCH CENTRE
	- The flight controller must be still and stable
- For better results you should obtain the Gyro temperature compensation slope.
	- Using the CLI invoke 'calibrate gyro'. This process will take 15 minutes 
	because we are waiting for the board to heat up. You want the board 'cold' (it has been off for a while). 
	In normal use the temperature of the board will heat by a few degrees, what you want to do is expose 
	the board to the temperatures you will be flying in.
- Accelerometer is calibrated by placing the board still and untouched in 6 different positions
as instructed. Using the CLI invoke 'calibrate accel'. At the moment this cannot not be done after
calibrating the accelerometer runtime bias. I have been getting mixed results using this method
and may just remove it. Using the accelerometer runtime bias method seems to work just as well.
- Accelerometer runtime bias can be calculated with the CLI, 'calibrate accelRT' or just like Multiwii.
	- DISARMED, THROTTLE MAX, YAW MIN, PITCH MIN
	- Flight controller needs to be the right side up, level and still.
- Magnetometer bias is calculated by rotating the board around all axis within 10 seconds of invoking with:
	- Multwii gui
	- The CLI using 'calibrate mag'
	- Stick commands
		- DISARMED, THROTTLE MAX, YAW RIGHT, PITCH MIN
			
Level mode, heading hold mode and headfree mode use an IMU/AHRS, Simon Madgwick's implementation of Mahony's DCM filter with
magnetic drift compensation (optional). If you do not have a magnetometer or cannot guarantee the magnetometer is
free from magnetic distortion set the config value 'magDriftCompensation' to false. In order to use level or
head holding mode, all sensors MUST be calibrated and you MUST have a tuned and working RATE mode. The IMU/AHRS
has a PI controller for state estimation. Included is a python script (support/imu.py Razor IMU 9DOF visualiser) that can be used to
tune the AHRS. You need to put the flight controller in telemetry mode, CLI 'telemetry' then run the script after substituting
your serial port on the correct line. Just like tuning for flight, you will want to tune the IMU, set 
the options imuKp and imuKi in the CLI, then run the script and move it around, adjust gains and recheck.
When flying in a mode that uses the IMU/AHRS, it is likely that higher frequency oscillations are from the IMU and lower
frequency oscillations are from the control loop.

The flight controller has a series of periodic 'events'. If you are interested in changing the period of the events,
they are initialised in main.c before the main loop. At the moment, accelerometer and gyro sampling is done in 'idle' time, the main
attitude loop is updated at 300 HZ and the main actuator loop is updated at 200HZ. The cycle time variable is measuring the 
time between actuator updates. Events that are placed in the event queue first have a 'higher priority'.

This code has been setup to compile with the Codesourcery toolchain and the provided Makefile. If you have the toolchain
installed correctly as well as all dependencies then you will be fine. Invoking 'make install' will run stmloader which is included
in the support folder. The included version is precompiled for Mac OS X Lion, other unix users can build a version for themselves.
This is a newer version of the one Timecop has on baseflight SVN. I have now modified it so you can pass option -r to reset to
bootloader. If you ever 'brick' the flight controller, just short the bootloader pads when applying power and use 
stmloader without the option -r.

For all others, you should be able to very easily compile and flash with any of the methods that have been running around
the Baseflight forum, as long as you set the project up correctly in the IDE.

Some final points:
- Understand that this code is very new and has only had a handful of flights.
- At the moment I only trust it in Rate mode.
- The default gains are a good starting point for a small-medium sized quadrotor.
- All checkbox items can be configured in a Multiwii GUI for mode selection. Modes that aren't implemented should do nothing. 
- Because I liked jihlein's mixer mapping, this has been used instead of the default multiwii mapping. If the airframe type 
has a servo on it, the motor outputs will be offset by two. Motors are generally numbered 1-N the from the front left 
motor moving clockwise. Check out mixer.c for your airframe type to confirm. A bad side affect of this is the motor 
and servo outputs do not display correctly in the current GUIs and you will have to rearrange connections.
- You can use the Multiwii GUI to tune Rate mode only, the other PIDS are being scaled by some value by the GUI,
 and I haven't fixed them up yet as I'm waiting for dynamic PIDs to be implemented.
