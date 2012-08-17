/*
 * Copyright (c) 2012 Scott Driessens
 * Licensed under the MIT License
 */
 
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

void updateAttitude(void);