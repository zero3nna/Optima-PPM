/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#ifndef __TYPES_H
#define __TYPES_H

typedef struct pin_map {
	uint8_t mask;
	volatile uint8_t *pin;
	volatile uint8_t *port;
	volatile uint8_t *ddr;
} pin_map;

typedef void (*output_callback)(void);
	
#endif