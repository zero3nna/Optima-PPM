#ifndef __OPTIMA_H
#define __OPTIMA_H

#include <stdint.h>
#include <core/types.h>

/* Define the optima receiver */
#define OPTIMA			6
#define PPM				OPTIMA-1
#define MODE			0

/* Always 9 channels unless Hitec decides to change their protocol */
#define NUM_CHANNELS	9

extern pin_map outputs[NUM_CHANNELS];

#endif