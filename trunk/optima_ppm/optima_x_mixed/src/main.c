/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include <board/optima.h>
#include <core/usart.h>

/* number of ticks in 20 ms - clock/1024 */
#define FAILSAFE		144
/* number of ticks in 400 us ppm pulse */
#define PPM_PRE_PULSE	2949
/* number of ticks in 4600 us ppm sync period */
#define PPM_SYNC_PERIOD	(33915 - PPM_PRE_PULSE)

/* State machine states */
enum states {START, PREAMBLE, PACKET, VERIFY};

static uint16_t inputs[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t buffer[2 * NUM_CHANNELS];
//static uint8_t failsafe = 0;

/* blocking delay */
#define delay(period) TIFR1 = (1 << OCF1A); TCNT1 = 0; OCR1A = period; while(!(TIFR1 & (1 << OCF1A)));
void ppm_handler(void);
void timer_init(void);

int main(void)
{	
	/* Initialisation here */
	timer_init();
	usart_init();
		
	sei();
	
	/* State variable */
	enum states state = START;
	uint8_t i;
	uint8_t byte;
	//uint8_t previous;
	
	/* initialise output - only on those that are available*/
	for(i = 0; i < OPTIMA; ++i) {
		*outputs[i].ddr |= outputs[i].mask;
	}
	
	/* main loop */
	for (;;) {
		/* We have a character, do something */
		if(!q_empty(usart_rx)) {
			byte = q_take(usart_rx);
			switch(state)
			{
				case START: // A packet is arriving
					if(byte == 0xFF) {
						state = PREAMBLE;
					}		
					break;
				case PREAMBLE: // The packet begins on the next byte
					if(byte == 0xFF) {
						state = PACKET;
						i = 0;
					} else {
						state = START;
					}
					break;
				case PACKET: // Fill the buffer with the packet values
					buffer[i++] = byte;
					if(i >= 2 * NUM_CHANNELS) {
						// We have received the channel data
						state = VERIFY;
					}
					break;
				case VERIFY: // Verify the packet has finished
					if(byte == 0xEE) {
						// Fill the input values with the adjusted channel timing
						for(i = 0; i < NUM_CHANNELS; ++i) {
							inputs[i] = ((uint16_t)(buffer[2*i] << 8) | (uint16_t)buffer[2*i+1]) - PPM_PRE_PULSE;
						}
						ppm_handler();
						//failsafe = 0;
					}
					// Fall through
				default:
					state = START;
			}
		} /*else if(failsafe > 49) { // failsafe if we have had no signal for 1s
			previous = failsafe;
			ppm_handler();
			failsafe = 50; // reset the counter
			
		}	*/
	}
}

void ppm_handler(void)
{
	uint8_t i;
	for(i = 0; i < PPM_CHANNELS; ++i) {
		/* ppm pre-pulse */
		*outputs[PPM].port |= outputs[PPM].mask;
		if(i < OPTIMA - 1)
			*outputs[i].port |= outputs[i].mask;
		delay(PPM_PRE_PULSE);
	
		/* end of pre-pulse */
		*outputs[PPM].port &= ~outputs[PPM].mask;
		delay(inputs[i]);
	
		/* end of channel timing */
		if(i < OPTIMA - 1)
			*outputs[i].port &= ~outputs[i].mask;
	}
	/* ppm sync pulse */
	*outputs[PPM].port |= outputs[PPM].mask;
	delay(PPM_PRE_PULSE);

	/* ppm sync period */
	*outputs[PPM].port &= ~outputs[PPM].mask;
	delay(PPM_SYNC_PERIOD);
}

void timer_init(void)
{
	/* Output compare for PPM/PWM generation */
	OCR1A = 0xFFFF;
	
	/* start TIMER1 with no prescaler */
	TCCR1B = ( 1 << CS10 );
	
	/*
	// TIMER 0 - 20 ms intterupts
	// Timer 0 CTC mode
	TCCR0A |= (1 << WGM01);
	
	// Enable CTC interrupt
	TIMSK0 |= (1 << OCIE0A);
	
	// Output compare 20 ms
	OCR0A = FAILSAFE;
	
	// Start timer at clock/1024
	TCCR0B |= ((1 << CS02) | (1 << CS00));
	*/
}
/*
ISR(TIMER0_COMPA_vect)
{
	++failsafe;
}
*/