#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include <board/optima.h>
#include <core/usart.h>

/* number of PPM channels */
#define PPM_CHANNELS	8
/* number of ticks in 400 us ppm pulse */
#define PPM_PRE_PULSE	2949
/* number of ticks in 4600 us ppm sync period */
#define PPM_SYNC_PERIOD	(33915 - PPM_PRE_PULSE)

/* State machine states */
enum states {START, PREAMBLE, PACKET, VERIFY};
static uint8_t ppm_flag = 0;

/* blocking delay */
#define delay(period) TIFR1 = (1 << OCF1A); TCNT1 = 0; OCR1A = period; while(!(TIFR1 & (1 << OCF1A)));
void mode_setup(void);
void timer_init(void);

int main(void)
{	
	/* Initialisation here */
	timer_init();
	mode_setup();
	usart_init();
	
	/* State variable */
	enum states state = START;
	uint8_t i;
	uint8_t byte;
	uint8_t buffer[2 * NUM_CHANNELS];
	uint16_t inputs[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	sei();
	
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
						if(ppm_flag) { /******************************************** PPM ***/
							// Fill the input values with the adjusted channel timing
							for(i = 0; i < NUM_CHANNELS; ++i) {
								inputs[i] = ((uint16_t)(buffer[2*i] << 8) | (uint16_t)buffer[2*i+1]) - PPM_PRE_PULSE;
							}
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
						} else { /******************************************** PWM **/
							// Fill the input values with the channel timing
							for(i = 0; i < NUM_CHANNELS; ++i) {
								inputs[i] = ((uint16_t)(buffer[2*i] << 8) | (uint16_t)buffer[2*i+1]);
							}
							for(i = 0; i < NUM_CHANNELS; ++i) {
								/* pwm delta */
								*outputs[i].port |= outputs[i].mask;
								delay(inputs[i]);
								*outputs[i].port &= ~outputs[i].mask;
							}	
						}
					}
					// Fall through
				default:
					state = START;
			}
		}	
	}
}

void mode_setup(void)
{
	uint8_t i;
	uint16_t j;
	
	*outputs[MODE].port |= outputs[MODE].mask;
	
	delay(737);
	for(j = 0; j < 50000; ++j) {
		if(!(*outputs[MODE].pin & outputs[MODE].mask))
			ppm_flag = 1;
	}
	
	*outputs[MODE].port &= ~outputs[MODE].mask;
	
	if(ppm_flag) {
		/* ppm mode */
		*outputs[PPM].ddr |= outputs[PPM].mask;
	} else {
		/* pwm mode */
		/* initialise output - only on those that are available*/
		for(i = 0; i < OPTIMA; ++i) {
			*outputs[i].ddr |= outputs[i].mask;
		}
	}
	
}

void timer_init(void)
{
	OCR1A = 0xFFFF;
	/* start TIMER1 with no prescaler */
	TCCR1B = ( 1 << CS10 );
}