/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include <core/usart.h>

volatile struct cqueue * usart_rx;

volatile struct cqueue usart_rx_s;

void usart_init()
{
	/* set up receive queue */
	usart_rx = &usart_rx_s;
	usart_rx->front = USART_BUFF/2;
	usart_rx->back = USART_BUFF/2;
	
	/* Asynchronous, no parity, 1 stop bit, 8 bit character size */
	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);
	
	/* Set baud rate */
	UBRR0H = (uint8_t)(MYUBR>>8);
	UBRR0L = (uint8_t)MYUBR;
	
	/* Enable receive, receive complete interrupt*/
	UCSR0B = (1<<RXEN0)|(1<<RXCIE0);
}

ISR(USART_RX_vect)
{
	uint8_t ch = UDR0;
	q_put(usart_rx, ch);
}

void q_put(volatile struct cqueue *q, uint8_t ch)
{
	/* if buffer overflows the queue automatically resets -> back == front */
	q->array[q->back++] = ch;
	while (q->back >= USART_BUFF) q->back -= USART_BUFF;
}

/* it is assumed that you will not take from the queue unless it has a value */
uint8_t q_take(volatile struct cqueue *q)
{
	uint8_t temp = q->array[q->front++];
	while (q->front >= USART_BUFF) q->front -= USART_BUFF;
	return temp;
}

uint8_t q_at(volatile struct cqueue *q, uint16_t idx)
{
	uint16_t iidx = q->front + idx;
	while (iidx >= USART_BUFF) iidx -= USART_BUFF;
	return q->array[iidx];
}

void q_gobble(volatile struct cqueue *q, uint16_t n)
{
	q->front += n;
	while (q->front >= USART_BUFF) q->front -= USART_BUFF;
}

uint16_t q_find(volatile struct cqueue *q, uint8_t val)
{
	uint16_t idx = q->front;
	for (; idx != q->back && idx <= USART_BUFF; idx++) {
		while (idx >= USART_BUFF) idx -= USART_BUFF;
		if (q->array[idx] == val) return 1;
	}
	return 0;
}

uint16_t q_used(volatile struct cqueue *q)
{
	uint16_t idx = q->back;
	if (idx < q->front)
		idx += USART_BUFF;
	return (idx - q->front);
}