/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#ifndef __USART_H
#define __USART_H

#include <stdint.h>

#define BAUD 115200L
#define MYUBR (CLOCK/16L/BAUD-1)

#define USART_BUFF              256

struct cqueue {
  uint16_t front;
  uint16_t back;
  uint8_t array[USART_BUFF];
};

extern volatile struct cqueue * usart_rx;

/* queue ADT functions */
/* some of these are macros to save space */
#define q_empty(q)    (q->back == q->front)
void q_put(volatile struct cqueue *q, uint8_t ch);
void q_gobble(volatile struct cqueue *q, uint16_t n);
uint8_t q_take(volatile struct cqueue *q);
uint16_t q_find(volatile struct cqueue *q, uint8_t value);
uint16_t q_used(volatile struct cqueue *q);
uint8_t q_at(volatile struct cqueue *q, uint16_t idx);
#define q_reset(q)    do { q->back = 0; q->front = 0; } while(0)
#define q_peek(q)     (q->array[q->back])

void usart_init(void);

#endif