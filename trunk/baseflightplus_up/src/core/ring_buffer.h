/*
 * Copyright (c) 2012 Baseflight U.P.
 * Licensed under the MIT License
 * @author  Scott Driessens v0.1 (August 2012)
 *
 * Based on https://github.com/pthrasher/c-generic-ring-buffer
 * Philip Thrasher's Crazy Awesome Ring Buffer Macros!
 *
 * Below you will find some naughty macros for easy owning and manipulating
 * generic ring buffers. Yes, they are slightly evil in readability, but they
 * are really fast, and they work great.
 *
 * Example usage:
 *
 * // So we can use this in any method, this gives us a typedef
 * // named 'intBuffer'.
 * ringBuffer_typedef(int, intBuffer);
 *
 * int main() {
 *   // Declare vars.
 *   intBuffer myBuffer;
 *   intBuffer* myBuffer_ptr;
 *   myBuffer_ptr = &myBuffer;
 * 
 *   int buffer[128];
 *
 *   bufferInit(myBuffer_ptr,buffer,128);
 *
 *   // Write two values.
 *   bufferWrite(myBuffer_ptr,37);
 *   bufferWrite(myBuffer_ptr,72);
 *
 *   // Read a value into a local variable.
 *   int first;
 *   bufferRead(myBuffer_ptr,first);
 *   assert(first == 37); // true
 *
 *   int second;
 *   bufferRead(myBuffer_ptr,second);
 *   assert(second == 72); // true
 *
 *   return 0;
 * }
 */

#ifndef _ringbuffer_h
#define _ringbuffer_h

#define ringBuffer_typedef(buffer_type, name) \
  typedef struct { \
    uint8_t size; \
    uint8_t start; \
    uint8_t count; \
    buffer_type* elems; \
  } name

#define bufferInit(cb, buf_ptr, buf_len) \
  cb->size = buf_len; \
  cb->start = 0; \
  cb->count = 0; \
  cb->elems = buf_ptr

#define nextStartIndex(cb) ((cb->start + 1) % cb->size)
#define nextEndIndex(cb) ((cb->start + cb->count) % cb->size)
#define bufferUsed(cb)  cb->count

/* Write an element, overwriting oldest element if buffer is full. App can
   choose to avoid the overwrite by checking cbIsFull(). */
#define bufferWrite(cb, elem) \
    cb->elems[nextEndIndex(cb)] = *elem; \
    if(cb->count == cb->size) \
        cb->start = nextStartIndex(cb); \
    else \
        ++(cb->count);

/* Read oldest element. App must ensure !cbIsEmpty() first. */
#define bufferRead(cb, elem) \
    *elem = cb->elems[cb->start]; \
    cb->start = nextStartIndex(cb); \
    --(cb->count)
    
#endif