/**
  ******************************************************************************
  * @file           : RingBuffer.c
  * @brief          : Ring Buffer implementation 
  ******************************************************************************
  */

#include "RingBuffer.h"

/* Initialize Ring Buffer */
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size) {
  rb->buffer = buffer;
  rb->size = size;
  rb->head = 0;
  rb->tail = 0;
}

/* Write one byte to Ring Buffer */
uint8_t RingBuffer_Write(RingBuffer_t *rb, uint8_t data) {
  uint16_t next_head = (rb->head + 1) % rb->size;
  
  if (next_head == rb->tail) {
    return 0; // Buffer full
  }
  
  rb->buffer[rb->head] = data;
  rb->head = next_head;
  return 1;
}

/* Read one byte from Ring Buffer */
uint8_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data) {
  if (rb->head == rb->tail) {
    return 0; // Buffer empty
  }
  
  *data = rb->buffer[rb->tail];
  rb->tail = (rb->tail + 1) % rb->size;
  return 1;
}

/* Get available data count */
uint16_t RingBuffer_Available(RingBuffer_t *rb) {
  if (rb->head >= rb->tail) {
    return rb->head - rb->tail;
  } else {
    return rb->size - rb->tail + rb->head;
  }
}

/* Check if buffer is empty */
uint8_t RingBuffer_IsEmpty(RingBuffer_t *rb) {
  return (rb->head == rb->tail);
}

/* Check if buffer is full */
uint8_t RingBuffer_IsFull(RingBuffer_t *rb) {
  return ((rb->head + 1) % rb->size == rb->tail);
}

/* Flush the buffer */
void RingBuffer_Flush(RingBuffer_t *rb) {
  rb->head = 0;
  rb->tail = 0;
}

/* Read multiple bytes */
uint16_t RingBuffer_ReadBytes(RingBuffer_t *rb, uint8_t *data, uint16_t len) {
  uint16_t count = 0;
  
  while (count < len && !RingBuffer_IsEmpty(rb)) {
    if (RingBuffer_Read(rb, &data[count])) {
      count++;
    }
  }
  
  return count;
}
