/**
  ******************************************************************************
  * @file           : RingBuffer.h
  * @brief          : Ring Buffer utility functions 
  ******************************************************************************
  */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "main.h"
#include <stdint.h>

/* Ring Buffer Structure */
typedef struct {
  uint8_t *buffer;
  uint16_t head;
  uint16_t tail;
  uint16_t size;
} RingBuffer_t;

/* Function Prototypes */
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size);
uint8_t RingBuffer_Write(RingBuffer_t *rb, uint8_t data);
uint8_t RingBuffer_Read(RingBuffer_t *rb, uint8_t *data);
uint16_t RingBuffer_Available(RingBuffer_t *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer_t *rb);
uint8_t RingBuffer_IsFull(RingBuffer_t *rb);
void RingBuffer_Flush(RingBuffer_t *rb);
uint16_t RingBuffer_ReadBytes(RingBuffer_t *rb, uint8_t *data, uint16_t len);

#endif /* RINGBUFFER_H_ */
