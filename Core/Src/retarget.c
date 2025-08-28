/*
 * retarget.c
 *
 *  Created on: Aug 11, 2025
 *      Author: skylink
 */


#include "stm32f4xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
