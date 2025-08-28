/*
 * e32.h
 *
 *  Created on: Aug 11, 2025
 *      Author: skylink
 */

/*
 * e32.h
 * E32 LoRa module driver header
 * Created on: Aug 11, 2025
 * Author: skylink
 */

#ifndef __E32_H
#define __E32_H

#include "stm32f4xx_hal.h"   // ensures UART_HandleTypeDef is known
#include <stdbool.h>
#include <string.h>

// Public API
void E32_Init(UART_HandleTypeDef *e32_uart, UART_HandleTypeDef *pc_uart);
void E32_SetModeNormal(void);
void E32_SetModeConfig(void);
bool E32_WaitAUX_High(uint32_t timeout_ms);
void E32_SendAT(const char *cmd);
void E32_QueryPower(void);

#endif
