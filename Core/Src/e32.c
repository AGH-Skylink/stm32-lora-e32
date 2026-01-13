/*
 * e32.c
 *
 *  Created on: Aug 11, 2025
 *      Author: skylink
 *
 *  DMA-only policy:
 *   - Do NOT call HAL_UART_Receive() here (RX는 main.c 링버퍼에서 처리)
 *   - Only TX + mode pins + AUX wait helpers
 */

#include "e32.h"
#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static UART_HandleTypeDef *huart_e32 = NULL;
static UART_HandleTypeDef *huart_pc  = NULL;

/* Optional: PC log (if pc uart is provided) */
static void pc_print(const char *s)
{
    if (!huart_pc || !s) return;
    (void)HAL_UART_Transmit(huart_pc, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

void E32_Init(UART_HandleTypeDef *e32_uart, UART_HandleTypeDef *pc_uart)
{
    huart_e32 = e32_uart;
    huart_pc  = pc_uart;
}

void E32_SetModeNormal(void)
{
    HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_RESET);
}

void E32_SetModeConfig(void)
{
    HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_SET);
}

bool E32_WaitAUX_High(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(3); /* guard time after AUX high */
            return true;
        }
        HAL_Delay(1);
    }
    return false;
}

/* Raw TX helper */
static HAL_StatusTypeDef e32_tx(const uint8_t *data, uint16_t len, uint32_t to)
{
    if (!huart_e32 || !data || len == 0) return HAL_ERROR;
    return HAL_UART_Transmit(huart_e32, (uint8_t*)data, len, to);
}

/* If you still want an "AT send" helper, keep it TX-only */
void E32_SendAT(const char *cmd)
{
    if (!cmd) return;
    (void)e32_tx((const uint8_t*)cmd, (uint16_t)strlen(cmd), 100);
}

/*
 * NOTE:
 *  - Query/Read functions that require RX are intentionally removed in DMA-only design.
 *  - Use main.c ring buffer functions (RB_ReadLine / RB_ReadExact / RB_FindSequence)
 *    to parse responses in main loop or app.c command handler.
 */
void E32_QueryPower(void)
{
    /* Keep a safe stub so existing code compiles, but do not RX here. */
    pc_print("[E32] QueryPower called - RX handled via DMA ring buffer\r\n");
    /* Example TX (optional): */
    // E32_SetModeConfig();
    // (void)E32_WaitAUX_High(200);
    // E32_SendAT("AT+POWER=?\r\n");
    // E32_SetModeNormal();
}
