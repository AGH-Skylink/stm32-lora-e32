/*
 * e32.c
 *
 *  Created on: Aug 11, 2025
 *      Author: skylink
 */


#include "e32.h"

// Pin definitions (match .ioc)
#define E32_M0_GPIO_Port GPIOB
#define E32_M0_Pin       GPIO_PIN_0
#define E32_M1_GPIO_Port GPIOB
#define E32_M1_Pin       GPIO_PIN_1
#define E32_AUX_GPIO_Port GPIOB
#define E32_AUX_Pin      GPIO_PIN_2

static UART_HandleTypeDef *huart_e32;
static UART_HandleTypeDef *huart_pc;

static void pc_print(const char *s) {
    HAL_UART_Transmit(huart_pc, (uint8_t*)s, strlen(s), 100);
}

void E32_Init(UART_HandleTypeDef *e32_uart, UART_HandleTypeDef *pc_uart) {
    huart_e32 = e32_uart;
    huart_pc  = pc_uart;
}

void E32_SetModeNormal(void) {
    HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_RESET);
}

void E32_SetModeConfig(void) {
    HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_SET);
}

bool E32_WaitAUX_High(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET)
            return true;
    }
    return false;
}

void E32_SendAT(const char *cmd) {
    HAL_UART_Transmit(huart_e32, (uint8_t*)cmd, strlen(cmd), 100);
}

void E32_QueryPower(void) {
    const char *cmd = "AT+POWER=?\r\n";
    char buf[64] = {0};
    size_t idx = 0;
    uint8_t ch;

    E32_SetModeConfig();
    E32_WaitAUX_High(100);

    HAL_UART_Transmit(huart_e32, (uint8_t*)cmd, strlen(cmd), 100);

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 600) {
        if (HAL_UART_Receive(huart_e32, &ch, 1, 20) == HAL_OK) {
            if (idx < sizeof(buf) - 1) buf[idx++] = ch;
            if (ch == '\n') break;
        }
    }

    pc_print("E32 reply: ");
    pc_print(buf);
    pc_print("\r\n");

    E32_SetModeNormal();
}
