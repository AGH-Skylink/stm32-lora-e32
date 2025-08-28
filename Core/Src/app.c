/*
 * app.c
 *
 *  Created on: Aug 12, 2025
 *      Author: skylink
 */


/*
 * app.c
 * Helpers for UART1 <-> E32 and USB-CDC command/printf path
 */

#include "main.h"
//#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "app.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
extern UART_HandleTypeDef huart1;   // defined in main.c
/* If you didn't make usbd_cmd.h, these externs match the functions
   you added in usbd_cdc_if.c for line-buffered CDC RX. */
// #include "usbd_cmd.h"
extern uint8_t CDC_Command_Available(void);
extern void    CDC_Command_Get(char *out, uint16_t maxlen);

/* ======================= Local CDC printf ======================= */

static void cdc_write(const uint8_t *buf, uint16_t len)
{
    /* CDC_Transmit_FS can return BUSY; retry briefly to avoid dropping logs. */
    for (int i = 0; i < 50; ++i) {
        if (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_OK) return;
        HAL_Delay(1);
    }
}

static void cdc_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    cdc_write((uint8_t*)buf, (uint16_t)n);
}

/* ======================= UART1 helpers (public) ======================= */

void sendByteUART(uint8_t b)
{
    HAL_UART_Transmit(&huart1, &b, 1, HAL_MAX_DELAY);
}

static void uart1_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
}

void uart1_send_str(const char *s)
{
    uart1_send((const uint8_t*)s, (uint16_t)strlen(s));
}

/* ======================= E32 helpers (local) ======================= */
/* Assumes pins are defined in CubeMX -> main.h:
   E32_M0_GPIO_Port/E32_M0_Pin, E32_M1_*, E32_AUX_*  */

static uint8_t e32_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET) {
            HAL_Delay(3); /* guard time after AUX=HIGH per datasheet */
            return 1;
        }
    }
    return 0;
}

static void e32_set_mode_config(void)
{
    HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
    (void)e32_wait_aux_high(500);
}

static void e32_set_mode_normal(void)
{
    HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    (void)e32_wait_aux_high(500);
}

static HAL_StatusTypeDef e32_send_at_wait(const char *cmd, char *reply, uint16_t maxlen, uint32_t timeout_ms)
{
    if (reply && maxlen) reply[0] = 0;

    uart1_send_str(cmd);

    if (!reply || maxlen == 0) return HAL_OK;

    uint16_t idx = 0;
    uint8_t ch;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (HAL_UART_Receive(&huart1, &ch, 1, 20) == HAL_OK) {
            if (idx < (uint16_t)(maxlen - 1)) reply[idx++] = (char)ch;
            if (ch == '\n') break;
        }
    }
    reply[idx] = 0;
    return (idx > 0) ? HAL_OK : HAL_TIMEOUT;
}

static HAL_StatusTypeDef e32_set_power_21dBm(void)
{
    char rx[48];
    HAL_StatusTypeDef st = e32_send_at_wait("AT+POWER=3\r\n", rx, sizeof(rx), 600);
    if (st == HAL_OK) {
        cdc_printf("[E32] Reply: %s", rx);
        if (strstr(rx, "OK") || strstr(rx, "SUCCESS")) return HAL_OK;
        return HAL_ERROR;
    } else {
        cdc_printf("[E32] No reply to AT+POWER=3\r\n");
        return HAL_TIMEOUT;
    }
}

/* ======================= Public init ======================= */

void e32_basic_init(void)
{
    cdc_printf("[INIT] E32 @9600 init...\r\n");
    e32_set_mode_config();
    if (e32_set_power_21dBm() == HAL_OK) {
        cdc_printf("[INIT] Power set to 21 dBm.\r\n");
    } else {
        cdc_printf("[INIT] Failed to set power.\r\n");
    }
    e32_set_mode_normal();

    /* Optional readback */
    char rx[64];
    e32_set_mode_config();
    if (e32_send_at_wait("AT+POWER=?\r\n", rx, sizeof(rx), 600) == HAL_OK) {
        cdc_printf("[INIT] POWER?: %s", rx);
    } else {
        cdc_printf("[INIT] POWER? no reply\r\n");
    }
    e32_set_mode_normal();
}

/* ======================= Command loop over USB CDC ======================= */

void waitForCommand(void)
{
    if (!CDC_Command_Available()) return;

    char line[256];
    CDC_Command_Get(line, sizeof(line));

    /* Tokenize: cmd [arg] */
    char *cmd = strtok(line, " \t");
    char *arg = strtok(NULL, " \t");
    if (!cmd) return;

    if (strcmp(cmd, "list") == 0) {
        /* Placeholder: hook Magda's serialization/file listing here */
        cdc_printf("[LIST] (stub) files: image1.bin, log.txt\r\n");
    }
    else if (strcmp(cmd, "sendfilename") == 0) {
        if (!arg) { cdc_printf("[ERR] usage: sendfilename <name>\r\n"); return; }

        cdc_printf("[SEND] Start: %s\r\n", arg);

        /* Example: send a small header toward the remote over UART1 */
        char hdr[64];
        int n = snprintf(hdr, sizeof(hdr), "AT+SEND %s\r\n", arg);
        if (n > 0) uart1_send((uint8_t*)hdr, (uint16_t)n);

        /* TODO: open file, read chunks, uart1_send(...) per chunk */
        cdc_printf("[SEND] (stub) done\r\n");
    }
    else if (strcmp(cmd, "status") == 0) {
        cdc_printf("[STATUS] USART1=9600 8N1, USB CDC OK\r\n");

        /* Ping E32 with plain "AT" */
        uart1_send_str("AT\r\n");
        uint8_t ch;
        char rx[48]; uint16_t idx = 0;
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 300) {
            if (HAL_UART_Receive(&huart1, &ch, 1, 20) == HAL_OK) {
                if (idx < sizeof(rx) - 1) rx[idx++] = (char)ch;
                if (ch == '\n') break;
            }
        }
        rx[idx] = 0;
        if (idx) cdc_printf("[STATUS] E32: %s", rx);
        else     cdc_printf("[STATUS] E32: no reply\r\n");
    }
    else {
        cdc_printf("[ERR] Unknown cmd: %s\r\n", cmd);
    }
}
