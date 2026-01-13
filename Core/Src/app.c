/*
 * app.c
 *
 *  Created on: Aug 12, 2025
 *      Author: skylink
 */

/*
 * Helpers for UART1 <-> E32 and USB-CDC command/printf path
 * DMA-only RX policy: NEVER call HAL_UART_Receive() on USART1.
 */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "app.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* UART1 is defined in main.c */
extern UART_HandleTypeDef huart1;

/* If you didn't make usbd_cmd.h, these externs match your CDC RX helpers */
extern uint8_t CDC_Command_Available(void);
extern void    CDC_Command_Get(char *out, uint16_t maxlen);

/* ===== Ring Buffer API (main.c or uart_rx.c should exist) ===== */
extern int RB_ReadLine(char *out, uint16_t maxlen, uint32_t timeout_ms);
extern int RB_FindSequence(const uint8_t *seq, uint16_t seqlen, uint32_t timeout_ms);

/* ======================= Local CDC printf ======================= */

static void cdc_write(const uint8_t *buf, uint16_t len)
{
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
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    cdc_write((uint8_t*)buf, (uint16_t)n);
}

/* ======================= UART1 helpers (public) ======================= */

void sendByteUART(uint8_t b)
{
    HAL_UART_Transmit(&huart1, &b, 1, 100);
}

static void uart1_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 200);
}

void uart1_send_str(const char *s)
{
    uart1_send((const uint8_t*)s, (uint16_t)strlen(s));
}

/* ======================= E32 helpers (local) ======================= */

static uint8_t e32_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET) {
            HAL_Delay(3); /* guard time after AUX=HIGH */
            return 1;
        }
        HAL_Delay(1);
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

/* DMA-only: send cmd and wait for OK token in ring buffer */
static HAL_StatusTypeDef e32_send_at_wait_ok(const char *cmd, uint32_t timeout_ms)
{
    uart1_send_str(cmd);

    static const uint8_t ok_seq[] = {'O','K'};
    if (RB_FindSequence(ok_seq, sizeof(ok_seq), timeout_ms)) return HAL_OK;

    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef e32_query_line(const char *cmd, char *out, uint16_t outlen, uint32_t timeout_ms)
{
    if (out && outlen) out[0] = 0;
    uart1_send_str(cmd);

    if (!out || outlen == 0) return HAL_OK;

    if (RB_ReadLine(out, outlen, timeout_ms)) return HAL_OK;
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef e32_set_power_21dBm(void)
{
    HAL_StatusTypeDef st = e32_send_at_wait_ok("AT+POWER=3\r\n", 800);
    if (st == HAL_OK) {
        cdc_printf("[E32] POWER=3 OK\r\n");
        return HAL_OK;
    }
    cdc_printf("[E32] POWER=3 timeout/no OK\r\n");
    return st;
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
    if (e32_query_line("AT+POWER=?\r\n", rx, sizeof(rx), 800) == HAL_OK) {
        cdc_printf("[INIT] POWER?: %s", rx);
        if (rx[0] && rx[strlen(rx)-1] != '\n') cdc_printf("\r\n");
    } else {
        cdc_printf("[INIT] POWER? timeout\r\n");
    }
    e32_set_mode_normal();
}

/* ======================= Command loop over USB CDC ======================= */

void waitForCommand(void)
{
    if (!CDC_Command_Available()) return;

    char line[256];
    CDC_Command_Get(line, sizeof(line));

    char *cmd = strtok(line, " \t");
    char *arg = strtok(NULL, " \t");
    if (!cmd) return;

    if (strcmp(cmd, "list") == 0) {
        cdc_printf("[LIST] (stub) files: image1.bin, log.txt\r\n");
    }
    else if (strcmp(cmd, "sendfilename") == 0) {
        if (!arg) { cdc_printf("[ERR] usage: sendfilename <name>\r\n"); return; }

        cdc_printf("[SEND] Start: %s\r\n", arg);

        char hdr[64];
        int n = snprintf(hdr, sizeof(hdr), "AT+SEND %s\r\n", arg);
        if (n > 0) uart1_send((uint8_t*)hdr, (uint16_t)n);

        cdc_printf("[SEND] (stub) done\r\n");
    }
    else if (strcmp(cmd, "status") == 0) {
        cdc_printf("[STATUS] USART1=9600 8N1, USB CDC OK\r\n");

        /* DMA-only ping */
        uart1_send_str("AT\r\n");
        char rx[64];
        if (RB_ReadLine(rx, sizeof(rx), 400)) {
            cdc_printf("[STATUS] E32: %s", rx);
            if (rx[0] && rx[strlen(rx)-1] != '\n') cdc_printf("\r\n");
        } else {
            cdc_printf("[STATUS] E32: no reply\r\n");
        }
    }
    else {
        cdc_printf("[ERR] Unknown cmd: %s\r\n", cmd);
    }
}
