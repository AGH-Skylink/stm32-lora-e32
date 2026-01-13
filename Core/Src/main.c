/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : E32-900T30S with DMA Ring Buffer UART Reception
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usb_device.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
#define RxBuf_SIZE   512
#define MainBuf_SIZE 2048

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];

/* DMA-only ring buffer pointers */
static volatile uint16_t writePos = 0;  // ISR에서 증가
static volatile uint16_t readPos  = 0;  // main loop에서 증가

/* Optional status flags */
static volatile int ok_print_pending = 0;
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void E32_SetModeNormal(void);
static void E32_SetModeConfig(void);
static uint8_t E32_WaitAUX_High(uint32_t timeout_ms);
static HAL_StatusTypeDef E32_SendFrame(const uint8_t *data, uint16_t len, uint32_t to);
/* NOTE: E32_ReadBytes() (blocking receive) is intentionally NOT used in DMA-only RX design */

static inline uint16_t rb_next(uint16_t p);
static uint16_t RB_Available(void);
static int RB_GetByte(uint8_t *out);
static int RB_ReadExact(uint8_t *dst, uint16_t len, uint32_t timeout_ms);
int RB_ReadLine(char *out, uint16_t maxlen, uint32_t timeout_ms);
int RB_FindSequence(const uint8_t *seq, uint16_t seqlen, uint32_t timeout_ms);

static int E32_ReadParams_DMA(uint8_t resp6[6], uint32_t timeout_ms);

void myprint(const char *text);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void E32_SetModeNormal(void) {
  HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_RESET);
}

static void E32_SetModeConfig(void) {
  HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, GPIO_PIN_SET);
}

static uint8_t E32_WaitAUX_High(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeout_ms) {
    if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET) return 1;
    HAL_Delay(1);
  }
  return 0;
}

static HAL_StatusTypeDef E32_SendFrame(const uint8_t *data, uint16_t len, uint32_t to) {
  return HAL_UART_Transmit(&huart1, (uint8_t*)data, len, to);
}

void myprint(const char *text) {
  /* NOTE: do NOT call this from UART RX callback (ISR context) */
  CDC_Transmit_FS((uint8_t*)text, strlen(text));
}

/* ================= DMA-only Ring Buffer Utils ================= */

static inline uint16_t rb_next(uint16_t p) { return (uint16_t)((p + 1u) % MainBuf_SIZE); }

static uint16_t RB_Available(void)
{
  uint16_t w = writePos, r = readPos;
  return (w >= r) ? (uint16_t)(w - r) : (uint16_t)(MainBuf_SIZE - r + w);
}

static int RB_GetByte(uint8_t *out)
{
  if (readPos == writePos) return 0;
  *out = MainBuf[readPos];
  readPos = rb_next(readPos);
  return 1;
}

static int RB_ReadExact(uint8_t *dst, uint16_t len, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  uint16_t got = 0;

  while (got < len)
  {
    uint8_t b;
    if (RB_GetByte(&b))
    {
      dst[got++] = b;
      continue;
    }
    if ((HAL_GetTick() - t0) >= timeout_ms) return 0;
    HAL_Delay(1);
  }
  return 1;
}

/* Read until '\n' (included) or timeout. Returns 1 if got any char. */
int RB_ReadLine(char *out, uint16_t maxlen, uint32_t timeout_ms)
{
  if (!out || maxlen < 2) return 0;

  uint32_t t0 = HAL_GetTick();
  uint16_t idx = 0;

  while ((HAL_GetTick() - t0) < timeout_ms)
  {
    uint8_t b;
    if (RB_GetByte(&b))
    {
      if (idx < (uint16_t)(maxlen - 1)) out[idx++] = (char)b;
      if (b == '\n') break;
      continue;
    }
    HAL_Delay(1);
  }

  out[idx] = 0;
  return (idx > 0);
}

/* Find a byte sequence within timeout, consuming bytes as it searches. */
int RB_FindSequence(const uint8_t *seq, uint16_t seqlen, uint32_t timeout_ms)
{
  if (!seq || seqlen == 0) return 0;

  uint32_t t0 = HAL_GetTick();
  uint16_t matched = 0;

  while ((HAL_GetTick() - t0) < timeout_ms)
  {
    uint8_t b;
    if (!RB_GetByte(&b)) {
      HAL_Delay(1);
      continue;
    }

    if (b == seq[matched])
    {
      matched++;
      if (matched == seqlen) return 1;
    }
    else
    {
      /* simple fallback: restart match; handle overlap minimally */
      matched = (b == seq[0]) ? 1 : 0;
    }
  }
  return 0;
}

/* DMA-based read params: send C1 C1 C1 and read 6 bytes from ring buffer */
static int E32_ReadParams_DMA(uint8_t resp6[6], uint32_t timeout_ms)
{
  const uint8_t cmd_read_params[] = {0xC1, 0xC1, 0xC1};

  /* flush pending RX (optional) */
  readPos = writePos;

  if (E32_SendFrame(cmd_read_params, sizeof(cmd_read_params), 1000) != HAL_OK)
    return 0;

  return RB_ReadExact(resp6, 6, timeout_ms);
}

/* DMA Ring Buffer Callback: push only, NO printing, NO heavy parsing */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART1) return;

  /* push RxBuf[0..Size-1] into MainBuf ring */
  for (uint16_t i = 0; i < Size; i++)
  {
    MainBuf[writePos] = RxBuf[i];
    writePos = rb_next(writePos);

    /* overflow policy: drop oldest */
    if (writePos == readPos)
    {
      readPos = rb_next(readPos);
    }
  }

  /* restart DMA last */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  myprint("STM32 boot OK (E32 + DMA Ring Buffer)\r\n");

  /* Start DMA reception ASAP (DMA-only RX policy) */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  // Enter CONFIG mode
  myprint("Entering CONFIG mode...\r\n");
  E32_SetModeConfig();
  HAL_Delay(60);
  E32_WaitAUX_High(1000);

  // Read parameters via DMA ring buffer
  uint8_t resp[6] = {0};
  myprint("Sending: C1 C1 C1\r\n");

  if (E32_ReadParams_DMA(resp, 1000))
  {
    char line[96];
    snprintf(line, sizeof(line), "Params reply: %02X %02X %02X %02X %02X %02X\r\n",
             resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
    myprint(line);
  }
  else
  {
    myprint("Params read timeout/no reply\r\n");
  }

  // Back to NORMAL mode
  E32_SetModeNormal();
  myprint("Back to NORMAL mode.\r\n");

  /* Example: watch for OK token in background (optional) */
  /* USER CODE END 2 */

  while (1) {
    /* USER CODE BEGIN 3 */

    /* Example background parse: find "OK" in stream (non-blocking-ish with short timeout) */
    static const uint8_t ok_seq[] = {'O','K'};
    if (RB_FindSequence(ok_seq, sizeof(ok_seq), 5)) {
      ok_print_pending = 1;
    }

    if (ok_print_pending) {
      ok_print_pending = 0;
      myprint("Received OK!\r\n");
    }

    HAL_Delay(10);
    /* USER CODE END 3 */
  }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
}

/* DMA Init Function */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration (USART1_RX) */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600; // default for E32
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

/* GPIO init function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USART1 PA9 (TX), PA10 (RX) */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* M0, M1 outputs */
  HAL_GPIO_WritePin(GPIOB, E32_M0_Pin|E32_M1_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = E32_M0_Pin|E32_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* AUX input with pull-up */
  GPIO_InitStruct.Pin = E32_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E32_AUX_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
