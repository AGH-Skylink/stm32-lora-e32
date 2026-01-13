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

uint16_t oldPos = 0;
uint16_t newPos = 0;

int isOK = 0;
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
static HAL_StatusTypeDef E32_ReadBytes(uint8_t *buf, uint16_t len, uint32_t to);
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
  }
  return 0;
}

static HAL_StatusTypeDef E32_SendFrame(const uint8_t *data, uint16_t len, uint32_t to) {
  return HAL_UART_Transmit(&huart1, (uint8_t*)data, len, to);
}

static HAL_StatusTypeDef E32_ReadBytes(uint8_t *buf, uint16_t len, uint32_t to) {
  return HAL_UART_Receive(&huart1, buf, len, to);
}

void myprint(const char *text) {
  CDC_Transmit_FS((uint8_t*)text, strlen(text));
}

/* DMA Ring Buffer Callback */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    oldPos = newPos;  // Update the last position before copying new data

    /* If the data is large and it is about to exceed the buffer size, 
     * we have to route it to the start of the buffer (circular buffer)
     */
    if (oldPos + Size > MainBuf_SIZE)
    {
      uint16_t datatocopy = MainBuf_SIZE - oldPos;
      memcpy((uint8_t *)MainBuf + oldPos, RxBuf, datatocopy);

      oldPos = 0;
      memcpy((uint8_t *)MainBuf, (uint8_t *)RxBuf + datatocopy, (Size - datatocopy));
      newPos = (Size - datatocopy);
    }
    else
    {
      memcpy((uint8_t *)MainBuf + oldPos, RxBuf, Size);
      newPos = Size + oldPos;
    }

    /* Restart the DMA */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    /****************** PROCESS THE DATA HERE *********************/
    /* Example: Check for "OK" keyword in incoming data */
    for (int i = 0; i < Size; i++)
    {
      if ((RxBuf[i] == 'O') && (RxBuf[i+1] == 'K'))
      {
        isOK = 1;
        myprint("Received OK!\r\n");
      }
    }
  }
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

  // Enter CONFIG mode
  myprint("Entering CONFIG mode...\r\n");
  E32_SetModeConfig();
  HAL_Delay(60);
  E32_WaitAUX_High(1000);

  // Send "Read Parameters" command: C1 C1 C1
  const uint8_t cmd_read_params[] = {0xC1, 0xC1, 0xC1};
  uint8_t resp[6] = {0};
  myprint("Sending: C1 C1 C1\r\n");

  if (E32_SendFrame(cmd_read_params, sizeof(cmd_read_params), 1000) == HAL_OK &&
      E32_ReadBytes(resp, sizeof(resp), 1000) == HAL_OK)
  {
    char line[96];
    snprintf(line, sizeof(line), "Params reply: %02X %02X %02X %02X %02X %02X\r\n",
             resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
    myprint(line);
  } else {
    myprint("No reply from module!\r\n");
  }

  // Back to NORMAL mode
  E32_SetModeNormal();
  myprint("Back to NORMAL mode.\r\n");

  // Start DMA Ring Buffer Reception
  myprint("Starting DMA reception...\r\n");
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* USER CODE END 2 */

  while (1) {
    HAL_Delay(1000);
    
    // You can add periodic tasks here
    // For example, send data or check ring buffer
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
