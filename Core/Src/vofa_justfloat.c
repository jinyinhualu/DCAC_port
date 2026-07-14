/**
 * @file vofa_justfloat.c
 * @brief Sends floating-point values in VOFA+ JustFloat frames.
 */
#include "vofa_justfloat.h"

#include <string.h>

#define VOFA_FRAME_SIZE      8U
#define VOFA_TX_BUFFER_SIZE  8192U

static UART_HandleTypeDef *vofa_uart = NULL;
static uint8_t vofa_tx_buffer[VOFA_TX_BUFFER_SIZE];
static volatile uint16_t vofa_tx_head = 0U;
static volatile uint16_t vofa_tx_tail = 0U;
static volatile uint16_t vofa_tx_len = 0U;
static volatile uint8_t vofa_tx_busy = 0U;

static uint16_t VOFA_RingUsed(void)
{
  if (vofa_tx_head >= vofa_tx_tail)
  {
    return (uint16_t)(vofa_tx_head - vofa_tx_tail);
  }

  return (uint16_t)(VOFA_TX_BUFFER_SIZE - vofa_tx_tail + vofa_tx_head);
}

static uint16_t VOFA_RingFree(void)
{
  return (uint16_t)(VOFA_TX_BUFFER_SIZE - VOFA_RingUsed() - 1U);
}

static uint16_t VOFA_RingContiguousUsed(void)
{
  if (vofa_tx_head >= vofa_tx_tail)
  {
    return (uint16_t)(vofa_tx_head - vofa_tx_tail);
  }

  return (uint16_t)(VOFA_TX_BUFFER_SIZE - vofa_tx_tail);
}

void VOFA_JustFloat_Init(UART_HandleTypeDef *huart)
{
  __disable_irq();
  vofa_uart = huart;
  vofa_tx_head = 0U;
  vofa_tx_tail = 0U;
  vofa_tx_len = 0U;
  vofa_tx_busy = 0U;
  __enable_irq();
}

static void VOFA_TryStartTx(void)
{
  uint16_t length;
  uint16_t tail;

  if (vofa_uart == NULL)
  {
    return;
  }

  __disable_irq();
  if ((vofa_tx_busy != 0U) || (vofa_tx_head == vofa_tx_tail))
  {
    __enable_irq();
    return;
  }

  length = VOFA_RingContiguousUsed();
  tail = vofa_tx_tail;
  vofa_tx_len = length;
  vofa_tx_busy = 1U;
  __enable_irq();

  if (HAL_UART_Transmit_DMA(vofa_uart, &vofa_tx_buffer[tail], length) != HAL_OK)
  {
    __disable_irq();
    vofa_tx_len = 0U;
    vofa_tx_busy = 0U;
    __enable_irq();
  }
}

HAL_StatusTypeDef VOFA_JustFloat_Send(UART_HandleTypeDef *huart, float value,
                                      uint32_t timeout_ms)
{
  uint8_t frame[VOFA_FRAME_SIZE] = {0U, 0U, 0U, 0U, 0x00U, 0x00U, 0x80U, 0x7FU};
  uint8_t i;

  (void)timeout_ms;

  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  if (vofa_uart == NULL)
  {
    VOFA_JustFloat_Init(huart);
  }

  if (vofa_uart != huart)
  {
    return HAL_ERROR;
  }

  memcpy(frame, &value, sizeof(value));

  __disable_irq();
  if (VOFA_RingFree() < VOFA_FRAME_SIZE)
  {
    __enable_irq();
    return HAL_BUSY;
  }

  for (i = 0U; i < VOFA_FRAME_SIZE; i++)
  {
    vofa_tx_buffer[vofa_tx_head] = frame[i];
    vofa_tx_head++;
    if (vofa_tx_head >= VOFA_TX_BUFFER_SIZE)
    {
      vofa_tx_head = 0U;
    }
  }
  __enable_irq();

  VOFA_TryStartTx();
  return HAL_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((vofa_uart == NULL) || (huart != vofa_uart))
  {
    return;
  }

  __disable_irq();
  vofa_tx_tail = (uint16_t)(vofa_tx_tail + vofa_tx_len);
  if (vofa_tx_tail >= VOFA_TX_BUFFER_SIZE)
  {
    vofa_tx_tail = (uint16_t)(vofa_tx_tail - VOFA_TX_BUFFER_SIZE);
  }
  vofa_tx_len = 0U;
  vofa_tx_busy = 0U;
  __enable_irq();

  VOFA_TryStartTx();
}
