/**
 * @file vofa_justfloat.h
 * @brief VOFA+ JustFloat protocol transmitter.
 */
#ifndef VOFA_JUSTFLOAT_H
#define VOFA_JUSTFLOAT_H

#include "main.h"

void VOFA_JustFloat_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef VOFA_JustFloat_Send(UART_HandleTypeDef *huart, float value,
                                      uint32_t timeout_ms);

#endif /* VOFA_JUSTFLOAT_H */
