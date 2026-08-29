#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

HAL_StatusTypeDef RC_Init(UART_HandleTypeDef *huart, uint8_t *memory0, uint8_t *memory1, uint16_t buffer_length);

#endif
