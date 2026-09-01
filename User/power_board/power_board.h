#ifndef POWER_BOARD_H
#define POWER_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#define POWER_BOARD_CAN_RX_ID           0x301U
#define POWER_BOARD_SITUATION_ERROR     0xFFU

typedef struct
{
    uint8_t situation;
    float robot_power;          // W，原始值为小端uint16，缩放系数0.1
    uint32_t last_rx_tick;
    uint32_t message_count;
} power_board_data_t;

HAL_StatusTypeDef power_board_can_init(FDCAN_HandleTypeDef *hfdcan);
bool power_board_get_data(power_board_data_t *data);

#endif
