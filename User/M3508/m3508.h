#ifndef M3508_H
#define M3508_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#define M3508_MOTOR_ID                4U
#define M3508_CONTROL_ID              0x200U
#define M3508_FEEDBACK_ID             (M3508_CONTROL_ID + M3508_MOTOR_ID)
#define M3508_ENCODER_RANGE           8192U
#define M3508_CURRENT_COMMAND_MAX     16384
#define M3508_CURRENT_MAX_AMP         20.0f

typedef struct
{
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t torque_current;
    float torque_current_amp;
    uint8_t temperature;
    uint32_t last_rx_tick;
    uint32_t message_count;
} m3508_measure_t;

extern m3508_measure_t m3508_pitch_motor;

HAL_StatusTypeDef m3508_can_init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef m3508_set_current(int16_t current);
HAL_StatusTypeDef m3508_stop(void);
bool m3508_motor_is_online(uint32_t timeout_ms);

#endif
