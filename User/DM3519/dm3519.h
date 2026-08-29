#ifndef DM3519_H
#define DM3519_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#define DM3519_MOTOR_COUNT       4U
#define DM3519_FIRST_SLAVE_ID    0x01U
#define DM3519_LAST_SLAVE_ID     0x04U
#define DM3519_FIRST_MASTER_ID   0x11U
#define DM3519_LAST_MASTER_ID    0x14U

#define DM_3519_P_MIN           -12.5f
#define DM_3519_P_MAX            12.5f
#define DM_3519_V_MIN          -200.0f
#define DM_3519_V_MAX           200.0f
#define DM_3519_T_MIN           -10.0f
#define DM_3519_T_MAX            10.0f
#define DM_3519_KP_MIN            0.0f
#define DM_3519_KP_MAX          500.0f
#define DM_3519_KD_MIN            0.0f
#define DM_3519_KD_MAX            5.0f

typedef struct
{
    uint8_t state;
    float position;
    float velocity;
    float torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    volatile uint32_t message_count;
    volatile uint32_t last_rx_tick;
} dm3519_measure_t;

extern dm3519_measure_t dm3519_motor[DM3519_MOTOR_COUNT];

HAL_StatusTypeDef dm3519_can_init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef dm3519_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id);
HAL_StatusTypeDef dm3519_clear_error(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id);
HAL_StatusTypeDef dm3519_send_mit(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id, float position, float velocity, float kp, float kd, float torque);
bool dm3519_motor_is_online(uint8_t index, uint32_t timeout_ms);

#endif
