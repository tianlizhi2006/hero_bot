#ifndef DM6220_H
#define DM6220_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#define DM6220_SLAVE_ID              0x05U
#define DM6220_MASTER_ID             0x14U

#define DM6220_P_MIN                 -3.141593f
#define DM6220_P_MAX                  3.141593f
#define DM6220_V_MIN                -45.0f
#define DM6220_V_MAX                 45.0f
#define DM6220_T_MIN                -10.0f
#define DM6220_T_MAX                 10.0f
#define DM6220_KP_MIN                0.0f
#define DM6220_KP_MAX                500.0f
#define DM6220_KD_MIN                0.0f
#define DM6220_KD_MAX                5.0f

typedef struct
{
    uint8_t state;
    float position;
    float velocity;
    float torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint32_t last_rx_tick;
    uint32_t message_count;
} dm6220_measure_t;

extern dm6220_measure_t dm6220_yaw_motor;

HAL_StatusTypeDef dm6220_can_init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef dm6220_enable(void);
HAL_StatusTypeDef dm6220_clear_error(void);
HAL_StatusTypeDef dm6220_send_torque(float torque);
bool dm6220_motor_is_online(uint32_t timeout_ms);

#endif
