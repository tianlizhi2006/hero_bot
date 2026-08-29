#ifndef __IMU_CAN_H
#define __IMU_CAN_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define IMU_CAN_ID    0x01U
#define IMU_CAN_SCALE 182.0f

typedef struct
{
    float pitch_angle;  // 俯仰角
    float pitch_speed;  // 俯仰角速度
    float yaw_angle;    // 偏航角
    float yaw_speed;    // 偏航角速度
} IMU_CAN_Data_t;

bool IMU_CAN_Init(FDCAN_HandleTypeDef *hfdcan);
void IMU_CAN_Process(FDCAN_HandleTypeDef *hfdcan);
IMU_CAN_Data_t IMU_CAN_GetData(void);
bool IMU_CAN_IsOnline(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif

