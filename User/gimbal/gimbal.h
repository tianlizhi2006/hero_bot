#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>

#include "stm32h7xx_hal.h"

#define GIMBAL_YAW_POSITION_PID_KP       0.7f
#define GIMBAL_YAW_POSITION_PID_KI       0.0001f
#define GIMBAL_YAW_POSITION_PID_KD       0.0f
#define GIMBAL_YAW_POSITION_MAX_SPEED    60.0f
#define GIMBAL_YAW_POSITION_MAX_INTEGRAL 1.0f

#define GIMBAL_YAW_SPEED_PID_KP          1.0f
#define GIMBAL_YAW_SPEED_PID_KI          0.0f
#define GIMBAL_YAW_SPEED_PID_KD          0.0f
#define GIMBAL_YAW_TORQUE_MAX            10.0f
#define GIMBAL_YAW_SPEED_MAX_INTEGRAL    0.0f

#define GIMBAL_PITCH_MIN_ANGLE            3.5f
#define GIMBAL_PITCH_MAX_ANGLE            43.5f
#define GIMBAL_PITCH_RC_MAX_SPEED_DPS     30.0f

#define GIMBAL_PITCH_POSITION_PID_KP      -20.0f
#define GIMBAL_PITCH_POSITION_PID_KI      0.0f
#define GIMBAL_PITCH_POSITION_PID_KD      0.0f
#define GIMBAL_PITCH_POSITION_MAX_RPM     4000.0f
#define GIMBAL_PITCH_POSITION_MAX_INTEGRAL 0.0f

#define GIMBAL_PITCH_SPEED_PID_KP         15.0f
#define GIMBAL_PITCH_SPEED_PID_KI         0.0f
#define GIMBAL_PITCH_SPEED_PID_KD         0.0f
#define GIMBAL_PITCH_CURRENT_MAX          15000.0f
#define GIMBAL_PITCH_SPEED_MAX_INTEGRAL   0.0f

#define GIMBAL_CONTROL_TIME_MS           1U
#define GIMBAL_FEEDBACK_TIMEOUT_MS       100U
#define GIMBAL_REENABLE_INTERVAL_MS      100U

bool gimbal_yaw_init(FDCAN_HandleTypeDef *motor_fdcan, FDCAN_HandleTypeDef *imu_fdcan);
void gimbal_yaw_control_step(void);
void gimbal_pitch_control_step(void);
void gimbal_yaw_set_target_angle(float angle);

#endif
