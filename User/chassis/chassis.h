#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

// 麦轮俯视布局为ABBA：左前A、右前B、左后B、右后A
// DM3519 Slave ID：左前1、右前2、左后3、右后4
// DM3519转子角速度rad/s乘以下系数得到轮缘线速度m/s
#define MOTOR_DISTANCE_TO_CENTER        0.35292f
#define DM3519_REDUCTION                 0.052074627106613946f
#define DM_WHEEL_RADIUS_M                0.0765f
#define CHASSIS_MOTOR_RADPS_TO_VECTOR    (DM3519_REDUCTION * DM_WHEEL_RADIUS_M)

// 外部速度PID输出MIT前馈力矩
#define CHASSIS_FRONT_SPEED_PID_KP       0.0757f
#define CHASSIS_REAR_SPEED_PID_KP        0.0478f
#define CHASSIS_SPEED_PID_KI             0.0f
#define CHASSIS_SPEED_PID_KD             0.0f
//力矩上限
#define CHASSIS_TORQUE_MAX               1.5f
//积分项上限
#define CHASSIS_INTEGRAL_MAX             0.0f

#define CHASSIS_MAX_MOTOR_RADPS          80.0f
#define CHASSIS_MAX_TRANSLATE_MPS        0.30f
#define CHASSIS_MAX_ROTATE_RADPS         0.60f
#define CHASSIS_GYRO_ROTATE_RADPS        0.60f
#define CHASSIS_GYRO_WEIGHT_DIVISOR      3.0f
#define CHASSIS_GYRO_MIN_ROTATE_WEIGHT   0.33f
#define CHASSIS_GIMBAL_TIMEOUT_MS        100U
#define CHASSIS_TARGET_SLEW_RADPS        0.40f
#define CHASSIS_CONTROL_TIME_MS          2U
#define CHASSIS_MOTOR_TIMEOUT_MS         100U

typedef enum
{
    CHASSIS_MOTOR_FRONT_LEFT = 0,
    CHASSIS_MOTOR_FRONT_RIGHT,
    CHASSIS_MOTOR_REAR_LEFT,
    CHASSIS_MOTOR_REAR_RIGHT
} chassis_motor_index_t;

bool chassis_init(FDCAN_HandleTypeDef *hfdcan);
void chassis_control_step(void);
void chassis_stop(void);

#endif
