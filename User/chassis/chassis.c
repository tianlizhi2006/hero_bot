#include "chassis.h"

#include <math.h>

#include "chassis_power_control.h"
#include "dm3519.h"
#include "dm6220.h"
#include "pid.h"
#include "power_board.h"
#include "remote_control.h"

typedef struct
{
    float forward;
    float right;
    float rotate;
} chassis_command_t;

static FDCAN_HandleTypeDef *chassis_fdcan;
static float motor_target_velocity[DM3519_MOTOR_COUNT];
static pid_t motor_speed_pid[DM3519_MOTOR_COUNT];
static const float motor_speed_kp[DM3519_MOTOR_COUNT] =
{
    CHASSIS_FRONT_SPEED_PID_KP,
    CHASSIS_FRONT_SPEED_PID_KP,
    CHASSIS_REAR_SPEED_PID_KP,
    CHASSIS_REAR_SPEED_PID_KP
};

// 根据ABBA麦轮布局将底盘速度逆解为四个DM3519目标角速度
static void chassis_kinematics_solve(const chassis_command_t *command, float target[DM3519_MOTOR_COUNT])
{
    float rotate_vector;
    float maximum = 0.0f;
    float scale = 1.0f;
    uint8_t i;
    
    //角速度计算得到线速度
    rotate_vector = command->rotate * MOTOR_DISTANCE_TO_CENTER;
    //三个分量叠加得到各个电机的线速度和，除以系数得到转子角速度。
    // 左侧电机镜像安装，目标值取负
    target[CHASSIS_MOTOR_FRONT_LEFT] = -(command->forward + command->right + rotate_vector) / CHASSIS_MOTOR_RADPS_TO_VECTOR;
    target[CHASSIS_MOTOR_FRONT_RIGHT] = (command->forward - command->right - rotate_vector) / CHASSIS_MOTOR_RADPS_TO_VECTOR;
    target[CHASSIS_MOTOR_REAR_LEFT] = -(command->forward - command->right + rotate_vector) / CHASSIS_MOTOR_RADPS_TO_VECTOR;
    target[CHASSIS_MOTOR_REAR_RIGHT] = (command->forward + command->right - rotate_vector) / CHASSIS_MOTOR_RADPS_TO_VECTOR;
    
    //找出四个目标速度中的最大绝对值maximum
    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        if (fabsf(target[i]) > maximum) maximum = fabsf(target[i]);
    }
    // 超过配置的电机速度上限时，四轮按相同比例缩小
    if (maximum > CHASSIS_MAX_MOTOR_RADPS)
    scale = CHASSIS_MAX_MOTOR_RADPS / maximum;
    target[CHASSIS_MOTOR_FRONT_LEFT] *= scale;
    target[CHASSIS_MOTOR_FRONT_RIGHT] *= scale;
    target[CHASSIS_MOTOR_REAR_LEFT] *= scale;
    target[CHASSIS_MOTOR_REAR_RIGHT] *= scale;
}

// 使用速度反馈计算力矩，并给四个电机分别发送MIT控制帧
static void chassis_motor_control(const float target[DM3519_MOTOR_COUNT])
{
    float requested_torque[DM3519_MOTOR_COUNT];
    float limited_torque[DM3519_MOTOR_COUNT];
    float motor_speed[DM3519_MOTOR_COUNT];
    float difference;
    bool motors_ready = true;
    uint8_t i;
    //防止疯车
    // 未使能时清错并重发使能；反馈超时则保持零力矩
    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        if (dm3519_motor[i].state != 1U)
        {
            (void)dm3519_clear_error(chassis_fdcan, i + DM3519_FIRST_SLAVE_ID);
            HAL_Delay(1U);
            (void)dm3519_enable(chassis_fdcan, i + DM3519_FIRST_SLAVE_ID);
            HAL_Delay(1U);
            motors_ready = false;
        }
        else if (!dm3519_motor_is_online(i, CHASSIS_MOTOR_TIMEOUT_MS))
        {
            motors_ready = false;
        }
    }
    if (!motors_ready)
    {
        chassis_stop();
        return;
    }
    
    // 先计算四个底盘电机的原始力矩
    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        // 每2ms最多改变0.40rad/s，减小启动和换向冲击
        difference = target[i] - motor_target_velocity[i];
        // 差值较大时按固定步长加速或减速，进入步长范围后直接到达目标
        if (difference > CHASSIS_TARGET_SLEW_RADPS) motor_target_velocity[i] += CHASSIS_TARGET_SLEW_RADPS;
        else if (difference < -CHASSIS_TARGET_SLEW_RADPS) motor_target_velocity[i] -= CHASSIS_TARGET_SLEW_RADPS;
        else motor_target_velocity[i] = target[i];
        // 输入目标速度和反馈速度，函数返回MIT力矩
        motor_speed[i] = dm3519_motor[i].velocity;
        requested_torque[i] = PID_Calculate(&motor_speed_pid[i], motor_target_velocity[i], motor_speed[i]);
    }

    // 根据功率模型和功率板Pout统一限制四轮力矩
    chassis_power_control_apply(requested_torque, motor_speed, limited_torque);

    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        (void)dm3519_send_mit(chassis_fdcan, i + DM3519_FIRST_SLAVE_ID,
                              0.0f, 0.0f, 0.0f, 0.0f, limited_torque[i]);
    }
}

bool chassis_init(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t i;

    if (power_board_can_init(hfdcan) != HAL_OK) return false;
    if (dm3519_can_init(hfdcan) != HAL_OK) return false;

    chassis_fdcan = hfdcan;
    chassis_power_control_init();
    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        PID_Init(&motor_speed_pid[i], motor_speed_kp[i], CHASSIS_SPEED_PID_KI, CHASSIS_SPEED_PID_KD, CHASSIS_TORQUE_MAX, CHASSIS_INTEGRAL_MAX);
        if (dm3519_enable(hfdcan, i + DM3519_FIRST_SLAVE_ID) != HAL_OK) return false;
    }
    return true;
}

void chassis_stop(void)
{
    uint8_t i;

    chassis_power_control_reset();

    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        PID_Reset(&motor_speed_pid[i]);
        motor_target_velocity[i] = 0.0f;
    }

    if (chassis_fdcan == NULL) return;

    for (i = 0U; i < DM3519_MOTOR_COUNT; ++i)
    {
        (void)dm3519_send_mit(chassis_fdcan, i + DM3519_FIRST_SLAVE_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

void chassis_control_step(void)
{
    //中心点目标速度
    chassis_command_t command;
    remote_command_t remote_command;
    //四个电机的目标角速度
    float target[DM3519_MOTOR_COUNT];

    float gimbal_forward;
    float gimbal_right;
    float sin_yaw;
    float cos_yaw;
    float rotate_weight;
    //安全处理
    if ((chassis_fdcan == NULL) || !remote_control_process(&remote_command))
    {
        chassis_stop();
        return;
    }

    //进入小陀螺模式分支
    if (remote_command.gyro_mode)
    {
        if (!dm6220_motor_is_online(CHASSIS_GIMBAL_TIMEOUT_MS) || (dm6220_yaw_motor.state != 1U))
        {
            chassis_stop();
            return;
        }

        gimbal_forward = remote_command.forward * CHASSIS_MAX_TRANSLATE_MPS;
        gimbal_right = remote_command.horizontal * CHASSIS_MAX_TRANSLATE_MPS;
        sin_yaw = sinf(dm6220_yaw_motor.position);
        cos_yaw = cosf(dm6220_yaw_motor.position);
        //利用旋转矩阵，把云台坐标系下的前后左右速度转换为底盘坐标系下的前后左右速度
        command.forward = cos_yaw * gimbal_forward - sin_yaw * gimbal_right;
        command.right = sin_yaw * gimbal_forward + cos_yaw * gimbal_right;
        //平移输入越大，旋转权重越低
        rotate_weight = 1.0f - (fabsf(remote_command.forward) + fabsf(remote_command.horizontal)) /
                               CHASSIS_GYRO_WEIGHT_DIVISOR;
        if (rotate_weight < CHASSIS_GYRO_MIN_ROTATE_WEIGHT) rotate_weight = CHASSIS_GYRO_MIN_ROTATE_WEIGHT;
        command.rotate = CHASSIS_GYRO_ROTATE_RADPS * rotate_weight;
    }
    //普通模式分支
    else
    {
        command.forward = remote_command.forward * CHASSIS_MAX_TRANSLATE_MPS;
        command.right = 0.0f;
        command.rotate = remote_command.horizontal * CHASSIS_MAX_ROTATE_RADPS;
    }

    // 将底盘速度逆解为四个电机目标角速度
    chassis_kinematics_solve(&command, target);

    // 使用DM3519速度反馈执行缓启动和PID，并发送四路MIT力矩
    chassis_motor_control(target);
}
