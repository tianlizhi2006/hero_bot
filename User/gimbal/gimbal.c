#include "gimbal.h"
#include <stddef.h>
#include "dm6220.h"
#include "IMU_can.h"
#include "m3508.h"
#include "pid.h"
#include "remote_control.h"

static pid_t yaw_position_pid;
static pid_t yaw_speed_pid;
static bool yaw_target_ready;
static float yaw_target_angle;
static uint32_t yaw_last_reenable_tick;
static pid_t pitch_position_pid;
static pid_t pitch_speed_pid;
static bool pitch_target_ready;
static float pitch_target_angle;


//角度映射，处理跨越 ±180°的问题
static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float limit_pitch_angle(float angle)
{
    if (angle > GIMBAL_PITCH_MAX_ANGLE) return GIMBAL_PITCH_MAX_ANGLE;
    if (angle < GIMBAL_PITCH_MIN_ANGLE) return GIMBAL_PITCH_MIN_ANGLE;
    return angle;
}

static void gimbal_yaw_stop(void)
{
    PID_Reset(&yaw_position_pid);
    PID_Reset(&yaw_speed_pid);
    yaw_target_ready = false;
    (void)dm6220_send_torque(0.0f);
}

static void gimbal_yaw_try_enable(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - yaw_last_reenable_tick) < GIMBAL_REENABLE_INTERVAL_MS) return;
    (void)dm6220_clear_error();
    (void)dm6220_enable();
    yaw_last_reenable_tick = now;
}

static void gimbal_pitch_stop(void)
{
    PID_Reset(&pitch_position_pid);
    PID_Reset(&pitch_speed_pid);
    pitch_target_ready = false;
    (void)m3508_stop();
}

bool gimbal_yaw_init(FDCAN_HandleTypeDef *motor_fdcan, FDCAN_HandleTypeDef *imu_fdcan)
{
    if ((motor_fdcan == NULL) || (imu_fdcan == NULL)) return false;
    if (!IMU_CAN_Init(imu_fdcan)) return false;
    if (m3508_can_init(imu_fdcan) != HAL_OK) return false;
    if (dm6220_can_init(motor_fdcan) != HAL_OK) return false;

    PID_Init(&yaw_position_pid, GIMBAL_YAW_POSITION_PID_KP, GIMBAL_YAW_POSITION_PID_KI, GIMBAL_YAW_POSITION_PID_KD,
             GIMBAL_YAW_POSITION_MAX_SPEED, GIMBAL_YAW_POSITION_MAX_INTEGRAL);
    PID_Init(&yaw_speed_pid, GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD,
             GIMBAL_YAW_TORQUE_MAX, GIMBAL_YAW_SPEED_MAX_INTEGRAL);
    PID_Init(&pitch_position_pid, GIMBAL_PITCH_POSITION_PID_KP, GIMBAL_PITCH_POSITION_PID_KI, GIMBAL_PITCH_POSITION_PID_KD,
             GIMBAL_PITCH_POSITION_MAX_RPM, GIMBAL_PITCH_POSITION_MAX_INTEGRAL);
    PID_Init(&pitch_speed_pid, GIMBAL_PITCH_SPEED_PID_KP, GIMBAL_PITCH_SPEED_PID_KI, GIMBAL_PITCH_SPEED_PID_KD,
             GIMBAL_PITCH_CURRENT_MAX, GIMBAL_PITCH_SPEED_MAX_INTEGRAL);
    gimbal_yaw_stop();
    gimbal_pitch_stop();
    return true;
}

void gimbal_yaw_control_step(void)
{
    static IMU_CAN_Data_t imu;
    float angle_error;
    float speed_target;
    float torque;
    bool motor_ready;
    remote_command_t remote_command;
    
    //无力档，安全配置
    if (!remote_control_process(&remote_command))
    {
        gimbal_yaw_stop();
        return;
    }
    motor_ready = dm6220_motor_is_online(GIMBAL_FEEDBACK_TIMEOUT_MS) && (dm6220_yaw_motor.state == 1U);
    if (!IMU_CAN_IsOnline(GIMBAL_FEEDBACK_TIMEOUT_MS) || !motor_ready)
    {
        if (!motor_ready) gimbal_yaw_try_enable();
        gimbal_yaw_stop();
        return;
    }
    
    //读取c板发送来的云台陀螺仪数据
    imu = IMU_CAN_GetData();
    if (!yaw_target_ready)
    {
        //对yaw角度进行映射处理
        yaw_target_angle = normalize_angle(imu.yaw_angle);
        yaw_target_ready = true;
    }
    //把遥控器的速度输入积分成角度，拉满摇杆速度为60度/秒
    yaw_target_angle = normalize_angle(yaw_target_angle - remote_command.yaw * GIMBAL_YAW_POSITION_MAX_SPEED *
                                       ((float)GIMBAL_CONTROL_TIME_MS / 1000.0f));
    
    //PID位置环速度环，双环控制
    angle_error = normalize_angle(yaw_target_angle - imu.yaw_angle);
    speed_target = PID_Calculate(&yaw_position_pid, angle_error, 0.0f);
    torque = PID_Calculate(&yaw_speed_pid, speed_target, imu.yaw_speed);
    //发送力矩
    (void)dm6220_send_torque(torque);
}

void gimbal_pitch_control_step(void)
{
    IMU_CAN_Data_t imu;
    remote_command_t remote_command;
    float pitch_angle;
    float speed_target;
    float current;
    
    //无力档，安全配置
    if (!remote_control_process(&remote_command) ||
        !IMU_CAN_IsOnline(GIMBAL_FEEDBACK_TIMEOUT_MS) ||
        !m3508_motor_is_online(GIMBAL_FEEDBACK_TIMEOUT_MS))
    {
        gimbal_pitch_stop();
        return;
    }
    
    //读取c板发送来的云台陀螺仪数据
    imu = IMU_CAN_GetData();
    //对pitch角度进行正反修正
    pitch_angle = -imu.pitch_angle;
    
    //对pitch角度进行限位处理
    if (!pitch_target_ready)
    {
        pitch_target_angle = limit_pitch_angle(pitch_angle);
        pitch_target_ready = true;
    }
    
    //把遥控器的速度输入积分成角度，拉满摇杆速度为15度/秒
    pitch_target_angle = limit_pitch_angle(pitch_target_angle + remote_command.pitch * GIMBAL_PITCH_RC_MAX_SPEED_DPS *
                                           ((float)GIMBAL_CONTROL_TIME_MS / 1000.0f));
    //PID位置环速度环，双环控制
    speed_target = PID_Calculate(&pitch_position_pid, pitch_target_angle, pitch_angle);
    current = PID_Calculate(&pitch_speed_pid, speed_target, (float)m3508_pitch_motor.speed_rpm);
    //到达机械限位后禁止继续向外输出
    if (((pitch_angle <= GIMBAL_PITCH_HARD_MIN_ANGLE) && (current > 0.0f)) ||
        ((pitch_angle >= GIMBAL_PITCH_HARD_MAX_ANGLE) && (current < 0.0f))) current = 0.0f;
    //发送电流
    (void)m3508_set_current((int16_t)current);
}

void gimbal_yaw_set_target_angle(float angle)
{
    yaw_target_angle = normalize_angle(angle);
    yaw_target_ready = true;
}
