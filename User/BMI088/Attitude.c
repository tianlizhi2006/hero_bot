#include "Attitude.h"
#include <math.h>

/* 姿态四元数与欧拉角 */
Quaternion_t att_quat = {1.0f, 0.0f, 0.0f, 0.0f};
Euler_t att_euler = {0.0f, 0.0f, 0.0f};

/* 互补滤波增益（可按实际效果调整）*/
#define KP  10.0f   /* 加速度计误差比例修正系数 */
#define KI  0.2f    /* 加速度计误差积分修正系数，抑制陀螺仪漂移 */

#define RAD2DEG  57.2957795131f

/* 误差积分累计量 */
static float ex_int = 0.0f;
static float ey_int = 0.0f;
static float ez_int = 0.0f;

/* 初始化姿态：四元数置为单位四元数（零姿态） */
void Attitude_Init(void)
{
    att_quat.q0 = 1.0f;
    att_quat.q1 = 0.0f;
    att_quat.q2 = 0.0f;
    att_quat.q3 = 0.0f;

    ex_int = 0.0f;
    ey_int = 0.0f;
    ez_int = 0.0f;

    Quaternion_ToEuler();
}

/* 四元数归一化 */
void Quaternion_Normalize(void)
{
    float norm = sqrtf(att_quat.q0 * att_quat.q0 +
                       att_quat.q1 * att_quat.q1 +
                       att_quat.q2 * att_quat.q2 +
                       att_quat.q3 * att_quat.q3);

    if (norm < 1e-8f)
    {
        Attitude_Init();
        return;
    }

    att_quat.q0 /= norm;
    att_quat.q1 /= norm;
    att_quat.q2 /= norm;
    att_quat.q3 /= norm;
}

/* 互补滤波姿态更新：陀螺仪积分 + 加速度计重力方向修正 */
void Attitude_Update(const Vector3f_t *gyro, const Vector3f_t *acc, float dt)
{
    float gx = gyro->x;
    float gy = gyro->y;
    float gz = gyro->z;

    float ax = acc->x;
    float ay = acc->y;
    float az = acc->z;

    /* 归一化加速度计，得到重力方向单位向量 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 1e-6f)
    {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    /* 由四元数估计当前姿态下的重力方向（机体坐标系） */
    float q0 = att_quat.q0;
    float q1 = att_quat.q1;
    float q2 = att_quat.q2;
    float q3 = att_quat.q3;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 误差 = 实测重力方向 × 估计重力方向（叉积） */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* 误差积分 */
    ex_int += KI * ex * dt;
    ey_int += KI * ey * dt;
    ez_int += KI * ez * dt;

    /* 用比例 + 积分误差修正陀螺仪 */
    gx += KP * ex + ex_int;
    gy += KP * ey + ey_int;
    gz += KP * ez + ez_int;

    /* 四元数一阶积分 */
    float half_dt = 0.5f * dt;
    att_quat.q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    att_quat.q1 += ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    att_quat.q2 += ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    att_quat.q3 += ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

    /* 归一化并转换为欧拉角 */
    Quaternion_Normalize();
    Quaternion_ToEuler();
}

/* 四元数转欧拉角（roll / pitch / yaw，单位度） */
void Quaternion_ToEuler(void)
{
    float q0 = att_quat.q0;
    float q1 = att_quat.q1;
    float q2 = att_quat.q2;
    float q3 = att_quat.q3;

    float sp = 2.0f * (q0 * q2 - q1 * q3);
    if (sp > 1.0f)  sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
		//欧拉角参数
    att_euler.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    att_euler.pitch = asinf(sp) * RAD2DEG;
    att_euler.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}
