#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "BMI088.h"

/* 四元数：q0 为实部，q1/q2/q3 为虚部 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

/* 欧拉角：单位度 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Euler_t;

extern Quaternion_t att_quat;
extern Euler_t att_euler;

void Attitude_Init(void);
void Attitude_Update(const Vector3f_t *gyro, const Vector3f_t *acc, float dt);
void Quaternion_Normalize(void);
void Quaternion_ToEuler(void);

#endif
