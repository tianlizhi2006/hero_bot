#include "pid.h"

#include <stddef.h>

static float limit_float(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_integral)
{
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->max_integral = max_integral;
    PID_Reset(pid);
}

void PID_Reset(pid_t *pid)
{
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float PID_Calculate(pid_t *pid, float target, float feedback)
{
    float error;
    float output;

    if (pid == NULL) return 0.0f;

    error = target - feedback;
    pid->integral = limit_float(pid->integral + pid->ki * error, pid->max_integral);
    output = pid->kp * error + pid->integral + pid->kd * (error - pid->last_error);
    pid->last_error = error;
    return limit_float(output, pid->max_output);
}
