#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float max_output;
    float max_integral;
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_integral);
float PID_Calculate(pid_t *pid, float target, float feedback);
void PID_Reset(pid_t *pid);

#endif
