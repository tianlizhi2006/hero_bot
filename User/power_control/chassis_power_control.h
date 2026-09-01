#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#define CHASSIS_POWER_MOTOR_COUNT    4U

void chassis_power_control_init(void);
void chassis_power_control_reset(void);

// 每2ms调用一次，根据四轮原始力矩和转速输出限功率后的力矩
void chassis_power_control_apply(const float requested_torque[CHASSIS_POWER_MOTOR_COUNT],
                                 const float motor_speed[CHASSIS_POWER_MOTOR_COUNT],
                                 float limited_torque[CHASSIS_POWER_MOTOR_COUNT]);

#endif
