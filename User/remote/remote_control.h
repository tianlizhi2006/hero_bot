#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define SBUS_RX_BUF_NUM             36U
#define RC_FRAME_LENGTH             18U

#define RC_CH_VALUE_OFFSET          ((uint16_t)1024U)

#define RC_SW_UP                    ((uint8_t)1U)
#define RC_SW_MID                   ((uint8_t)3U)
#define RC_SW_DOWN                  ((uint8_t)2U)

#define RC_COMMAND_FORWARD_CHANNEL  1U
#define RC_COMMAND_HORIZONTAL_CHANNEL 0U
#define RC_COMMAND_YAW_CHANNEL      2U
#define RC_COMMAND_PITCH_CHANNEL    3U
#define RC_COMMAND_SWITCH_CHANNEL   0U
#define RC_COMMAND_DEADBAND         10
#define RC_COMMAND_FULL_SCALE       660.0f

#define KEY_PRESSED_OFFSET_W        ((uint16_t)1U << 0)
#define KEY_PRESSED_OFFSET_S        ((uint16_t)1U << 1)
#define KEY_PRESSED_OFFSET_A        ((uint16_t)1U << 2)
#define KEY_PRESSED_OFFSET_D        ((uint16_t)1U << 3)
#define KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)1U << 4)
#define KEY_PRESSED_OFFSET_CTRL     ((uint16_t)1U << 5)
#define KEY_PRESSED_OFFSET_Q        ((uint16_t)1U << 6)
#define KEY_PRESSED_OFFSET_E        ((uint16_t)1U << 7)
#define KEY_PRESSED_OFFSET_R        ((uint16_t)1U << 8)
#define KEY_PRESSED_OFFSET_F        ((uint16_t)1U << 9)
#define KEY_PRESSED_OFFSET_G        ((uint16_t)1U << 10)
#define KEY_PRESSED_OFFSET_Z        ((uint16_t)1U << 11)
#define KEY_PRESSED_OFFSET_X        ((uint16_t)1U << 12)
#define KEY_PRESSED_OFFSET_C        ((uint16_t)1U << 13)
#define KEY_PRESSED_OFFSET_V        ((uint16_t)1U << 14)
#define KEY_PRESSED_OFFSET_B        ((uint16_t)1U << 15)

typedef struct
{
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct
    {
        uint16_t v;
    } key;
} RC_ctrl_t;

typedef struct
{
    float forward;
    float horizontal;
    float yaw;
    float pitch;
    bool gyro_mode;
} remote_command_t;

bool remote_control_init(void);
const RC_ctrl_t *get_remote_control_point(void);
bool remote_control_get(RC_ctrl_t *out);
bool remote_control_is_online(void);
bool remote_control_process(remote_command_t *command);

#endif
