#include "remote_control.h"
#include "main.h"
#include "bsp_rc.h"
#include "usart.h"
#include "stm32h7xx_hal.h"

#include <stddef.h>

#define RC_DMA_DISABLE_GUARD_COUNT  100000U
#define RC_CHANNEL_MAX_ABS          700
#define RC_OFFLINE_TIMEOUT_MS       100U

//解析出的参数存放在RC_ctrl_t类型结构体中
static RC_ctrl_t rc_ctrl;
// 偶数表示数据稳定，奇数表示中断正在更新，供任务无锁读取
static volatile uint32_t rc_update_sequence;
static volatile uint32_t rc_last_update_tick;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

static void SBUS_ToRC(volatile const uint8_t *sbus_buf, RC_ctrl_t *out);

bool remote_control_init(void)
{
    return RC_Init(&huart5, sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM) == HAL_OK;
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

bool remote_control_get(RC_ctrl_t *out)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (out == NULL)
    {
        return false;
    }

    // 序列号前后一致时才接受快照，避免读到更新一半的遥控数据
    do
    {
        sequence_before = rc_update_sequence;
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        __DMB();
        *out = rc_ctrl;
        __DMB();
        sequence_after = rc_update_sequence;
    } while ((sequence_before != sequence_after) ||
             ((sequence_after & 1U) != 0U));

    return remote_control_is_online();
}

bool remote_control_is_online(void)
{
    return (rc_update_sequence != 0U) &&
           ((HAL_GetTick() - rc_last_update_tick) <= RC_OFFLINE_TIMEOUT_MS);
}

bool remote_control_process(remote_command_t *command)
{
    RC_ctrl_t remote;
    int16_t channel_forward;
    int16_t channel_horizontal;
    int16_t channel_yaw;
    int16_t channel_pitch;

    if (!remote_control_get(&remote) || (remote.rc.s[RC_COMMAND_SWITCH_CHANNEL] == RC_SW_DOWN)) return false;
    if (command == NULL) return true;

    channel_forward = remote.rc.ch[RC_COMMAND_FORWARD_CHANNEL];
    channel_horizontal = remote.rc.ch[RC_COMMAND_HORIZONTAL_CHANNEL];
    channel_yaw = remote.rc.ch[RC_COMMAND_YAW_CHANNEL];
    channel_pitch = remote.rc.ch[RC_COMMAND_PITCH_CHANNEL];
    if ((channel_forward >= -RC_COMMAND_DEADBAND) && (channel_forward <= RC_COMMAND_DEADBAND)) channel_forward = 0;
    if ((channel_horizontal >= -RC_COMMAND_DEADBAND) && (channel_horizontal <= RC_COMMAND_DEADBAND)) channel_horizontal = 0;
    if ((channel_yaw >= -RC_COMMAND_DEADBAND) && (channel_yaw <= RC_COMMAND_DEADBAND)) channel_yaw = 0;
    if ((channel_pitch >= -RC_COMMAND_DEADBAND) && (channel_pitch <= RC_COMMAND_DEADBAND)) channel_pitch = 0;

    command->forward = (float)channel_forward / RC_COMMAND_FULL_SCALE;
    command->horizontal = (float)channel_horizontal / RC_COMMAND_FULL_SCALE;
    command->yaw = (float)channel_yaw / RC_COMMAND_FULL_SCALE;
    command->pitch = (float)channel_pitch / RC_COMMAND_FULL_SCALE;
    command->gyro_mode = remote.rc.s[RC_COMMAND_SWITCH_CHANNEL] == RC_SW_UP;
    return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    DMA_HandleTypeDef *hdma;
    DMA_Stream_TypeDef *stream;
    uint8_t *completed_buffer;
    uint32_t current_target;
    uint32_t guard = RC_DMA_DISABLE_GUARD_COUNT;
    __IO uint32_t *ifcr;

    if (huart != &huart5)
    {
        return;
    }

    hdma = huart->hdmarx;
    stream = (DMA_Stream_TypeDef *)hdma->Instance;
    current_target = stream->CR & DMA_SxCR_CT;

    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_DISABLE(hdma);
    while (((stream->CR & DMA_SxCR_EN) != 0U) && (guard > 0U))
    {
        --guard;
    }

    if ((stream->CR & DMA_SxCR_EN) != 0U)
    {
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        return;
    }

    // CT表示DMA当前目标；解析当前缓冲区，并切换到另一个缓冲区接收
    if (current_target == 0U)
    {
        completed_buffer = sbus_rx_buf[0];
        SET_BIT(stream->CR, DMA_SxCR_CT);
    }
    else
    {
        completed_buffer = sbus_rx_buf[1];
        CLEAR_BIT(stream->CR, DMA_SxCR_CT);
    }

    stream->NDTR = SBUS_RX_BUF_NUM;
    huart->RxXferCount = SBUS_RX_BUF_NUM;

    ifcr = (__IO uint32_t *)(hdma->StreamBaseAddress + 8U);
    *ifcr = 0x3FUL << (hdma->StreamIndex & 0x1FU);

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);

    __DMB();
    __HAL_DMA_ENABLE(hdma);
    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    // 只解析完整18字节DBUS帧，异常帧不更新时间戳
    if (size == RC_FRAME_LENGTH)
    {
        RC_ctrl_t decoded;

        SBUS_ToRC(completed_buffer, &decoded);
        if ((decoded.rc.ch[0] >= -RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[0] <= RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[1] >= -RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[1] <= RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[2] >= -RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[2] <= RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[3] >= -RC_CHANNEL_MAX_ABS) &&
            (decoded.rc.ch[3] <= RC_CHANNEL_MAX_ABS) &&
            ((decoded.rc.s[0] == RC_SW_UP) ||
             (decoded.rc.s[0] == RC_SW_MID) ||
             (decoded.rc.s[0] == RC_SW_DOWN)) &&
            ((decoded.rc.s[1] == RC_SW_UP) ||
             (decoded.rc.s[1] == RC_SW_MID) ||
             (decoded.rc.s[1] == RC_SW_DOWN)))
        {
            ++rc_update_sequence;
            __DMB();
            rc_ctrl = decoded;
            rc_last_update_tick = HAL_GetTick();
            __DMB();
            ++rc_update_sequence;
        }
    }
}

static uint16_t ReadU16Le(volatile const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void SBUS_ToRC(volatile const uint8_t *sbus_buf, RC_ctrl_t *out)
{
    // DR16前四个通道为11位紧凑排列，原始中值为1024
    out->rc.ch[0] = (int16_t)((sbus_buf[0] |
                              ((uint16_t)sbus_buf[1] << 8)) & 0x07FFU);
    out->rc.ch[1] = (int16_t)(((sbus_buf[1] >> 3) |
                              ((uint16_t)sbus_buf[2] << 5)) & 0x07FFU);
    out->rc.ch[2] = (int16_t)(((sbus_buf[2] >> 6) |
                              ((uint16_t)sbus_buf[3] << 2) |
                              ((uint16_t)sbus_buf[4] << 10)) & 0x07FFU);
    out->rc.ch[3] = (int16_t)(((sbus_buf[4] >> 1) |
                              ((uint16_t)sbus_buf[5] << 7)) & 0x07FFU);

    out->rc.s[0] = (uint8_t)((sbus_buf[5] >> 4) & 0x03U);
    out->rc.s[1] = (uint8_t)((sbus_buf[5] >> 6) & 0x03U);

    out->mouse.x = (int16_t)ReadU16Le(&sbus_buf[6]);
    out->mouse.y = (int16_t)ReadU16Le(&sbus_buf[8]);
    out->mouse.z = (int16_t)ReadU16Le(&sbus_buf[10]);
    out->mouse.press_l = sbus_buf[12];
    out->mouse.press_r = sbus_buf[13];
    out->key.v = ReadU16Le(&sbus_buf[14]);
    out->rc.ch[4] = (int16_t)ReadU16Le(&sbus_buf[16]);

    out->rc.ch[0] -= (int16_t)RC_CH_VALUE_OFFSET;
    out->rc.ch[1] -= (int16_t)RC_CH_VALUE_OFFSET;
    out->rc.ch[2] -= (int16_t)RC_CH_VALUE_OFFSET;
    out->rc.ch[3] -= (int16_t)RC_CH_VALUE_OFFSET;
    out->rc.ch[4] -= (int16_t)RC_CH_VALUE_OFFSET;
}
