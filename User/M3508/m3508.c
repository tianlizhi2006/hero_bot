#include "m3508.h"

#include <stddef.h>

m3508_measure_t m3508_pitch_motor;

static FDCAN_HandleTypeDef *m3508_fdcan;
static bool m3508_initialized;

static int16_t limit_current(int16_t current)
{
    if (current > M3508_CURRENT_COMMAND_MAX) return M3508_CURRENT_COMMAND_MAX;
    if (current < -M3508_CURRENT_COMMAND_MAX) return -M3508_CURRENT_COMMAND_MAX;
    return current;
}

static HAL_StatusTypeDef m3508_send_current(int16_t current)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};

    if (m3508_fdcan == NULL) return HAL_ERROR;

    current = limit_current(current);
    data[6] = (uint8_t)((uint16_t)current >> 8);
    data[7] = (uint8_t)current;

    header.Identifier = M3508_CONTROL_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(m3508_fdcan, &header, data);
}

#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U
static void m3508_rx_fifo1_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((hfdcan != m3508_fdcan) || ((rx_fifo1_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, data) != HAL_OK) break;
        if ((header.IdType != FDCAN_STANDARD_ID) || (header.RxFrameType != FDCAN_DATA_FRAME) || (header.DataLength != FDCAN_DLC_BYTES_8)) continue;
        if (header.Identifier != M3508_FEEDBACK_ID) continue;

        m3508_pitch_motor.encoder = ((uint16_t)data[0] << 8) | data[1];
        m3508_pitch_motor.speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        m3508_pitch_motor.torque_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        m3508_pitch_motor.torque_current_amp = (float)m3508_pitch_motor.torque_current * M3508_CURRENT_MAX_AMP / (float)M3508_CURRENT_COMMAND_MAX;
        m3508_pitch_motor.temperature = data[6];
        m3508_pitch_motor.last_rx_tick = HAL_GetTick();
        ++m3508_pitch_motor.message_count;
    }
}
#endif

HAL_StatusTypeDef m3508_can_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};
    bool was_started;

    if (hfdcan == NULL) return HAL_ERROR;
    if (m3508_initialized) return hfdcan == m3508_fdcan ? HAL_OK : HAL_ERROR;

    was_started = hfdcan->State == HAL_FDCAN_STATE_BUSY;
    if (was_started)
    {
        if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) return HAL_ERROR;
    }
    else if (hfdcan->State != HAL_FDCAN_STATE_READY)
    {
        return HAL_ERROR;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 1U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = M3508_FEEDBACK_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        if (was_started) (void)HAL_FDCAN_Start(hfdcan);
        return HAL_ERROR;
    }

    m3508_fdcan = hfdcan;
#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U
    if (HAL_FDCAN_RegisterRxFifo1Callback(hfdcan, m3508_rx_fifo1_callback) != HAL_OK)
    {
        m3508_fdcan = NULL;
        if (was_started) (void)HAL_FDCAN_Start(hfdcan);
        return HAL_ERROR;
    }
#else
    m3508_fdcan = NULL;
    if (was_started) (void)HAL_FDCAN_Start(hfdcan);
    return HAL_ERROR;
#endif

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        m3508_fdcan = NULL;
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U) != HAL_OK)
    {
        m3508_fdcan = NULL;
        return HAL_ERROR;
    }

    m3508_initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef m3508_set_current(int16_t current)
{
    return m3508_send_current(current);
}

HAL_StatusTypeDef m3508_stop(void)
{
    return m3508_send_current(0);
}

bool m3508_motor_is_online(uint32_t timeout_ms)
{
    return (m3508_pitch_motor.message_count != 0U) && ((HAL_GetTick() - m3508_pitch_motor.last_rx_tick) <= timeout_ms);
}
