#include "dm6220.h"

#include <stddef.h>

dm6220_measure_t dm6220_yaw_motor;

static FDCAN_HandleTypeDef *dm6220_fdcan;
static bool dm6220_initialized;

static uint16_t float_to_uint(float value, float minimum, float maximum, uint8_t bits)
{
    uint32_t range = (1UL << bits) - 1UL;

    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    return (uint16_t)((value - minimum) * (float)range / (maximum - minimum));
}

static float uint_to_float(uint16_t value, float minimum, float maximum, uint8_t bits)
{
    uint32_t range = (1UL << bits) - 1UL;
    return (float)value * (maximum - minimum) / (float)range + minimum;
}

static HAL_StatusTypeDef dm6220_send_frame(const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (dm6220_fdcan == NULL) return HAL_ERROR;

    header.Identifier = DM6220_SLAVE_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(dm6220_fdcan, &header, data);
}

static HAL_StatusTypeDef dm6220_send_command(uint8_t command)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};
    return dm6220_send_frame(data);
}

#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U
static void dm6220_rx_fifo0_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint16_t p;
    uint16_t v;
    uint16_t t;

    if ((hfdcan != dm6220_fdcan) || ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if ((header.IdType != FDCAN_STANDARD_ID) || (header.RxFrameType != FDCAN_DATA_FRAME) || (header.DataLength != FDCAN_DLC_BYTES_8)) continue;
        if ((header.Identifier != DM6220_MASTER_ID) || ((data[0] & 0x0FU) != DM6220_SLAVE_ID)) continue;

        p = ((uint16_t)data[1] << 8) | data[2];
        v = ((uint16_t)data[3] << 4) | (data[4] >> 4);
        t = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

        dm6220_yaw_motor.state = data[0] >> 4;
        dm6220_yaw_motor.position = uint_to_float(p, DM6220_P_MIN, DM6220_P_MAX, 16U);
        dm6220_yaw_motor.velocity = uint_to_float(v, DM6220_V_MIN, DM6220_V_MAX, 12U);
        dm6220_yaw_motor.torque = uint_to_float(t, DM6220_T_MIN, DM6220_T_MAX, 12U);
        dm6220_yaw_motor.mos_temperature = data[6];
        dm6220_yaw_motor.rotor_temperature = data[7];
        dm6220_yaw_motor.last_rx_tick = HAL_GetTick();
        ++dm6220_yaw_motor.message_count;
    }
}
#endif

HAL_StatusTypeDef dm6220_can_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL) return HAL_ERROR;
    if (dm6220_initialized) return hfdcan == dm6220_fdcan ? HAL_OK : HAL_ERROR;
    if (hfdcan->State != HAL_FDCAN_STATE_READY) return HAL_ERROR;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = DM6220_MASTER_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return HAL_ERROR;

    dm6220_fdcan = hfdcan;
#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U
    if (HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, dm6220_rx_fifo0_callback) != HAL_OK)
    {
        dm6220_fdcan = NULL;
        return HAL_ERROR;
    }
#else
    dm6220_fdcan = NULL;
    return HAL_ERROR;
#endif

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        dm6220_fdcan = NULL;
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(hfdcan);
        dm6220_fdcan = NULL;
        return HAL_ERROR;
    }

    dm6220_initialized = true;
    return dm6220_enable();
}

HAL_StatusTypeDef dm6220_enable(void)
{
    return dm6220_send_command(0xFCU);
}

HAL_StatusTypeDef dm6220_clear_error(void)
{
    return dm6220_send_command(0xFBU);
}

HAL_StatusTypeDef dm6220_send_torque(float torque)
{
    uint16_t p = float_to_uint(0.0f, DM6220_P_MIN, DM6220_P_MAX, 16U);
    uint16_t v = float_to_uint(0.0f, DM6220_V_MIN, DM6220_V_MAX, 12U);
    uint16_t kp = float_to_uint(0.0f, DM6220_KP_MIN, DM6220_KP_MAX, 12U);
    uint16_t kd = float_to_uint(0.0f, DM6220_KD_MIN, DM6220_KD_MAX, 12U);
    uint16_t t = float_to_uint(torque, DM6220_T_MIN, DM6220_T_MAX, 12U);
    uint8_t data[8];

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp >> 8));
    data[4] = (uint8_t)kp;
    data[5] = (uint8_t)(kd >> 4);
    data[6] = (uint8_t)(((kd & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    return dm6220_send_frame(data);
}

bool dm6220_motor_is_online(uint32_t timeout_ms)
{
    return (dm6220_yaw_motor.message_count != 0U) && ((HAL_GetTick() - dm6220_yaw_motor.last_rx_tick) <= timeout_ms);
}
