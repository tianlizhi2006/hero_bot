#include "dm3519.h"

#include "fdcan.h"

dm3519_measure_t dm3519_motor[DM3519_MOTOR_COUNT];

static uint16_t float_to_uint(float value, float minimum, float maximum, uint8_t bits)
{
    float span = maximum - minimum;
    uint32_t range = (1UL << bits) - 1UL;
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    return (uint16_t)((value - minimum) * (float)range / span);
}

static float uint_to_float(uint16_t value, float minimum, float maximum, uint8_t bits)
{
    uint32_t range = (1UL << bits) - 1UL;
    return (float)value * (maximum - minimum) / (float)range + minimum;
}

static HAL_StatusTypeDef dm3519_send_frame(FDCAN_HandleTypeDef *hfdcan, uint16_t identifier, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};

    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data);
}

static HAL_StatusTypeDef dm3519_send_command(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id, uint8_t command)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};

    if ((slave_id < DM3519_FIRST_SLAVE_ID) || (slave_id > DM3519_LAST_SLAVE_ID)) return HAL_ERROR;
    return dm3519_send_frame(hfdcan, slave_id, data);
}

HAL_StatusTypeDef dm3519_can_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL) return HAL_ERROR;
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY) return HAL_OK;
    if (hfdcan->State != HAL_FDCAN_STATE_READY) return HAL_ERROR;

    // 反馈Master ID固定为对应Slave ID加0x10，即0x11至0x14
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = DM3519_FIRST_MASTER_ID;
    filter.FilterID2 = DM3519_LAST_MASTER_ID;

    //打开can
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(hfdcan);
        return HAL_ERROR;
    }
    return HAL_OK;
}

//使能函数
HAL_StatusTypeDef dm3519_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id)
{
    return dm3519_send_command(hfdcan, slave_id, 0xFCU);
}

HAL_StatusTypeDef dm3519_clear_error(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id)
{
    return dm3519_send_command(hfdcan, slave_id, 0xFBU);
}

HAL_StatusTypeDef dm3519_send_mit(FDCAN_HandleTypeDef *hfdcan, uint8_t slave_id, float position, float velocity, float kp, float kd, float torque)
{
    uint16_t p = float_to_uint(position, DM_3519_P_MIN, DM_3519_P_MAX, 16U);
    uint16_t v = float_to_uint(velocity, DM_3519_V_MIN, DM_3519_V_MAX, 12U);
    uint16_t kp_value = float_to_uint(kp, DM_3519_KP_MIN, DM_3519_KP_MAX, 12U);
    uint16_t kd_value = float_to_uint(kd, DM_3519_KD_MIN, DM_3519_KD_MAX, 12U);
    uint16_t t = float_to_uint(torque, DM_3519_T_MIN, DM_3519_T_MAX, 12U);
    uint8_t data[8];

    if ((slave_id < DM3519_FIRST_SLAVE_ID) || (slave_id > DM3519_LAST_SLAVE_ID)) return HAL_ERROR;

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp_value >> 8));
    data[4] = (uint8_t)kp_value;
    data[5] = (uint8_t)(kd_value >> 4);
    data[6] = (uint8_t)(((kd_value & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    return dm3519_send_frame(hfdcan, slave_id, data);
}

bool dm3519_motor_is_online(uint8_t index, uint32_t timeout_ms)
{
    if ((index >= DM3519_MOTOR_COUNT) || (dm3519_motor[index].message_count == 0U)) return false;
    return (HAL_GetTick() - dm3519_motor[index].last_rx_tick) <= timeout_ms;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t slave_id;
    uint8_t index;
    uint16_t p;
    uint16_t v;
    uint16_t t;

    if ((hfdcan != &hfdcan1) || ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if ((header.IdType != FDCAN_STANDARD_ID) || (header.RxFrameType != FDCAN_DATA_FRAME) || (header.DataLength != FDCAN_DLC_BYTES_8)) continue;
        if ((header.Identifier < DM3519_FIRST_MASTER_ID) || (header.Identifier > DM3519_LAST_MASTER_ID)) continue;

        slave_id = data[0] & 0x0FU;
        if ((slave_id < DM3519_FIRST_SLAVE_ID) || (slave_id > DM3519_LAST_SLAVE_ID)) continue;
        if (header.Identifier != (uint32_t)(slave_id + 0x10U)) continue;

        index = slave_id - DM3519_FIRST_SLAVE_ID;
        p = ((uint16_t)data[1] << 8) | data[2];
        v = ((uint16_t)data[3] << 4) | (data[4] >> 4);
        t = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
        dm3519_motor[index].state = data[0] >> 4;
        dm3519_motor[index].position = uint_to_float(p, DM_3519_P_MIN, DM_3519_P_MAX, 16U);
        dm3519_motor[index].velocity = uint_to_float(v, DM_3519_V_MIN, DM_3519_V_MAX, 12U);
        dm3519_motor[index].torque = uint_to_float(t, DM_3519_T_MIN, DM_3519_T_MAX, 12U);
        dm3519_motor[index].mos_temperature = data[6];
        dm3519_motor[index].rotor_temperature = data[7];
        dm3519_motor[index].last_rx_tick = HAL_GetTick();
        ++dm3519_motor[index].message_count;
    }
}
