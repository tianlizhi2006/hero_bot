#include "IMU_can.h"

static FDCAN_HandleTypeDef *s_hfdcan;
static IMU_CAN_Data_t s_data;
static uint32_t s_last_rx_tick;
static uint32_t s_message_count;
static bool s_initialized;

#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U
static void IMU_CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
        IMU_CAN_Process(hfdcan);
}
#endif

bool IMU_CAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL) return false;
    if (s_initialized) return hfdcan == s_hfdcan;
    if (hfdcan->State != HAL_FDCAN_STATE_READY) return false;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = IMU_CAN_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return false;
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return false;

    s_hfdcan = hfdcan;
#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1U
    if (HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, IMU_CAN_RxFifo0Callback) != HAL_OK)
    {
        s_hfdcan = NULL;
        return false;
    }
#else
    s_hfdcan = NULL;
    return false;
#endif
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        s_hfdcan = NULL;
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(hfdcan);
        s_hfdcan = NULL;
        return false;
    }
    s_initialized = true;
    return true;
}

void IMU_CAN_Process(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[8];

    if (hfdcan != s_hfdcan)
        return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK)
            break;
        if (h.Identifier != IMU_CAN_ID || h.DataLength != 8U)
            continue;

        s_data.pitch_angle = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8U)) / IMU_CAN_SCALE;
        s_data.pitch_speed = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8U)) / IMU_CAN_SCALE;
        s_data.yaw_angle   = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8U)) / IMU_CAN_SCALE;
        s_data.yaw_speed   = (int16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8U)) / IMU_CAN_SCALE;
        s_last_rx_tick = HAL_GetTick();
        ++s_message_count;
    }
}

IMU_CAN_Data_t IMU_CAN_GetData(void)
{
    return s_data;
}

bool IMU_CAN_IsOnline(uint32_t timeout_ms)
{
    return (s_message_count != 0U) && ((HAL_GetTick() - s_last_rx_tick) <= timeout_ms);
}

