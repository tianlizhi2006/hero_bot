#include "power_board.h"

#include <stddef.h>

static volatile power_board_data_t power_board_data;
static FDCAN_HandleTypeDef *power_board_fdcan;
static bool power_board_initialized;

// 接收并解析功率板0x301反馈报文
static void power_board_rx_fifo1_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint16_t robot_power_raw;

    if ((hfdcan != power_board_fdcan) ||
        ((rx_fifo1_its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, data) != HAL_OK) break;
        if ((header.Identifier != POWER_BOARD_CAN_RX_ID) ||
            (header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.DataLength != FDCAN_DLC_BYTES_8) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN))
        {
            continue;
        }

        // 实时功率按小端格式解析
        robot_power_raw = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

        power_board_data.situation = data[0];
        power_board_data.robot_power = (float)robot_power_raw * 0.1f;
        power_board_data.last_rx_tick = HAL_GetTick();
        ++power_board_data.message_count;
    }
}

HAL_StatusTypeDef power_board_can_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL) return HAL_ERROR;
    if (power_board_initialized) return hfdcan == power_board_fdcan ? HAL_OK : HAL_ERROR;
    if (hfdcan->State != HAL_FDCAN_STATE_READY) return HAL_ERROR;

    // 功率板与底盘电机共用FDCAN1，Filter1将0x301报文送入FIFO1
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 1U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = POWER_BOARD_CAN_RX_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_RegisterRxFifo1Callback(hfdcan, power_board_rx_fifo1_callback) != HAL_OK) return HAL_ERROR;

    power_board_fdcan = hfdcan;
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U) != HAL_OK)
    {
        power_board_fdcan = NULL;
        return HAL_ERROR;
    }

    // FDCAN1由DM3519初始化函数统一启动
    power_board_initialized = true;
    return HAL_OK;
}

bool power_board_get_data(power_board_data_t *data)
{
    uint32_t primask;

    if (data == NULL) return false;

    // 复制期间关闭中断，避免读取到同一帧的新旧混合数据
    primask = __get_PRIMASK();
    __disable_irq();
    data->situation = power_board_data.situation;
    data->robot_power = power_board_data.robot_power;
    data->last_rx_tick = power_board_data.last_rx_tick;
    data->message_count = power_board_data.message_count;
    if (primask == 0U) __enable_irq();

    return data->message_count != 0U;
}
