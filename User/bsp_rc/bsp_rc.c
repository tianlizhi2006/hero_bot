#include "bsp_rc.h"

#define RC_DMA_DISABLE_GUARD_COUNT  100000U

HAL_StatusTypeDef RC_Init(UART_HandleTypeDef *huart, uint8_t *memory0, uint8_t *memory1, uint16_t buffer_length)
{
    DMA_HandleTypeDef *hdma;
    DMA_Stream_TypeDef *stream;
    HAL_StatusTypeDef status;
    uint32_t guard = RC_DMA_DISABLE_GUARD_COUNT;

    // 使用两个36字节缓冲区，UART空闲中断按18字节DBUS帧切换
    if ((huart == NULL) || (huart->hdmarx == NULL) ||
        (memory0 == NULL) || (memory1 == NULL) ||
        (memory0 == memory1) || (buffer_length == 0U))
    {
        return HAL_ERROR;
    }

    hdma = huart->hdmarx;
    stream = (DMA_Stream_TypeDef *)hdma->Instance;

    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_DMA_DISABLE(hdma);
    while (((stream->CR & DMA_SxCR_EN) != 0U) && (guard > 0U))
    {
        --guard;
    }
    if ((stream->CR & DMA_SxCR_EN) != 0U)
    {
        return HAL_TIMEOUT;
    }

    CLEAR_BIT(stream->CR, DMA_SxCR_DBM | DMA_SxCR_CT);

    huart->pRxBuffPtr = memory0;
    huart->RxXferSize = buffer_length;
    huart->RxXferCount = buffer_length;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_IDLE;

    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);

    // 双缓冲DMA直接从UART接收寄存器搬运，任务不参与逐字节接收
    status = HAL_DMAEx_MultiBufferStart(hdma, (uint32_t)(uintptr_t)&huart->Instance->RDR, (uint32_t)(uintptr_t)memory0, (uint32_t)(uintptr_t)memory1, buffer_length);
    if (status != HAL_OK)
    {
        huart->RxState = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        return status;
    }

    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

    return HAL_OK;
}
