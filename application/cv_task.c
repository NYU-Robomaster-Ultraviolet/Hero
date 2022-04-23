/**
 * @file cv_task.c
 * @author Jerry Wu
 * @brief cv related tasks
 * @version 0.
 * @date 2022-04-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "cv_task.h"

// TODO
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h" // TODO: delete
#include "fifo.h"       // TODO: delete
#include "protocol.h"   // TODO

/**
 * @brief
 *
 */
void cv_process(void);

extern UART_HandleTypeDef huart1;

#define USART1_RX_BUFFER_SIZE 10
uint8_t usart1_buf[2][USART1_RX_BUFFER_SIZE];
uint8_t *usart1_data; // newest data pointer

/**
 * @brief
 *
 */
// TODO
void cv_usart_task(void const *argument)
{
    // usart init
    usart1_init(usart1_buf[0], usart1_buf[1], USART1_RX_BUFFER_SIZE);

    while (1)
    {
        cv_process();
        osDelay(100); // 1/100ms = 10Hz processing rate
    }
}

void cv_process(void)
{
    // process data (TODO in future)
    // send processed data
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART1_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_RX_BUFFER_SIZE);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            usart1_data = usart1_buf[0];
        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART1_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART1_RX_BUFFER_SIZE);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            usart1_data = usart1_buf[1];
        }
        if (this_time_rx_len > 0)
        {

            usart1_tx_dma_enable(usart1_data, this_time_rx_len); // TODO
        }
    }
}
