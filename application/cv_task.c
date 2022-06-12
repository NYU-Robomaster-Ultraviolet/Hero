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
#include "gimbal_task.h"
#include "remote_control.h"
#include "shoot.h"

/**
 * @brief
 *
 */
void cv_process(void);

extern UART_HandleTypeDef huart1;

#define USART1_RX_BUFFER_SIZE 20
uint8_t usart1_buf[2][USART1_RX_BUFFER_SIZE];
uint8_t *usart1_data; // newest data pointer

#define FLOAT_LEN = 3 // yaw_add, pitch_add, shoot in fp32
fp32 *cv_float_data;

gimbal_control_t *cv_task_gimbal;


void gimbal_set_angle(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief
 *
 */
// TODO
void cv_usart_task(void const *argument)
{
    // usart init
    usart1_init(usart1_buf[0], usart1_buf[1], USART1_RX_BUFFER_SIZE);

    // getting the gimbal_control_t struct
    cv_task_gimbal = get_gimbal_control_point();

    // clear cv data for both buffer
    cv_float_data = (fp32 *)usart1_buf[0];
    cv_float_data[0] = 0;
    cv_float_data[1] = 0;
    cv_float_data[2] = 0;
    cv_float_data = (fp32 *)usart1_buf[1];
    cv_float_data[0] = 0;
    cv_float_data[1] = 0;
    cv_float_data[2] = 0;

    while (1)
    {
        // switch in middle is the cv mode
        if (switch_is_mid(cv_task_gimbal->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            cv_process();
        }
        osDelay(100); // 1/100ms = 10Hz processing rate
    }
}

void cv_process(void)
{
    // no gimbal value available yet TODO: could be error
    if (cv_task_gimbal == NULL)
    {
        return;
    }

    // pointing to newest data
    cv_float_data = (fp32 *)usart1_data;

    // parse data
    fp32 yaw_add = cv_float_data[0];
    fp32 pitch_add = cv_float_data[1];
    //fp32 shooting = cv_float_data[2]; // 0 for not shoot and 1 for shoot
		//get_uart_mode(cv_float_data[2]);
    // clear out data
    cv_float_data[0] = 0;
    cv_float_data[1] = 0;
    cv_float_data[2] = 0;

    // set angle
    gimbal_set_angle(&(cv_task_gimbal->gimbal_yaw_motor), yaw_add);
    gimbal_set_angle(&(cv_task_gimbal->gimbal_pitch_motor), pitch_add);
}
// This function moves the gimbal based on the value given from UART
static void gimbal_set_angle(gimbal_motor_t *gimbal_motor, fp32 add)
{
	  if (gimbal_motor == NULL)
    {
        return;
    }
		if(add > 0.0f){
		while(1){
			if(add < .01f){
				gimbal_motor->relative_angle_set += add;
				add = 0.0f;
				return;
			}
			else if( add < .03f){
				gimbal_motor->relative_angle_set += 0.01f;
				add -= 0.01f;
			}
			else{
				gimbal_motor->relative_angle_set += 0.03f;
				add -= 0.03f;
			}
			// check whether the limit is exceed
			if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
			{
					gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
				  return;
			}
			else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
			{
					gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
				  return;
			}
			osDelay(100);
	  }
	}else if(add < 0.0f){
		while(1){
			if(add < -.01f){
				gimbal_motor->relative_angle_set += add;
				add = 0.0f;
				return;
			}
			else if( add < -.03f){
				gimbal_motor->relative_angle_set -= 0.01f;
				add += 0.015f;
			}
			else{
				gimbal_motor->relative_angle_set -= 0.03f;
				add += 0.03f;
			}
			// check whether the limit is exceed
			if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
			{
					gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
				  return;
			}
			else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
			{
					gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
				  return;
			}
			osDelay(100);
	  }
	}
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
