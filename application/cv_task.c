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

uart_stability *uart_shooting_status; //uart shooting structure

#define USART1_RX_BUFFER_SIZE 20
uint8_t usart1_buf[2][USART1_RX_BUFFER_SIZE];
uint8_t *usart1_data; // newest data pointer

#define FLOAT_LEN = 3 // yaw_add, pitch_add, shoot in fp32
fp32 *cv_float_data;

gimbal_control_t *cv_task_gimbal; //address to gimbal details

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
	
		//initialize uart shooting detection
		uart_shooting_status->stability = 0;
		uart_shooting_status->detected_enemy = SHOOT_PAUSE;
		uart_shooting_status->uart_reading = NO_DETECTION;
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
						if(uart_shooting_status->stability > 4){
								uart_shooting_status->detected_enemy = SHOOT_START;
						}
						else{
								uart_shooting_status->detected_enemy = SHOOT_PAUSE;
						}
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
    // clear out data
    cv_float_data[0] = -2;
    cv_float_data[1] = 0;
    cv_float_data[2] = 0;

		if(yaw_add != -2){ //confirm that there was a reading
				uart_shooting_status->uart_reading = DETECTED;
			  // set angle
				gimbal_set_angle(&(cv_task_gimbal->gimbal_yaw_motor), yaw_add);
				gimbal_set_angle(&(cv_task_gimbal->gimbal_pitch_motor), pitch_add);
		}
		else{
				uart_shooting_status->uart_reading = NO_DETECTION;
		}
}
// This function moves the gimbal based on the value given from UART
static void gimbal_set_angle(gimbal_motor_t *gimbal_motor, fp32 add)
{	
		if( add < 0.002f && add > -0.002f && add != 0){ //Interval that determines stable aiming
				if(uart_shooting_status->stability < 10){
						uart_shooting_status->stability += 1; //Increase stability reading if in range
				}
		}
		else{ //decrease stability if not in range
				if(uart_shooting_status->stability < 4){
						uart_shooting_status->stability = 0; 
				}
				else{
						uart_shooting_status->stability -= 2;
				}
		}
		if(add > 0.002f){ //minimum add value
				while(1){
						if(add < .01f){
								gimbal_motor->relative_angle_set += add;
								add = 0.0f;
						}
						else if( add < .03f){ //slows down motor near end
								gimbal_motor->relative_angle_set += 0.01f;
								add -= 0.01f;
						}
						else{
								gimbal_motor->relative_angle_set += 0.03f;
								add -= 0.03f;
						}
						// check whether the limit is exceed
						if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle){
								gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
								return;
						}
						else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle){
								gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
								return;
						}
						osDelay(100);
				}
	}else if(add < -0.002f){
			while(1){
					if(add < -.01f){
					gimbal_motor->relative_angle_set += add;
					add = 0.0f;
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
					if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle){
							gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
							return;
					}
					else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle){
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
