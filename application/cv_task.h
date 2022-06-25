/**
 * @file cv_task.c
 * @author Jerry Wu
 * @brief cv related tasks
 * @version 0.1
 * @date 2022-04-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef CV_TASK_H
#define CV_TASK_H
#include "main.h"  // TODO: is it neccessary?

extern void cv_usart_task(void const *argument); // todo

typedef enum
{ 
	SHOOT_PAUSE = 0,
	SHOOT_START = 1
} uart_shoot_mode_e;

typedef enum
{
	NO_DETECTION = 0,
	DETECTED = 1
}uart_detected;

typedef struct
{
	uint16_t stability;
	uart_shoot_mode_e detected_enemy;
	uart_detected uart_reading;
} uart_stability;

#endif
