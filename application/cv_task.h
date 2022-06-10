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

uint16_t uart_shooting = 0;

typedef struct
{
	float uart_yaw_add;
	float uart_pitch_add;
	uint16_t uart_shooting;
}uart_instruction_t;

#endif
