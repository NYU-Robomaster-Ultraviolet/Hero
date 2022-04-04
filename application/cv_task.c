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
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"


/**
 * @brief 
 * 
 */
// TODO
void cv_usart_task(void const * argument)
{
    // pass
    while(1)
    {
        osDelay(100);  // 1/100ms = 10Hz processing rate
    }
}