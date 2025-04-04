/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       串口初始化代码(一般是串口0)
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32S3 BOX 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#ifndef _USART_H
#define _USART_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/uart_select.h"
#include "driver/gpio.h"


/* 引脚和串口定义 */
#define USART_UX            UART_NUM_1
#define USART_TX_GPIO_PIN   GPIO_NUM_17
#define USART_RX_GPIO_PIN   GPIO_NUM_18

/* 串口接收相关定义 */
#define RX_BUF_SIZE         1024    /* 环形缓冲区大小 */

/* 函数声明 */
void usart_init(uint32_t baudrate); /* 初始化串口 */

typedef struct 
{   char connect;
    int loss_cnt;
	float x;
	float y;
	float bat,bat_percent;
}joy;
extern joy adc_rc[2];
extern char key_rc[2];
extern char key_sel[4];
#endif
