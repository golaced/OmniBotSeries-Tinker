/**
 ****************************************************************************************************
 * @file        xl9555.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       XL9555驱动代码
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

#ifndef __XL9555_H
#define __XL9555_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iic.h"


/* 引脚与相关参数定义 */
#define XL9555_INT_IO               GPIO_NUM_3                      /* XL9555_INT引脚 */
#define XL9555_INT                  gpio_get_level(XL9555_INT_IO)   /* 读取XL9555_INT的电平 */

/* XL9555命令宏 */
#define XL9555_INPUT_PORT0_REG      0                               /* 输入寄存器0地址 */
#define XL9555_INPUT_PORT1_REG      1                               /* 输入寄存器1地址 */
#define XL9555_OUTPUT_PORT0_REG     2                               /* 输出寄存器0地址 */
#define XL9555_OUTPUT_PORT1_REG     3                               /* 输出寄存器1地址 */
#define XL9555_INVERSION_PORT0_REG  4                               /* 极性反转寄存器0地址 */
#define XL9555_INVERSION_PORT1_REG  5                               /* 极性反转寄存器1地址 */
#define XL9555_CONFIG_PORT0_REG     6                               /* 方向配置寄存器0地址 */
#define XL9555_CONFIG_PORT1_REG     7                               /* 方向配置寄存器1地址 */

#define XL9555_ADDR                 0X20                            /* XL9555地址(左移了一位)-->请看手册（9.1. Device Address） */

/* XL9555各个IO的功能 */
#define AP_INT_IO                   0x0001
#define QMA_INT_IO                  0x0002
#define BEEP_IO                     0x0004
#define KEY1_IO                     0x0008
#define KEY0_IO                     0x0010
#define SPK_CTRL_IO                 0x0020
#define CTP_RST_IO                  0x0040
#define LCD_BL_IO                   0x0080
#define LEDR_IO                     0x0100
#define CTP_INT_IO                  0x0200
#define IO1_2                       0x0400
#define IO1_3                       0x0800
#define IO1_4                       0x1000
#define IO1_5                       0x2000
#define IO1_6                       0x4000
#define IO1_7                       0x8000

#define KEY0                        xl9555_pin_read(KEY0_IO)        /* 读取K1引脚 */
#define KEY1                        xl9555_pin_read(KEY1_IO)        /* 读取K2引脚 */

#define KEY0_PRES                   2                               /* K1按下 */
#define KEY1_PRES                   3                               /* K2按下 */

#define LEDR_TOGGLE()    do { xl9555_pin_write(LEDR_IO, !xl9555_pin_read(LEDR_IO)); } while(0)  /* LEDR翻转 */

/* 函数声明 */
void xl9555_init(i2c_obj_t self);                                   /* 初始化XL9555 */
int xl9555_pin_read(uint16_t pin);                                  /* 获取某个IO状态 */
uint16_t xl9555_pin_write(uint16_t pin, int val);                   /* 控制某个IO的电平 */
esp_err_t xl9555_read_byte(uint8_t* data, size_t len);              /* 读取XL9555的16位IO值 */
uint8_t xl9555_key_scan(uint8_t mode);                              /* 扫描按键值 */

#endif
