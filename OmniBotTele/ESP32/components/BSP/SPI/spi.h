/**
 ****************************************************************************************************
 * @file        spi.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       SPI驱动代码
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

#ifndef __SPI_H
#define __SPI_H

#include <string.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


/* 引脚定义 */
#define SPI_MOSI_GPIO_PIN   GPIO_NUM_16         /* SPI2_MOSI */
#define SPI_CLK_GPIO_PIN    GPIO_NUM_7         /* SPI2_CLK */
#define SPI_MISO_GPIO_PIN   GPIO_NUM_15         /* SPI2_MISO */

/* 函数声明 */
void spi2_init(void);                                                               /* 初始化SPI2 */
void spi2_write_cmd(spi_device_handle_t handle, uint8_t cmd);                       /* SPI发送命令 */
void spi2_write_data(spi_device_handle_t handle, const uint8_t *data, int len);     /* SPI发送数据 */
uint8_t spi2_transfer_byte(spi_device_handle_t handle, uint8_t byte);               /* SPI处理数据 */
#endif
