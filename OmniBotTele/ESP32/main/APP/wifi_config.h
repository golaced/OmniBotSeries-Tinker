/**
 ****************************************************************************************************
 * @file        wifi_config.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-26
 * @brief       网络连接配置
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32S3 BOX开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __WIFI_CONFIG_H
#define __WIFI_CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include <netdb.h>
#include "lcd.h"


/* WIFI设备信息 */
typedef struct _network_connet_info_t
{
    uint8_t         connet_state;    /* 网络连接状态 */
    char            ip_buf[100];     /* 分配的IP地址 */
    char            mac_buf[100];    /* 子网掩码 */
    void (*fun)(uint8_t x);          /* 函数指针 */
} network_connet_info;

extern network_connet_info network_connet;

/* 声明函数 */
void wifi_sta_init(wifi_config_t input_wifi);

#endif
