/**
 ****************************************************************************************************
 * @file        lvgl_demo.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       LVGL SPI Flash读取XBF字库
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
 
#ifndef __LVGL_DEMO_H
#define __LVGL_DEMO_H

#include "lvgl.h"


/* 函数声明 */
void lvgl_demo(void);                                                                               /* lvgl_demo入口函数 */
void lv_port_disp_init(void);                                                                       /* 初始化并注册显示设备 */
void lv_port_indev_init(void);                                                                      /* 初始化并注册输入设备 */
void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);                               /* 图形库的触摸屏读取回调函数 */
static void increase_lvgl_tick(void *arg);                                                          /* 告诉LVGL运行时间 */
static void lvgl_disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);   /* 将内部缓冲区的内容刷新到显示屏上的特定区域 */

#endif
