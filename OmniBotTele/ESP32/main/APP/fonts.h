/**
 ****************************************************************************************************
 * @file        font.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       字库 代码
 *              提供fonts_update_font和fonts_init用于字库更新和初始化
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

#ifndef __FONTS_H
#define __FONTS_H

#include "esp_partition.h"
#include "spi_flash_mmap.h"
#include "esp_log.h"
#include "ff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/* 字体信息保存首地址
 * 占33个字节,第1个字节用于标记字库是否存在.后续每8个字节一组,分别保存起始地址和文件大小
 */
extern uint32_t FONTINFOADDR;

/* 字库信息结构体定义
 * 用来保存字库基本信息，地址，大小等
 */
typedef struct
{
    uint8_t fontok;             /* 字库存在标志，0XAA，字库正常；其他，字库不存在 */
    uint32_t ugbkaddr;          /* unigbk的地址 */
    uint32_t ugbksize;          /* unigbk的大小 */
    uint32_t f12addr;           /* gbk12地址 */
    uint32_t gbk12size;         /* gbk12的大小 */
    uint32_t f16addr;           /* gbk16地址 */
    uint32_t gbk16size;         /* gbk16的大小 */
    uint32_t f24addr;           /* gbk24地址 */
    uint32_t gbk24size;         /* gbk24的大小 */

    uint32_t lvgl_12addr;       /* LVGL12地址 */
    uint32_t lvgl_12size;       /* LVGL12的大小 */
    uint32_t lvgl_24addr;       /* LVGL24地址 */
    uint32_t lvgl_24size;       /* LVGL24的大小 */
    uint32_t lvgl_36addr;       /* LVGL36地址 */
    uint32_t lvgl_36size;       /* LVGL36的大小 */
} _font_info;

/* 字库信息结构体 */
extern _font_info ftinfo;

/* 函数声明 */
esp_err_t fonts_partition_read(void * buffer, uint32_t offset, uint32_t length);                                /* 分区读取 */
esp_err_t fonts_partition_write(void * buffer, uint32_t offset, uint32_t length);                               /* 分区写入 */
esp_err_t fonts_partition_erase_sector(uint32_t offset);                                                        /* 分区擦除 */
uint8_t fonts_update_font(uint16_t x, uint16_t y, uint8_t size, uint8_t *src, uint16_t color);                  /* 更新全部字库 */
void fonts_progress_show(uint16_t x, uint16_t y, uint8_t size, uint32_t totsize, uint32_t pos, uint16_t color); /* 显示当前字体更新进度 */
uint8_t fonts_init(void);                                                                                       /* 初始化字库 */

#endif
