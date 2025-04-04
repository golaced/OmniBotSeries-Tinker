/**
 ****************************************************************************************************
 * @file        lv_mainstart.c
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
 
#include "lv_mainstart.h"
#include "lvgl.h"


LV_FONT_DECLARE(Font24)         /* 声明Font24字体,这个字体已经烧写到SPIFLASH中 */
static lv_style_t font_style;   /* 定义样式变量 */
lv_obj_t *font_label;           /* 定义一个字体标签指针 */

/**
 * @brief  LVGL演示
 * @param  无
 * @return 无
 */
void lv_mainstart(void)
{
    lv_style_init(&font_style);                             /* 初始化样式 */
    lv_style_set_text_font(&font_style, &Font24);           /* 设置文本字体属性 */
    lv_style_set_text_color(&font_style, lv_color_black()); /* 设置文本颜色属性 */
    
    font_label = lv_label_create(lv_scr_act());             /* 创建一个标签 */
    lv_obj_add_style(font_label, &font_style, 0);           /* 添加字体标签的样式 */
    lv_label_set_text(font_label, "正点原子: Hello LVGL!");  /* 设置文本 */
    lv_obj_align(font_label, LV_ALIGN_TOP_MID, 0, 0);       /* 顶部中间对齐显示 */
}
