/**
 ****************************************************************************************************
 * @file        touch.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       触摸屏 驱动代码
 * @note        支持电容式触摸屏
 *
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32S3 BOX 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.oT_PENedv.com
 * 公司网址:www.alientek.com
 * 购买地址:oT_PENedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "touch.h"


_m_tp_dev tp_dev =
{{
    tp_init,
    0,
    0,
    0,
    0,
    0,
}};

/**
 * @brief       触摸屏初始化
 * @param       无
 * @retval      0,触摸屏初始化成功
 *              1,触摸屏有问题
 */
uint8_t tp_init(void)
{
    tp_dev.touchtype = 0;                       /* 默认设置(电阻屏 & 竖屏) */
    tp_dev.touchtype |= lcd_dev.dir & 0X01;     /* 根据LCD判定是横屏还是竖屏 */

    chsc5xxx_init();
    tp_dev.scan = chsc5xxx_scan;                /* 扫描函数指向CHSC5xxx触摸屏扫描 */
    tp_dev.touchtype |= 0X80;
    return 0;
}
