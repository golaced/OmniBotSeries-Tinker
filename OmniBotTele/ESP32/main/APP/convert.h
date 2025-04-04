/**
 ****************************************************************************************************
* @file        convert.h
* @author      正点原子团队(ALIENTEK)
* @version     V1.0
* @date        2023-12-01
* @brief       UTF8与GBK互转 代码
* @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
****************************************************************************************************
* @attention
*
* 实验平台:正点原子 ESP32-S3 开发板
* 在线视频:www.yuanzige.com
* 技术论坛:www.openedv.com
* 公司网址:www.alientek.com
* 购买地址:openedv.taobao.com
*
****************************************************************************************************
*/

#ifndef CONVERY_H_
#define CONVERY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* 数组大小 */
#define ARRAY_SIZE(a)       ((sizeof(a))/(sizeof((a)[0])))

/* 函数声明 */
int gbk2utf8(char **ptr, void *pin_buf, int in_len);    /* 将 gbk转为 utf8 */
int utf82gbk(char **ptr, void *pin_buf, int in_len);    /* 将 utf8转为 gbk */
void convet_test(void);                                 /* 编码转换测试 */

#endif
