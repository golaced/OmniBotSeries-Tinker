#ifndef __BEEP_H
#define __BEEP_H	 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//蜂鸣器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//蜂鸣器端口定义
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
void Beep_Init(u32 arr,u32 psc);
void Tone(u8 level, u8 tone);
void Play_Music_Direct(u8 sel);
u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt);
void Play_Music_Task(u8 sel,float dt);
#define MEMS_RIGHT_BEEP 0
#define MEMS_ERROR_BEEP 1
#define START_BEEP 2
#define BAT_ERO_BEEP 3
#define RC_ERO_BEEP 4
#define RC_RESET_BEEP 5
#define UPLOAD_BEEP 6
#endif

