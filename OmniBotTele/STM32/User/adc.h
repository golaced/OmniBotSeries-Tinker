#ifndef __ADC_H
#define __ADC_H	
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//ADC 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define  ADC_BAT		0
#define  ADC_YAW		2
#define  ADC_THRUST		1

//初始化ADC，使用DMA传输
void Adc_Init(void);
void  Adc_Init1(void);
u16 getAdcValue1(uint8_t axis);
void ADC_Filter(uint16_t* adc_val);	//ADC均值滤波
uint16_t getAdcValue(uint8_t axis);
extern int rc_off[2];
//飞控数据结构
typedef struct 
{
	float x;
	float y;
	float bat,bat_percent;
}joystickFlyui16_t;
extern joystickFlyui16_t adc_rc[2];
void getFlyDataADCValue(void);
#endif 
