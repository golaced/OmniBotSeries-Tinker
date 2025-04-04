#ifndef __LOCK_H
#define	__LOCK_H
#include "stm32f10x.h"

extern void Lock_init(void);
#define LOCK1_OPEN  GPIO_SetBits(GPIOB,GPIO_Pin_0)//输出0，关闭蜂鸣器输出
#define LOCK1_CLOSE GPIO_ResetBits(GPIOB,GPIO_Pin_0)//输出0，关闭蜂鸣器输出

#define LOCK2_OPEN  GPIO_SetBits(GPIOB,GPIO_Pin_1)//输出0，关闭蜂鸣器输出
#define LOCK2_CLOSE GPIO_ResetBits(GPIOB,GPIO_Pin_1)//输出0，关闭蜂鸣器输出

#define  LED_GREEN_ON GPIO_SetBits(GPIOB,GPIO_Pin_8)//输出0，关闭蜂鸣器输出;
#define  LED_GREEN_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_8)//输出0，关闭蜂鸣器输出;

#define  LED_RED_ON GPIO_SetBits(GPIOB,GPIO_Pin_9)//输出0，关闭蜂鸣器输出;
#define  LED_RED_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_9)//输出0，关闭蜂鸣器输出;

#define  Get_open_key GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)//输出0，关闭蜂鸣器输出;
extern void Lock_tast(u8 lock , u8 state);
extern void Lock_Task(void);
#endif /* __USART_H */
