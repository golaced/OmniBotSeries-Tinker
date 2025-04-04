#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"
#include <stdio.h>
#include "usart_config.h"
#include "stm32f10x_dma.h"

void USART_init(void);



#define USART1_DR_Base  0x40013804		// 0x40013800 + 0x04 = 0x40013804
#define USART2_DR_Base  0x40004404		// 0x4000 4400 + 0x04 = 
#define USART3_DR_Base  0x40004804		// 0x40013800 + 0x04 = 
#define SENDBUFF_SIZE   20//DMA1 size

extern u8 dma_can_tx;
extern u8 uart_test[4];

extern void UART_TX_CHAR(u8 sel ,char data);
#define Set_RE GPIO_SetBits(GPIOA,GPIO_Pin_11)//
#define Reset_RE GPIO_ResetBits(GPIOA,GPIO_Pin_11)//
#define Set_DE GPIO_SetBits(GPIOA,GPIO_Pin_12)//
#define Reset_DE GPIO_ResetBits(GPIOA,GPIO_Pin_12)//
#define EN_485_RX  GPIO_ResetBits(GPIOA,GPIO_Pin_11);GPIO_SetBits(GPIOA,GPIO_Pin_12)	//关闭485模块发射使能，打开接收功能
#define EN_485_TX  GPIO_SetBits(GPIOA,GPIO_Pin_11);GPIO_ResetBits(GPIOA,GPIO_Pin_12)	//打开485模块发射使能，关闭接收功能
extern void RS485_Init(void);
#define MAX_NUM 125
extern char RX_BUF[MAX_NUM],RX_BUF_OUT[MAX_NUM];
extern u8 open_lock1;


typedef struct
{
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	u16 active_time;
}TimeStruct_Card;

typedef struct card{
				u16 id;
				TimeStruct_Card time_s;
	      TimeStruct_Card time_e;
	      u8 times;
		}ID;
#define MAX_SAVE 10
extern ID CARD[MAX_SAVE];
extern void my_itoa(int num,u8* str);  
		
void Send_To_Wio(void);
#endif /* __USART_H */
