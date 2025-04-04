#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//SPI 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/13 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

 
 				  	    													  
void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 Spi_RW(u8 TxData);//SPI总线读写一个字节
void SPI_CS(u8 sel,u8 set);		 
		 
#define CS_MPU9250 1
#define CS_SD 2
#define CS_NRF  3
#define CS_MAG 4

#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_3) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_3)

#define SPI_CSN_H()  SPI_CS(CS_NRF,1)
#define SPI_CSN_L()  SPI_CS(CS_NRF,0)



uint8_t SPI_Write_Reg(u8 sel,uint8_t reg,uint8_t value);
uint8_t SPI_Read_Reg(u8 sel,uint8_t reg);
uint8_t SPI_Write_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars);
uint8_t SPI_Read_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars);
void SPI_BufferRead(u8 sel,u8*buf, u8 add, u8 len);
void SPI_Transmit(uint8_t *pData, uint16_t Size);
void SPI_Receive(uint8_t *pData, uint16_t Size);



#endif

