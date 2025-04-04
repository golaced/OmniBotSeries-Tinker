#include "spi.h"
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
 

//以下是SPI模块的初始化代码，配置成主机模式，访问SD Card/W25X16/24L01/JF24C							  
//SPI口初始化
//这里针是对SPI1的初始化

SPI_InitTypeDef  SPI_InitStructure;

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_7; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_3; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_CS(CS_SD,1);
	SPI_CS(CS_NRF,1);
	SPI_CS(CS_MPU9250,1);
	
	//这里只针对SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //????? 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //??? 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //????8? 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //????,????? 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //?1?????,???????? 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS??????? 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8??,9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //???? 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	 
}   
//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
  
void SPI1_SetSpeed(u8 SpeedSet)
{
	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1,ENABLE);
} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 Spi_RW1(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}


u8 Spi_RW(u8 dat) 
{ 
	/* ? SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
	/* ?? SPI2??????? */ 
	SPI_I2S_SendData(SPI1, dat); 
	/* ?SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI1); 
}


void SPI_CS(u8 sel,u8 set)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_7);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	delay_us(10);
switch(sel)
{
	case 1:
  if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	delay_us(10);
	break;
	case 2:
	if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	delay_us(10);
	break;
	case 3:
	if(set)	
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	else
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	delay_us(10);
	break;
}
}	


//====value:写入的值
uint8_t SPI_Write_Reg(u8 sel,uint8_t reg,uint8_t value)
{
	uint8_t status;
	SPI_CS(sel,0);
	if (sel==CS_MAG)
		reg=0x80|reg;
	
	status=Spi_RW(reg); //发送写命令+寄存器号
	Spi_RW(value);//写入寄存器值
	SPI_CS(sel,1);
	return(status);//返回状态值
}

//====SPI读取寄存器
//====reg:指定的寄存器地址
uint8_t SPI_Read_Reg(u8 sel,uint8_t reg)
{
	uint8_t reg_val;
	SPI_CS(sel,0);
	Spi_RW(reg|0x80); //====发送读命令+寄存器号
	reg_val=Spi_RW(0xff);//====读取寄存器值
	SPI_CS(sel,1);
	return(reg_val);
}


uint8_t SPI_Write_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CS(sel,0);
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		Spi_RW(pBuf[i]);		/* ??? */
	}
	SPI_CS(sel,1);
  return 	status;	
}


uint8_t SPI_Read_Buf(u8 sel,uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CS(sel,0);
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = Spi_RW(0); /* ?????? */ 	
	}
	SPI_CS(sel,1);
  return 	status;
}

void SPI_BufferRead(u8 sel,u8*buf, u8 add, u8 len)
{
	u8 i=0;
 SPI_CS(sel,0);
	if(sel!=CS_MAG)
	Spi_RW(add|0x80);	
	else
	Spi_RW(add|0xC0);//连续读增加地址
	for(i=0;i<len;i++)
	{
	if(sel!=CS_MAG)
	 *buf++ = Spi_RW(0xff); 
	else
	 *buf++ = Spi_RW(0xff); 
	} 
 SPI_CS(sel,1);
}

void SPI_Transmit(uint8_t *pData, uint16_t Size)
{uint16_t i;
    for( i=0; i<Size; i++)
    {
        Spi_RW(pData[i]);
    }
}

void SPI_Receive(uint8_t *pData, uint16_t Size)
{uint16_t i;
    for( i=0; i<Size; i++)
    {
        pData[i] = Spi_RW(0);
    }
}