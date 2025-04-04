#include "nrf.h"
#include "spi.h"
#include "head.h"
uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01??????
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01???????
u8  TX_ADDRESS[TX_ADR_WIDTH]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//????
u8  RX_ADDRESS[RX_ADR_WIDTH]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//????

/*
*****************************************************************
* ????
*****************************************************************
*/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	SPI_CSN_L();					  /* ???? */
	status = Spi_RW(reg);  /* ?????? */
	Spi_RW(value);		  /* ??? */
	SPI_CSN_H();					  /* ????? */
  return 	status;
}
/*
*****************************************************************
* ????
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI_CSN_L();					  /* ???? */
	Spi_RW(reg);			  /* ?????? */
	reg_val = Spi_RW(0);	  /* ?????????? */
	SPI_CSN_H();					  /* ????? */
    return 	reg_val;
}
/*
*****************************************************************
*
* ????
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();				        /* ???? */
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		Spi_RW(pBuf[i]);		/* ??? */
	}
	SPI_CSN_H();						/* ????? */
    return 	status;	
}
/*
*****************************************************************
* ????
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();						/* ???? */
	status = Spi_RW(reg);	/* ?????? */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = Spi_RW(0); /* ?????? */ 	
	}
	SPI_CSN_H();						/* ????? */
    return 	status;
}

void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	SPI_CE_L();		 //StandBy I??	
	
	SPI_Write_Buf(CS_NRF,NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // ???????
	SPI_Write_Buf(CS_NRF,WR_TX_PLOAD, tx_buf, len); 			 // ????	
	SPI_CE_H();		 //??CE,??????
}
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	SPI_CE_L();		 //StandBy I??	
	SPI_Write_Buf(CS_NRF,0xa8, tx_buf, len); 			 // ????
	SPI_CE_H();		 //??CE
}
u8 Nrf24l01_Check(void)
{ 
	u8 buf1[5]={0}; 
	u8 i; 
	/*??5??????. */ 
	SPI_Write_Buf(CS_NRF,NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*??????? */ 
	SPI_Read_Buf(CS_NRF,TX_ADDR,buf1,5); 
	/*??*/ 
	for(i=0;i<5;i++) 
	{ 
		if(buf1[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5){
		module.nrf=1;
		return SUCCESS ; //MCU?NRF???? 
	  }
	else{
	  module.nrf=0;
		return ERROR ; //MCU?NRF????? 
	}
}


#define STATUS          0x07  //?????;bit0:TX FIFO???;bit3:1,???????(??:6);bit4,???????
#define WRITE_REG_NRF       0x20  //??????,?5???????
#define RX_OK   		0x40  //???????
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   

	sta=SPI_Read_Reg(CS_NRF,STATUS);  //?????????    	 
	SPI_Write_Reg(CS_NRF,WRITE_REG_NRF+STATUS,sta); //??TX_DS?MAX_RT????
	if(sta&RX_OK)//?????
	{
		SPI_Read_Buf(CS_NRF,RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//????
		SPI_Write_Reg(CS_NRF,FLUSH_RX,0xff);//??RX FIFO??? 
		return 0; 
	}	   
	return 1;//???????
}		
//  
// void EXTIX_Init(void)
// {
// 	NVIC_InitTypeDef   NVIC_InitStructure;
// 	EXTI_InitTypeDef   EXTI_InitStructure;
// 	
//   GPIO_InitTypeDef  GPIO_InitStructure;

//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA,GPIOE时钟
//  
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //KEY0 KEY1 KEY2对应引脚
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//   GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOE2,3,4
//  
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
// 	
// 	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);//PE4 连接到中断线4
// 	
// 	/* 配置EXTI_Line2,3,4 */
// 	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
//   EXTI_Init(&EXTI_InitStructure);//配置
// 	
// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断4
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级1
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
//   NVIC_Init(&NVIC_InitStructure);//配置
// 	   
// }

void Nrf24l01_Init(u8 model, u8 ch)
{
	SPI_CE_L();
	SPI_Write_Buf(CS_NRF,NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//?RX???? 
	SPI_Write_Buf(CS_NRF,NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//?TX????  
	SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+EN_AA,0x01); 													//????0????? 
	SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+EN_RXADDR,0x01);											//????0????? 
	SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+SETUP_RETR,0x1a);											//??????????:500us;????????:10? 
	SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+RF_CH,ch);														//??RF???CHANAL
	SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+RF_SETUP,0x0f); 											//??TX????,0db??,2Mbps,???????
//SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+RF_SETUP,0x07); 										  //??TX????,0db??,1Mbps,???????
//	EXTIX_Init();
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//????0??????? 
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ????????,16?CRC,???
	}
	else if(model==2)		//TX
	{
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//????0??????? 
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ????????,16?CRC,???
	}
	else if(model==3)		//RX2	???
	{
		SPI_Write_Reg(CS_NRF,FLUSH_TX,0xff);
		SPI_Write_Reg(CS_NRF,FLUSH_RX,0xff);
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ????????,16?CRC,???
		SPI_CSN_L();	
		Spi_RW(0x50);
		Spi_RW(0x73);
		SPI_CSN_H();	
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+0x1c,0x01);
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+0x1d,0x07);
	}
	else								//TX2	???
	{
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ????????,16?CRC,???
		SPI_Write_Reg(CS_NRF,FLUSH_TX,0xff);
		SPI_Write_Reg(CS_NRF,FLUSH_RX,0xff);
		
		SPI_CSN_L();	
		Spi_RW(0x50);
		Spi_RW(0x73);
		SPI_CSN_H();	
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+0x1c,0x01);
		SPI_Write_Reg(CS_NRF,NRF_WRITE_REG+0x1d,0x07);
	}
	SPI_CE_H();
}


void EXTI0_IRQHandler(void)
{
   
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除EXTI0线路挂起位
}

////////////////////////////////////////////////////////////////////////////////
