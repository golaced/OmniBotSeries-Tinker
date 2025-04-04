/**
  ******************************************************************************
  * @file    SPI_MSD0_Driver.c
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   主函数.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "SPI_MSD0_Driver.h"
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
//#define PRINT_INFO  1	

/* Private variables ---------------------------------------------------------*/
MSD_CARDINFO SD0_CardInfo;

/*******************************************************************************
* Function Name  : MSD0_spi_read_write
* Description    : None
* Input          : - data:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
__inline int MSD0_spi_read_write(uint8_t data)
{
  //等待发送缓冲寄存器为空
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  //发送数据
  SPI_I2S_SendData(SPI2, data);		
  //等待接收缓冲寄存器为非空
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  //返回从SPI通信中接收到的数据
  return SPI_I2S_ReceiveData(SPI2);
}

/*******************************************************************************
* Function Name  : MSD0_SPI_Configuration
* Description    : SD Card SPI Configuration
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void MSD0_SPI_Configuration(void)
{		


GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 

  //使能APB2上相关时钟
  //使能SPI时钟，使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE );
    
  //定义一个GPIO结构体
 
  
  //SPI SCK MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  //SPI MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
	 //SPI MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode =   GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 

  MSD0_card_disable(); 

  MSD0_SPIHighSpeed(0);		

  SPI_Cmd(SPI2, ENABLE);
}

/*******************************************************************************
* Function Name  : MSD0_SPIHighSpeed
* Description    : SD Card Speed Set
* Input          : - b_high: 1 = 18MHz, 0 = 281.25Hz
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void MSD0_SPIHighSpeed(uint8_t b_high)
{
  SPI_InitTypeDef SPI_InitStructure;

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  /* Speed select */
  if(b_high == 0)
  {
	 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  }
  else
  {
	 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  }

  SPI_Init(SPI2, &SPI_InitStructure);
}

/*******************************************************************************
* Function Name  : MSD0_Init
* Description    : SD Card initializtion
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD0_Init(void)
{
	uint8_t r1;	
	uint8_t buff[6] = {0};
	uint16_t retry; 

	/* Check , if no card insert */


	/* Power on and delay some times */	
	for(retry=0; retry<0x100; retry++)
	{
		MSD0_card_power_on();
	}	

	/* Satrt send 74 clocks at least */
	for(retry=0; retry<10; retry++)
	{
		MSD0_spi_read_write(DUMMY_BYTE);
	}	
	
	/* Start send CMD0 till return 0x01 means in IDLE state */
	for(retry=0; retry<0xFFF; retry++)
	{
		r1 = MSD0_send_command(CMD0, 0, 0x95);
		if(r1 == 0x01)
		{
			retry = 0;
			break;
		}
	}
	/* Timeout return */
	if(retry == 0xFFF)
	{
#ifdef PRINT_INFO 
		printf("Reset card into IDLE state failed!\r\n");
#endif
		return 1;
	}
	
	/* Get the card type, version */
	r1 = MSD0_send_command_hold(CMD8, 0x1AA, 0x87);
	/* r1=0x05 -> V1.0 */
	if(r1 == 0x05)
	{
	  SD0_CardInfo.CardType = CARDTYPE_SDV1;

	  /* End of CMD8, chip disable and dummy byte */
	  MSD0_card_disable();
	  MSD0_spi_read_write(DUMMY_BYTE);
		
	  /* SD1.0/MMC start initialize */
	  /* Send CMD55+ACMD41, No-response is a MMC card, otherwise is a SD1.0 card */
	  for(retry=0; retry<0xFFF; retry++)
	  {
	     r1 = MSD0_send_command(CMD55, 0, 0);			/* should be return 0x01 */
		 if(r1 != 0x01)
		 {
#ifdef PRINT_INFO 
			printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
#endif
			return r1;
		 }

		 r1 = MSD0_send_command(ACMD41, 0, 0);			/* should be return 0x00 */
		 if(r1 == 0x00)
		 {
			retry = 0;
			break;
		 }
	  }

	  /* MMC card initialize start */
	  if(retry == 0xFFF)
	  {
		 for(retry=0; retry<0xFFF; retry++)
	     {
			 r1 = MSD0_send_command(CMD1, 0, 0);		/* should be return 0x00 */
			 if(r1 == 0x00)
			 {
				retry = 0;
				break;
			 }
		 }

		 /* Timeout return */
		 if(retry == 0xFFF)
		 {
#ifdef PRINT_INFO
			printf("Send CMD1 should return 0x00, response=0x%02x\r\n", r1);
#endif
			return 2;
		 }	
			
		SD0_CardInfo.CardType = CARDTYPE_MMC;		
#ifdef PRINT_INFO 
		printf("Card Type                     : MMC\r\n");
#endif 				
	  }		
		/* SD1.0 card detected, print information */
#ifdef PRINT_INFO
	  else
	  {
		 printf("Card Type                     : SD V1\r\n");
	  }
#endif 
		
	  /* Set spi speed high */
	  MSD0_SPIHighSpeed(1);		
		
	  /* CRC disable */
	  r1 = MSD0_send_command(CMD59, 0, 0x01);
	  if(r1 != 0x00)
	  {
#ifdef PRINT_INFO 
		  printf("Send CMD59 should return 0x00, response=0x%02x\r\n", r1);
#endif
		  return r1;		/* response error, return r1 */
	  }
		  
	  /* Set the block size */
	  r1 = MSD0_send_command(CMD16, MSD_BLOCKSIZE, 0xFF);
	  if(r1 != 0x00)
	  {
#ifdef PRINT_INFO
		  printf("Send CMD16 should return 0x00, response=0x%02x\r\n", r1);
#endif
		  return r1;		/* response error, return r1 */
	  }
   }	
	
   /* r1=0x01 -> V2.x, read OCR register, check version */
   else if(r1 == 0x01)
   {
	 /* 4Bytes returned after CMD8 sent	*/
	 buff[0] = MSD0_spi_read_write(DUMMY_BYTE);				/* should be 0x00 */
	 buff[1] = MSD0_spi_read_write(DUMMY_BYTE);				/* should be 0x00 */
	 buff[2] = MSD0_spi_read_write(DUMMY_BYTE);				/* should be 0x01 */
	 buff[3] = MSD0_spi_read_write(DUMMY_BYTE);				/* should be 0xAA */
		
	 /* End of CMD8, chip disable and dummy byte */ 
	 MSD0_card_disable();
	 MSD0_spi_read_write(DUMMY_BYTE);
		
	 /* Check voltage range be 2.7-3.6V	*/
	 if(buff[2]==0x01 && buff[3]==0xAA)
	 {
		for(retry=0; retry<0xFFF; retry++)
		{
			r1 = MSD0_send_command(CMD55, 0, 0);			/* should be return 0x01 */
			if(r1!=0x01)
			{
#ifdef PRINT_INFO 
				printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
#endif
				return r1;
			}				

			r1 = MSD0_send_command(ACMD41, 0x40000000, 0);	/* should be return 0x00 */
			if(r1 == 0x00)
			{
				retry = 0;
				break;
			}
		}
 		 	
		/* Timeout return */
		if(retry == 0xFFF)
		{
#ifdef PRINT_INFO
			printf("Send ACMD41 should return 0x00, response=0x%02x\r\n", r1);
#endif
			return 3;
		}

		/* Read OCR by CMD58 */
	    r1 = MSD0_send_command_hold(CMD58, 0, 0);
	    if(r1!=0x00)
	    {
#ifdef PRINT_INFO
			printf("Send CMD58 should return 0x00, response=0x%02x\r\n", r1);
#endif
            return r1;		/* response error, return r1 */
	    }

	    buff[0] = MSD0_spi_read_write(DUMMY_BYTE);					
		buff[1] = MSD0_spi_read_write(DUMMY_BYTE);					
		buff[2] = MSD0_spi_read_write(DUMMY_BYTE);					
		buff[3] = MSD0_spi_read_write(DUMMY_BYTE);					

		/* End of CMD58, chip disable and dummy byte */
		MSD0_card_disable();
		MSD0_spi_read_write(DUMMY_BYTE);
	
	    /* OCR -> CCS(bit30)  1: SDV2HC	 0: SDV2 */
	    if(buff[0] & 0x40)
	    {
           SD0_CardInfo.CardType = CARDTYPE_SDV2HC;
#ifdef PRINT_INFO 
		   printf("Card Type                     : SD V2HC\r\n");
#endif 	
	    }
	    else
	    {
           SD0_CardInfo.CardType = CARDTYPE_SDV2;
#ifdef PRINT_INFO
		   printf("Card Type                     : SD V2\r\n");
#endif 	
	    }

		/* Set spi speed high */
		MSD0_SPIHighSpeed(1);
		}	
   }
   return 0;
}

/*******************************************************************************
* Function Name  : MSD0_GetCardInfo
* Description    : Get SD Card Information
* Input          : None
* Output         : None
* Return         : 0：NO_ERR; TRUE: Error
* Attention		 : None
*******************************************************************************/
int MSD0_GetCardInfo(PMSD_CARDINFO SD0_CardInfo)
{
  uint8_t r1;
  uint8_t CSD_Tab[16];
  uint8_t CID_Tab[16];

  /* Send CMD9, Read CSD */
  r1 = MSD0_send_command(CMD9, 0, 0xFF);
  if(r1 != 0x00)
  {
    return r1;
  }

  if(MSD0_read_buffer(CSD_Tab, 16, RELEASE))
  {
	return 1;
  }

  /* Send CMD10, Read CID */
  r1 = MSD0_send_command(CMD10, 0, 0xFF);
  if(r1 != 0x00)
  {
    return r1;
  }

  if(MSD0_read_buffer(CID_Tab, 16, RELEASE))
  {
	return 2;
  }  

  /* Byte 0 */
  SD0_CardInfo->CSD.CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
  SD0_CardInfo->CSD.SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
  SD0_CardInfo->CSD.Reserved1 = CSD_Tab[0] & 0x03;
  /* Byte 1 */
  SD0_CardInfo->CSD.TAAC = CSD_Tab[1] ;
  /* Byte 2 */
  SD0_CardInfo->CSD.NSAC = CSD_Tab[2];
  /* Byte 3 */
  SD0_CardInfo->CSD.MaxBusClkFrec = CSD_Tab[3];
  /* Byte 4 */
  SD0_CardInfo->CSD.CardComdClasses = CSD_Tab[4] << 4;
  /* Byte 5 */
  SD0_CardInfo->CSD.CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
  SD0_CardInfo->CSD.RdBlockLen = CSD_Tab[5] & 0x0F;
  /* Byte 6 */
  SD0_CardInfo->CSD.PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
  SD0_CardInfo->CSD.WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
  SD0_CardInfo->CSD.RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
  SD0_CardInfo->CSD.DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
  SD0_CardInfo->CSD.Reserved2 = 0; /* Reserved */
  SD0_CardInfo->CSD.DeviceSize = (CSD_Tab[6] & 0x03) << 10;
  /* Byte 7 */
  SD0_CardInfo->CSD.DeviceSize |= (CSD_Tab[7]) << 2;
  /* Byte 8 */
  SD0_CardInfo->CSD.DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
  SD0_CardInfo->CSD.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
  SD0_CardInfo->CSD.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
  /* Byte 9 */
  SD0_CardInfo->CSD.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
  SD0_CardInfo->CSD.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
  SD0_CardInfo->CSD.DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
  /* Byte 10 */
  SD0_CardInfo->CSD.DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
  SD0_CardInfo->CSD.EraseGrSize = (CSD_Tab[10] & 0x7C) >> 2;
  SD0_CardInfo->CSD.EraseGrMul = (CSD_Tab[10] & 0x03) << 3;
  /* Byte 11 */
  SD0_CardInfo->CSD.EraseGrMul |= (CSD_Tab[11] & 0xE0) >> 5;
  SD0_CardInfo->CSD.WrProtectGrSize = (CSD_Tab[11] & 0x1F);
  /* Byte 12 */
  SD0_CardInfo->CSD.WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
  SD0_CardInfo->CSD.ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
  SD0_CardInfo->CSD.WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
  SD0_CardInfo->CSD.MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
  /* Byte 13 */
  SD0_CardInfo->CSD.MaxWrBlockLen |= (CSD_Tab[13] & 0xc0) >> 6;
  SD0_CardInfo->CSD.WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
  SD0_CardInfo->CSD.Reserved3 = 0;
  SD0_CardInfo->CSD.ContentProtectAppli = (CSD_Tab[13] & 0x01);
  /* Byte 14 */
  SD0_CardInfo->CSD.FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
  SD0_CardInfo->CSD.CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
  SD0_CardInfo->CSD.PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
  SD0_CardInfo->CSD.TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
  SD0_CardInfo->CSD.FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
  SD0_CardInfo->CSD.ECC = (CSD_Tab[14] & 0x03);
  /* Byte 15 */
  SD0_CardInfo->CSD.CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
  SD0_CardInfo->CSD.Reserved4 = 1;

  if(SD0_CardInfo->CardType == CARDTYPE_SDV2HC)
  {
	 /* Byte 7 */
	 SD0_CardInfo->CSD.DeviceSize = (u16)(CSD_Tab[8]) *256;
	 /* Byte 8 */
	 SD0_CardInfo->CSD.DeviceSize += CSD_Tab[9] ;
  }

  SD0_CardInfo->Capacity = SD0_CardInfo->CSD.DeviceSize * MSD_BLOCKSIZE * 1024;
  SD0_CardInfo->BlockSize = MSD_BLOCKSIZE;

  /* Byte 0 */
  SD0_CardInfo->CID.ManufacturerID = CID_Tab[0];
  /* Byte 1 */
  SD0_CardInfo->CID.OEM_AppliID = CID_Tab[1] << 8;
  /* Byte 2 */
  SD0_CardInfo->CID.OEM_AppliID |= CID_Tab[2];
  /* Byte 3 */
  SD0_CardInfo->CID.ProdName1 = CID_Tab[3] << 24;
  /* Byte 4 */
  SD0_CardInfo->CID.ProdName1 |= CID_Tab[4] << 16;
  /* Byte 5 */
  SD0_CardInfo->CID.ProdName1 |= CID_Tab[5] << 8;
  /* Byte 6 */
  SD0_CardInfo->CID.ProdName1 |= CID_Tab[6];
  /* Byte 7 */
  SD0_CardInfo->CID.ProdName2 = CID_Tab[7];
  /* Byte 8 */
  SD0_CardInfo->CID.ProdRev = CID_Tab[8];
  /* Byte 9 */
  SD0_CardInfo->CID.ProdSN = CID_Tab[9] << 24;
  /* Byte 10 */
  SD0_CardInfo->CID.ProdSN |= CID_Tab[10] << 16;
  /* Byte 11 */
  SD0_CardInfo->CID.ProdSN |= CID_Tab[11] << 8;
  /* Byte 12 */
  SD0_CardInfo->CID.ProdSN |= CID_Tab[12];
  /* Byte 13 */
  SD0_CardInfo->CID.Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  /* Byte 14 */
  SD0_CardInfo->CID.ManufactDate = (CID_Tab[13] & 0x0F) << 8;
  /* Byte 15 */
  SD0_CardInfo->CID.ManufactDate |= CID_Tab[14];
  /* Byte 16 */
  SD0_CardInfo->CID.CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
  SD0_CardInfo->CID.Reserved2 = 1;

  return 0;  
}

/*******************************************************************************
* Function Name  : MSD0_read_buffer
* Description    : None
* Input          : - *buff:
*				   - len:
*				   - release:
* Output         : None
* Return         : 0：NO_ERR; TRUE: Error
* Attention		 : None
*******************************************************************************/
int MSD0_read_buffer(uint8_t *buff, uint16_t len, uint8_t release)
{
  uint8_t r1;
  register uint16_t retry;

  /* Card enable, Prepare to read	*/
  MSD0_card_enable();

  /* Wait start-token 0xFE */
  for(retry=0; retry<2000; retry++)
  {
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 if(r1 == 0xFE)
	 {
		 retry = 0;
		 break;
	 }
  }

  /* Timeout return	*/
  if(retry == 2000)
  {
	 MSD0_card_disable();
	 return 1;
  }

  /* Start reading */
  for(retry=0; retry<len; retry++)
  {
     *(buff+retry) = MSD0_spi_read_write(DUMMY_BYTE);
  }

  /* 2bytes dummy CRC */
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_spi_read_write(DUMMY_BYTE);

  /* chip disable and dummy byte */ 
  if(release)
  {
	 MSD0_card_disable();
	 MSD0_spi_read_write(DUMMY_BYTE);
  }

  return 0;
}

/*******************************************************************************
* Function Name  : MSD0_ReadSingleBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD0_ReadSingleBlock(uint32_t sector, uint8_t *buffer)
{
  uint8_t r1;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  /* Send CMD17 : Read single block command */
  r1 = MSD0_send_command(CMD17, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }
	
  /* Start read and return the result */
  r1 = MSD0_read_buffer(buffer, MSD_BLOCKSIZE, RELEASE);

  /* Send stop data transmit command - CMD12 */
  MSD0_send_command(CMD12, 0, 0);

  return r1;
}

/*******************************************************************************
* Function Name  : MSD0_ReadMultiBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD0_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector)
{
  uint8_t r1;
  register uint32_t i;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }

  /* Send CMD18 : Read multi block command */
  r1 = MSD0_send_command(CMD18, sector, 0);
  if(r1 != 0x00)
  {
     return 1;
  }

  /* Start read	*/
  for(i=0; i<NbrOfSector; i++)
  {
     if(MSD0_read_buffer(buffer+i*MSD_BLOCKSIZE, MSD_BLOCKSIZE, HOLD))
     {
		 /* Send stop data transmit command - CMD12	*/
		 MSD0_send_command(CMD12, 0, 0);
		 /* chip disable and dummy byte */
		 MSD0_card_disable();
		 return 2;
     }
  }
	
  /* Send stop data transmit command - CMD12 */
  MSD0_send_command(CMD12, 0, 0);

  /* chip disable and dummy byte */
  MSD0_card_disable();
  MSD0_spi_read_write(DUMMY_BYTE);
	
  return 0;
}

/*******************************************************************************
* Function Name  : MSD0_WriteSingleBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD0_WriteSingleBlock(uint32_t sector, uc8 *buffer)
{
  uint8_t r1;
  register uint16_t i;
  uint32_t retry;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  /* Send CMD24 : Write single block command */
  r1 = MSD0_send_command(CMD24, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }

  /* Card enable, Prepare to write */
  MSD0_card_enable();
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_spi_read_write(DUMMY_BYTE);
  /* Start data write token: 0xFE */
  MSD0_spi_read_write(0xFE);
	
  /* Start single block write the data buffer */
  for(i=0; i<MSD_BLOCKSIZE; i++)
  {
    MSD0_spi_read_write(*buffer++);
  }

  /* 2Bytes dummy CRC */
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_spi_read_write(DUMMY_BYTE);
	
  /* MSD card accept the data */
  r1 = MSD0_spi_read_write(DUMMY_BYTE);
  if((r1&0x1F) != 0x05)
  {
    MSD0_card_disable();
    return 2;
  }
	
  /* Wait all the data programm finished */
  retry = 0;
  while(MSD0_spi_read_write(DUMMY_BYTE) == 0x00)
  {	
	 /* Timeout return */
	 if(retry++ == 0x40000)
	 {
	    MSD0_card_disable();
	    return 3;
	 }
  }

  /* chip disable and dummy byte */ 
  MSD0_card_disable();
  MSD0_spi_read_write(DUMMY_BYTE);
	
  return 0;
}

/*******************************************************************************
* Function Name  : MSD0_WriteMultiBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD0_WriteMultiBlock(uint32_t sector, uc8 *buffer, uint32_t NbrOfSector)
{
  uint8_t r1;
  register uint16_t i;
  register uint32_t n;
  uint32_t retry;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD0_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	  sector = sector<<9;
  }

  /* Send command ACMD23 berfore multi write if is not a MMC card */
  if(SD0_CardInfo.CardType != CARDTYPE_MMC)
  {
	  MSD0_send_command(ACMD23, NbrOfSector, 0x00);
  }
	
  /* Send CMD25 : Write nulti block command	*/
  r1 = MSD0_send_command(CMD25, sector, 0);
	
  if(r1 != 0x00)
  {
	  return 1;
  }

  /* Card enable, Prepare to write */
  MSD0_card_enable();
  MSD0_spi_read_write(DUMMY_BYTE);
  //MSD0_spi_read_write(DUMMY_BYTE);
  //MSD0_spi_read_write(DUMMY_BYTE);

  for(n=0; n<NbrOfSector; n++)
  {	
	 /* Start multi block write token: 0xFC */
	 MSD0_spi_read_write(0xFC);

	 for(i=0; i<MSD_BLOCKSIZE; i++)
	 {
		MSD0_spi_read_write(*buffer++);
	 }	

	 /* 2Bytes dummy CRC */
	 MSD0_spi_read_write(DUMMY_BYTE);
	 MSD0_spi_read_write(DUMMY_BYTE);

	 /* MSD card accept the data */
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 if((r1&0x1F) != 0x05)
	 {
	    MSD0_card_disable();
	    return 2;
	 }

	 /* Wait all the data programm finished	*/
	 retry = 0;
	 while(MSD0_spi_read_write(DUMMY_BYTE) != 0xFF)
	 {	
		/* Timeout return */
		if(retry++ == 0x40000)
		{
		   MSD0_card_disable();
		   return 3;
		}
	 }
  }

  /* Send end of transmit token: 0xFD */
  r1 = MSD0_spi_read_write(0xFD);
  if(r1 == 0x00)
  {
	 return 4;
  }

  /* Wait all the data programm finished */
  retry = 0;
  while(MSD0_spi_read_write(DUMMY_BYTE) != 0xFF)
  {	
	 /* Timeout return */
	 if(retry++ == 0x40000)
	 {
	     MSD0_card_disable();
	     return 5;
	 }
  }

  /* chip disable and dummy byte */
  MSD0_card_disable();
  MSD0_spi_read_write(DUMMY_BYTE);

  return 0;
}


/*******************************************************************************
* Function Name  : MSD0_send_command
* Description    : None
* Input          : - cmd:
*				   - arg:
*                  - crc:
* Output         : None
* Return         : R1 value, response from card
* Attention		 : None
*******************************************************************************/
int MSD0_send_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1;
  uint8_t retry;

  /* Dummy byte and chip enable */
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_card_enable();

  /* Command, argument and crc */
  MSD0_spi_read_write(cmd | 0x40);
  MSD0_spi_read_write(arg >> 24);
  MSD0_spi_read_write(arg >> 16);
  MSD0_spi_read_write(arg >> 8);
  MSD0_spi_read_write(arg);
  MSD0_spi_read_write(crc);
  
  /* Wait response, quit till timeout */
  for(retry=0; retry<200; retry++)
  {
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }

  /* Chip disable and dummy byte */ 
  MSD0_card_disable();
  MSD0_spi_read_write(DUMMY_BYTE);

  return r1;
}	

/*******************************************************************************
* Function Name  : MSD0_send_command_hold
* Description    : None
* Input          : - cmd:
*				   - arg:
*                  - crc:
* Output         : None
* Return         : R1 value, response from card
* Attention		 : None
*******************************************************************************/
int MSD0_send_command_hold(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1;
  uint8_t retry;

  /* Dummy byte and chip enable */
  MSD0_spi_read_write(DUMMY_BYTE);
  MSD0_card_enable();

  /* Command, argument and crc */
  MSD0_spi_read_write(cmd | 0x40);
  MSD0_spi_read_write(arg >> 24);
  MSD0_spi_read_write(arg >> 16);
  MSD0_spi_read_write(arg >> 8);
  MSD0_spi_read_write(arg);
  MSD0_spi_read_write(crc);
  
  /* Wait response, quit till timeout */
  for(retry=0; retry<200; retry++)
  {
	 r1 = MSD0_spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }

  return r1;
}

