/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               SPI_MSD_Driver.h
** Descriptions:            The SPI SD Card application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Ya Dan
** Created date:            2011-1-4
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef __SPI_MSD0_DRIVER_H
#define __SPI_MSD0_DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "SPI_MSD1_Driver.h"
//
///* Private define ------------------------------------------------------------*/
//#define CARDTYPE_MMC     	     0x00
//#define CARDTYPE_SDV1      	     0x01
//#define CARDTYPE_SDV2      	     0x02
//#define CARDTYPE_SDV2HC    	     0x04
//
//#define DUMMY_BYTE				 0xFF 
//#define MSD_BLOCKSIZE			 512
//
///* SD/MMC command list - SPI mode */
//#define CMD0                     0       /* Reset */
//#define CMD1                     1       /* Send Operator Condition - SEND_OP_COND */
//#define CMD8                     8       /* Send Interface Condition - SEND_IF_COND	*/
//#define CMD9                     9       /* Read CSD */
//#define CMD10                    10      /* Read CID */
//#define CMD12                    12      /* Stop data transmit */
//#define CMD16                    16      /* Set block size, should return 0x00 */ 
//#define CMD17                    17      /* Read single block */
//#define CMD18                    18      /* Read multi block */
//#define ACMD23                   23      /* Prepare erase N-blokcs before multi block write */
//#define CMD24                    24      /* Write single block */
//#define CMD25                    25      /* Write multi block */
//#define ACMD41                   41      /* should return 0x00 */
//#define CMD55                    55      /* should return 0x01 */
//#define CMD58                    58      /* Read OCR */
//#define CMD59                    59      /* CRC disable/enbale, should return 0x00 */
//
///* Physical level marcos */
#define MSD0_card_enable()      	GPIOB->BRR = GPIO_Pin_12//GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define MSD0_card_disable()     	GPIOB->BSRR = GPIO_Pin_12//GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define MSD0_card_power_on()
#define MSD0_card_insert()       	GPIOB->IDR & GPIO_Pin_0//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
//
//
///* Private typedef -----------------------------------------------------------*/
//enum _CD_HOLD
//{
//	HOLD = 0,
//	RELEASE = 1,
//};
//
//typedef struct               /* Card Specific Data */
//{
//  vu8  CSDStruct;            /* CSD structure */
//  vu8  SysSpecVersion;       /* System specification version */
//  vu8  Reserved1;            /* Reserved */
//  vu8  TAAC;                 /* Data read access-time 1 */
//  vu8  NSAC;                 /* Data read access-time 2 in CLK cycles */
//  vu8  MaxBusClkFrec;        /* Max. bus clock frequency */
//  vu16 CardComdClasses;      /* Card command classes */
//  vu8  RdBlockLen;           /* Max. read data block length */
//  vu8  PartBlockRead;        /* Partial blocks for read allowed */
//  vu8  WrBlockMisalign;      /* Write block misalignment */
//  vu8  RdBlockMisalign;      /* Read block misalignment */
//  vu8  DSRImpl;              /* DSR implemented */
//  vu8  Reserved2;            /* Reserved */
//  vu32 DeviceSize;           /* Device Size */
//  vu8  MaxRdCurrentVDDMin;   /* Max. read current @ VDD min */
//  vu8  MaxRdCurrentVDDMax;   /* Max. read current @ VDD max */
//  vu8  MaxWrCurrentVDDMin;   /* Max. write current @ VDD min */
//  vu8  MaxWrCurrentVDDMax;   /* Max. write current @ VDD max */
//  vu8  DeviceSizeMul;        /* Device size multiplier */
//  vu8  EraseGrSize;          /* Erase group size */
//  vu8  EraseGrMul;           /* Erase group size multiplier */
//  vu8  WrProtectGrSize;      /* Write protect group size */
//  vu8  WrProtectGrEnable;    /* Write protect group enable */
//  vu8  ManDeflECC;           /* Manufacturer default ECC */
//  vu8  WrSpeedFact;          /* Write speed factor */
//  vu8  MaxWrBlockLen;        /* Max. write data block length */
//  vu8  WriteBlockPaPartial;  /* Partial blocks for write allowed */
//  vu8  Reserved3;            /* Reserded */
//  vu8  ContentProtectAppli;  /* Content protection application */
//  vu8  FileFormatGrouop;     /* File format group */
//  vu8  CopyFlag;             /* Copy flag (OTP) */
//  vu8  PermWrProtect;        /* Permanent write protection */
//  vu8  TempWrProtect;        /* Temporary write protection */
//  vu8  FileFormat;           /* File Format */
//  vu8  ECC;                  /* ECC code */
//  vu8  CSD_CRC;              /* CSD CRC */
//  vu8  Reserved4;            /* always 1*/
//}
//MSD_CSD;
//
//typedef struct				 /*Card Identification Data*/
//{
//  vu8  ManufacturerID;       /* ManufacturerID */
//  vu16 OEM_AppliID;          /* OEM/Application ID */
//  vu32 ProdName1;            /* Product Name part1 */
//  vu8  ProdName2;            /* Product Name part2*/
//  vu8  ProdRev;              /* Product Revision */
//  vu32 ProdSN;               /* Product Serial Number */
//  vu8  Reserved1;            /* Reserved1 */
//  vu16 ManufactDate;         /* Manufacturing Date */
//  vu8  CID_CRC;              /* CID CRC */
//  vu8  Reserved2;            /* always 1 */
//}
//MSD_CID;
//
//typedef struct
//{
//  MSD_CSD CSD;
//  MSD_CID CID;
//  u32 Capacity;              /* Card Capacity */
//  u32 BlockSize;             /* Card Block Size */
//  u16 RCA;
//  u8 CardType;
//  u32 SpaceTotal;            /* Total space size in file system */
//  u32 SpaceFree;      	     /* Free space size in file system */
//}
//MSD_CARDINFO, *PMSD_CARDINFO;


/* Private function prototypes -----------------------------------------------*/

int MSD0_Init(void);
int MSD0_GetCardInfo(PMSD_CARDINFO cardinfo);
int MSD0_ReadSingleBlock(uint32_t sector, uint8_t *buffer);
int MSD0_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector);
int MSD0_WriteSingleBlock(uint32_t sector, uc8 *buffer);
int MSD0_WriteMultiBlock(uint32_t sector, uc8 *buffer, uint32_t NbrOfSector);

void MSD0_SPI_Configuration(void);
void MSD0_SPIHighSpeed(uint8_t b_high);

__inline int MSD0_spi_read_write(uint8_t data);
int MSD0_send_command(uint8_t cmd, uint32_t arg, uint8_t crc);
int MSD0_send_command_hold(uint8_t cmd, uint32_t arg, uint8_t crc);
int MSD0_read_buffer(uint8_t *buff, uint16_t len, uint8_t release);

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

