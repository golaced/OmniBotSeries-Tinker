/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               diskio.c
** Descriptions:            The FATFS Diskio
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-4
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

/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "ffconf.h"
#include "SPI_MSD0_Driver.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
extern MSD_CARDINFO SD0_CardInfo;


DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	int Status;
	switch (drv) 
	{
		case 0 :
			Status = MSD0_Init();
			if(Status==0){
				return RES_OK;
			}else{
				return STA_NOINIT;
			}
		case 1 :
			return RES_OK;	  
		case 2 :
			return RES_OK;	  
		case 3 :
			return RES_OK;
		default:
			return STA_NOINIT;
	}
}




DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
	switch (drv)
	{
		case 0 :
			return RES_OK;
		case 1 :
			return RES_OK;
		case 2 :
			return RES_OK;
		default:
			return STA_NOINIT;
	}
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
	int Status;
	if( !count )
	{    
		return RES_PARERR;  /* count不能等于0，否则返回参数错误 */
	}
	switch (drv)
	{
		case 0:
		    if(count==1)            /* 1个sector的读操作 */      
		    {   
				Status =  MSD0_ReadSingleBlock( sector ,buff );
				if(Status == 0){
					return RES_OK;
				}else{
					return RES_ERROR;
				}    
		    }                                                
		    else                    /* 多个sector的读操作 */     
		    {  
				Status = MSD0_ReadMultiBlock( sector , buff ,count);
				if(Status == 0){
					return RES_OK;
				}else{
					return RES_ERROR;
				} 
		    }                                                
		case 1:
		    if(count==1)            /* 1个sector的读操作 */      
		    {   
				return RES_OK;    
		    }                                                
		    else                    /* 多个sector的读操作 */     
		    {  
				return RES_OK; 
		    } 

		default:
			return RES_ERROR;
	}
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	        /* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	int Status;
	if( !count )
	{    
		return RES_PARERR;  /* count不能等于0，否则返回参数错误 */
	}
	switch (drv)
	{
		case 0:
		    if(count==1)            /* 1个sector的写操作 */      
		    {   
				Status = MSD0_WriteSingleBlock( sector , (uint8_t *)(&buff[0]) ); 
				if(Status == 0){
					return RES_OK;
				}else{
					return RES_ERROR;
				} 
		    }                                                
		    else                    /* 多个sector的写操作 */    
		    {  
				Status = MSD0_WriteMultiBlock( sector , (uint8_t *)(&buff[0]) , count );
				if(Status == 0){
					return RES_OK;
				}else{
					return RES_ERROR;
				}   
		    }                                                
		case 1:
		    if(count==1)            /* 1个sector的写操作 */      
		    {  
				return RES_OK; 
		    }                                                
		    else                    /* 多个sector的写操作 */    
		    {  
				return RES_OK; 
		    }                                                
 
		default:return RES_ERROR;
	}
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	if (drv==0)
	{    
		MSD0_GetCardInfo(&SD0_CardInfo);
		switch (ctrl) 
		{
			case CTRL_SYNC : 
				return RES_OK;
		  	case GET_SECTOR_COUNT : 
				*(DWORD*)buff = SD0_CardInfo.Capacity/SD0_CardInfo.BlockSize;
		    	return RES_OK;
		  	case GET_BLOCK_SIZE :
				*(WORD*)buff = SD0_CardInfo.BlockSize;
		    	return RES_OK;	
		  	case CTRL_POWER :
				break;
		  	case CTRL_LOCK :
				break;
		  	case CTRL_EJECT :
				break;
	      	/* MMC/SDC command */
		  	case MMC_GET_TYPE :
				break;
		  	case MMC_GET_CSD :
				break;
		  	case MMC_GET_CID :
				break;
			case MMC_GET_OCR :
				break;
			case MMC_GET_SDSTAT :
				break;	
		} 
    }else if(drv==1){
		switch (ctrl) 
		{
			case CTRL_SYNC : 
				return RES_OK;
		  	case GET_SECTOR_COUNT : 
		    	return RES_OK;
		  	case GET_SECTOR_SIZE :
				return RES_OK;
		  	case GET_BLOCK_SIZE :
		    	return RES_OK;	
		  	case CTRL_POWER :
				break;
		  	case CTRL_LOCK :
				break;
		  	case CTRL_EJECT :
				break;
	      	/* MMC/SDC command */
		  	case MMC_GET_TYPE :
				break;
		  	case MMC_GET_CSD :
				break;
		  	case MMC_GET_CID :
				break;
		  	case MMC_GET_OCR :
				break;
		  	case MMC_GET_SDSTAT :
				break;	
		} 	
	}
	else{				  
		return RES_PARERR;  
	}
	return RES_PARERR; 
}

/* 得到文件Calendar格式的建立日期,是DWORD get_fattime (void) 逆变换 */							
/*-----------------------------------------------------------------------*/
/* User defined function to give a current time to fatfs module          */
/* 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
/* 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{
   
    return 0;
}



/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
