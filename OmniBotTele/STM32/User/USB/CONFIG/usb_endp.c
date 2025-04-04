/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
	u16 USB_Tx_ptr;
	u16 USB_Tx_length; 
	if(uu_txfifo.readptr==uu_txfifo.writeptr)		//无任何数据要发送,直接退出
	{
		return;
	}
	if(uu_txfifo.readptr<uu_txfifo.writeptr)		//没有超过数组,读指针<写指针
	{
		USB_Tx_length=uu_txfifo.writeptr-uu_txfifo.readptr;//得到要发送的数据长度
	}
	else											//超过数组了 读指针>写指针
	{
		USB_Tx_length=USB_USART_TXFIFO_SIZE-uu_txfifo.readptr;//得到要发送的数据长度 
	} 
	if(USB_Tx_length>VIRTUAL_COM_PORT_DATA_SIZE)	//超过64字节?
	{
		USB_Tx_length=VIRTUAL_COM_PORT_DATA_SIZE;	//此次发送数据量
	}
	USB_Tx_ptr=uu_txfifo.readptr;					//发送起始地址		
	uu_txfifo.readptr+=USB_Tx_length;				//读指针偏移  
	if(uu_txfifo.readptr>=USB_USART_TXFIFO_SIZE)	//读指针归零
	{
		uu_txfifo.readptr=0;
	} 
	UserToPMABufferCopy(&uu_txfifo.buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
	SetEPTxCount(ENDP1, USB_Tx_length);
	SetEPTxValid(ENDP1);   
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	u16 USB_Rx_Cnt; 
	USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);	//得到USB接收到的数据及其长度  
	USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);	//处理数据（其实就是保存数据） 
	
	SetEPRxValid(ENDP3);								//时能端点3的数据接收
}


/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
	static uint32_t FrameCount = 0;

	if(bDeviceState == CONFIGURED)
	{
		if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
		{
			/* Reset the frame counter */
			FrameCount = 0;

			/* Check the data to be sent through IN pipe */
			EP1_IN_Callback();//通过EP1_IN_Callback函数实现TX数据发送给USB
			//Handle_USBAsynchXfer();
		}
	}  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

