/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <stdio.h>
#include "head.h"
#include "usart.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
extern void USART1_IRQ_TASK(u8 res);
extern void USART2_IRQ_TASK(	u8 res);//FLY
extern void USART3_IRQ_TASK(void);
void USART1_IRQHandler(void)
{
	uint8_t ch;
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
	{
						USART_ClearFlag(USART1,USART_FLAG_ORE); //读SR其实就是清除标志
						USART_ReceiveData(USART1);    //读DR
	 }
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
			ch = USART_ReceiveData(USART1);
		if(uart_test[1]){
						USART_SendData(USART2, (u8) ch);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;}
		 else
	
			USART_ClearITPendingBit(USART1,  USART_IT_RXNE);  
		USART_ClearFlag(USART1,USART_FLAG_TC);  //清除标志位
	} 
}

void USART2_IRQHandler(void)
{
	uint8_t ch;
		if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)==SET)
	{
						USART_ClearFlag(USART2,USART_FLAG_ORE); //读SR其实就是清除标志
						USART_ReceiveData(USART2);    //读DR
	 }
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
			ch = USART_ReceiveData(USART2);
		if(uart_test[2]){
					USART_SendData(USART2, (u8) ch);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;}
		else
	
		USART_ClearITPendingBit(USART2,  USART_IT_RXNE);  
		USART_ClearFlag(USART2,USART_FLAG_TC);  //清除标志位	

	}  
	
}
void USART3_IRQHandler(void)
{
	uint8_t ch;
		if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)==SET)
	{
						USART_ClearFlag(USART3,USART_FLAG_ORE); //读SR其实就是清除标志
						USART_ReceiveData(USART3);    //读DR
	 }
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
			ch = USART_ReceiveData(USART3);

		if(uart_test[3]){
					USART_SendData(USART2, (u8) ch);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;}
		else
		USART3_IRQ_TASK();
			USART_ClearITPendingBit(USART3,  USART_IT_RXNE);  
		USART_ClearFlag(USART3,USART_FLAG_TC);  //清除标志位
	}  
	
}

//串口1DMA方式发送中断  
void DMA1_Channel7_IRQHandler(void)  
{  u8 temp=0;

	DMA_ClearFlag(DMA1_FLAG_TC7);
    DMA_Cmd(DMA1_Channel7,DISABLE);  
	DMA1_Channel7->CNDTR = SENDBUFF_SIZE;//重装填
   	dma_can_tx=1;

}  

extern void Time4_IntHandle(void);
void TIM4_IRQHandler(void)  
{if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{		
  Time4_IntHandle();
	}
}  

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
