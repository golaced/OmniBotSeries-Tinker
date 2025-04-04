/* delay.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
提供精确的延时API  有微秒级 和毫秒级延时
------------------------------------
 */
#include "delay.h"
#include "time.h" 
static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数

//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
/**************************实现函数********************************************
*函数原型:		void delay_init(u8 SYSCLK)
*功　　能:		初始化延迟系统，使延时程序进入可用状态
*******************************************************************************/
void delay_init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}				
				    


void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//开启定时器外设时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		//配置定时器参数
		TIM_DeInit(TIM3); 
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 								 	//1ms定时			 
		TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);              
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
//		//中断配置
//		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //抢占优先级2 低优先级别中断 
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	 //响应优先级0 高级别的响应中断
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//		NVIC_Init(&NVIC_InitStructure);	  
//		//开中断
//		TIM_ClearFlag(TIM3, TIM_FLAG_Update);					  
//		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
		//开启定时器			 
		TIM_Cmd(TIM3, ENABLE); 
}

#define GET_TIME_NUM 10

volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

u32 Get_Cycle_T(u8 item)	
{
	
	Cycle_T[item][NOW] = micros(); //?????
	if(Cycle_T[item][NOW]>Cycle_T[item][OLD])
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//?????(??)
	else
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] +(0xFFFF- Cycle_T[item][OLD] )) );//?????(??)	
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//??????
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}
}



void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
}   

//延时nus
//nus为要延时的us数.
/**************************实现函数********************************************
*函数原型:		void delay_us(u32 nus)
*功　　能:		微秒级延时  延时nus  nms<=1864 
*******************************************************************************/		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}

//------------------End of File----------------------------
