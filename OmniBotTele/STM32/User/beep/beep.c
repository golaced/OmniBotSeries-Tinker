#include   "beep.h"
#include   "time.h"
#include   "head.h"
#include   "rc_mine.h"
#include   "delay.h"
u8 start_music[]={
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 17,21 , 21,21,
// 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 17,21, 21,21,
};

u8 mems_right_music[]={
 05,34, 06,34, 07,34, 
};

u8 mems_error_music[]={
 07,36, 00,32, 07,36, 00,32, 
};

u8 rc_reset_music[]={
 07,12, 00,12, 07,12, 00,12, 
};

u8 bat_error_music[]={
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32, 	
};

u8 rc_error_music[]={
 27,11, 00,11,   27,11, 00,11,  27,11, 00,11, 00,39 , 00,39
};


u8 upload_music[]={
 37,11, 13,11,   17,21, 13,11,  37,11
};

u8 start_music_pix[]={
 04,32, 24,22, 05,32, 
 04,32, 24,22, 05,32,
 04,32, 24,22, 05,32,
};
u8 test_to24;
u16 Beat_delay[7]={0,62,94,125,125,187,250};
void Play_Music(u8 *music, u16 st,u16 ed)
{ u8 loop,beat,level,tone;
	u16 i;
  for(i=st;i<ed-st;i++)
   {
		 level=music[i*2]/10;
		 tone=music[i*2]%10;
		 Tone(level,tone);
		 loop=music[i*2+1]%10;
		 beat=music[i*2+1]/10;
     delay_ms(Beat_delay[beat]*loop);	 
	 }	
   Tone(0,0);		 
}

u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt)
{ static u8 state=0;
	static u16 i,cnt;
	u8 loop,beat,level,tone;
	switch(state)
	{
		case 0:
	     if(en)
	     {state=1;i=st;cnt=0;}
	  break;
	  case 1:
			 if(en) 
			 {
				level=music[i*2]/10;
				tone=music[i*2]%10;
				loop=music[i*2+1]%10;
				beat=music[i*2+1]/10;
				 
				 if(cnt++>Beat_delay[beat]*loop/1000./dt){Tone(level,tone);i++;cnt=0;
				 				} 
			 }
	     else
			 {Tone(0,0);state=0;  }
			 if(i>ed)
			 {state=0;Tone(0,0);	}
		break;
	} 
}

void Play_Music_Direct(u8 sel)
{

  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music(mems_right_music,0,sizeof(mems_right_music)/2);	
	  break;
		case START_BEEP:
		Play_Music(start_music,0,sizeof(start_music)/2);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music(mems_error_music,0,sizeof(mems_error_music)/2);		
		break;
		case UPLOAD_BEEP:
		Play_Music(upload_music,0,sizeof(upload_music)/2);		
		break;
		case RC_RESET_BEEP:
		Play_Music(rc_reset_music,0,sizeof(rc_reset_music)/2);		
		break;
	}

}

void Play_Music_Task(u8 sel,float dt)//<-----------------
{

  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music_In_Task(mems_right_music,0,sizeof(mems_right_music)/2,1,dt);	
	  break;
		case START_BEEP:
		Play_Music_In_Task(start_music,0,sizeof(start_music)/2,1,dt);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music_In_Task(mems_error_music,0,sizeof(mems_error_music)/2,1,dt);		
		break;
		case BAT_ERO_BEEP:
		Play_Music_In_Task(bat_error_music,0,sizeof(bat_error_music)/2,1,dt);		
		break;
		case RC_ERO_BEEP:
			 
		break;
		default:
			Tone(0,0);
		break;
	}

}
float k_beep=2;
u32 tone_init=1000000/2;
void Beep_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	u8 i,j;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //使能GPIO外设时钟使能
	                                                                     	

   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/k_beep;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM14
  Tone(0,0);
		
// 	for(i=0;i<3;i++)
// 	 for(j=1;j<8;j++)
// 	 {Tone(i,j);delay_ms(111);}
// 	 Tone(0,0);
}  

u16 tone_table[3][8]={
     {0,261,293,329,349,391,440,493},
     {0,523,587,659,698,783,880,987},
     {0,1046,1174,1318,1396,1567,1760,1975}};
void Tone(u8 level, u8 tone)
{ u32 psc=72-1;
  u32 arr=tone_init/tone_table[level][tone]-1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  if(tone==0)
		arr=tone_init/1-1;

	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/k_beep;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM14
}
