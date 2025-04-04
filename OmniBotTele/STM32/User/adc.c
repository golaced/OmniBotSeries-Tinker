 #include "adc.h"
 #include "mymath.h"

#define ADC_SAMPLE_NUM	20

u16 adc_value[5*ADC_SAMPLE_NUM];//ADC采集值存放缓冲区
 
int rc_init[2];
//初始化ADC，使用DMA传输
//通道PA0\PA1\PA3\PA4
void Adc_Init(void)
{ 
  u16 test;	
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure; 
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//使ADC1时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//使能DMA时钟
	
	//PA0\1\2 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//DMA 配置
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;	 //ADC1->DR地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&adc_value;//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 5*ADC_SAMPLE_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址增加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;				//扫描模式，用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 5;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	ADC_DMACmd(ADC1, ENABLE);//使能ADC1 DMA
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	//配置连续转换通道，55.5个采样周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);	//1个通道转换一次耗时21us 4个通道
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);	//采样个数ADC_SAMPLE_NUM
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_71Cycles5);	//总共耗时4*21*ADC_SAMPLE_NUM（64）=5.4ms<10ms
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_71Cycles5);	//总共耗时4*21*ADC_SAMPLE_NUM（64）=5.4ms<10ms
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_71Cycles5);	//总共耗时4*21*ADC_SAMPLE_NUM（64）=5.4ms<10ms

	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}				 

 

 u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


//ADC均值滤波
void ADC_Filter(u16* adc_val)
{
	u16 i=0;
	u32 sum[5]={0,0,0,0};
	
	for(;i<ADC_SAMPLE_NUM;i++)
	{
		sum[0]+=adc_value[5*i+0];
		sum[1]+=adc_value[5*i+1];
		sum[2]+=adc_value[5*i+2];
	}
	adc_val[0]=sum[0]/ADC_SAMPLE_NUM;
	adc_val[1]=sum[1]/ADC_SAMPLE_NUM;
	adc_val[2]=sum[2]/ADC_SAMPLE_NUM;
}


u16 getAdcValue1(uint8_t axis)
{ uint8_t i=0;
	u32 sum=0;
	for( i=0;i<ADC_SAMPLE_NUM;i++)
	{
		sum += Get_Adc(axis);
	}
	return sum/ADC_SAMPLE_NUM;
}

u16 getAdcValue(uint8_t axis)
{ uint8_t i=0;
	u32 sum=0;
	for( i=0;i<ADC_SAMPLE_NUM;i++)
	{
		sum += adc_value[5*i+axis];
	}
	return sum/ADC_SAMPLE_NUM;
}

joystickFlyui16_t adc_rc[2];
/*获取摇杆ADC值*/

float k_bat=0.00102;
float k_t=-1;
void getFlyDataADCValue(void)
{
	adc_rc[0].x = -(LIMIT(getAdcValue(1),0,4096))/4096.*1000+1500;
	adc_rc[0].y = (LIMIT(getAdcValue(2),0,4096))/4096.*1000+500;
	adc_rc[1].y = (LIMIT(getAdcValue(3),0,4096))/4096.*1000+500;
	adc_rc[1].x = (LIMIT(getAdcValue(4),0,4096))/4096.*1000+500;
	
	adc_rc[0].bat = getAdcValue(0)*k_bat;
	adc_rc[0].bat_percent=LIMIT((adc_rc[0].bat-1*3.65)/(4.2-1*3.65),0,0.99)*100;	
}











