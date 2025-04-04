#ifndef _LD3320_CONFIG_H__
#define _LD3320_CONFIG_H__

///识别码（客户修改处）
#define CODE_LSD	1	 //流水灯
#define CODE_SS	  2	 //闪烁
#define CODE_AJCF	3	 //按键触发
#define CODE_QM	  4	 //全灭
#define CODE_JT		5  //状态

///LD3320引脚相关定义
#define LD3320RST_PIN					GPIO_Pin_1
#define LD3320RST_GPIO_PORT		GPIOA
#define LD3320RST_GPIO_CLK		RCC_APB2Periph_GPIOA
#define LD_RST_H() 						GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define LD_RST_L() 						GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define LD3320CS_PIN					GPIO_Pin_4		
#define LD3320CS_GPIO_PORT		GPIOA
#define LD3320CS_GPIO_CLK			RCC_APB2Periph_GPIOA
#define LD_CS_H()							GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define LD_CS_L()							GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define LD3320IRQ_GPIO_CLK		RCC_APB2Periph_GPIOB
#define LD3320IRQ_PIN					GPIO_Pin_1
#define LD3320IRQ_GPIO_PORT		GPIOB
#define LD3320IRQEXIT_PORTSOURCE		GPIO_PortSourceGPIOB
#define LD3320IRQPINSOURCE		GPIO_PinSource1
#define LD3320IRQEXITLINE			EXTI_Line1
#define LD3320IRQN						EXTI1_IRQn


#define LD3320WR_PIN					GPIO_Pin_13
#define LD3320WR_GPIO_PORT		GPIOC
#define LD3320WR_GPIO_CLK			RCC_APB2Periph_GPIOC
#define LD_SPIS_H()  					GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LD_SPIS_L()  					GPIO_ResetBits(GPIOC, GPIO_Pin_13)

#define	LD3320SPI							SPI1
#define LD3320SPI_CLK					RCC_APB2Periph_SPI1						

#define LD3320SPIMISO_PIN					GPIO_Pin_6
#define LD3320SPIMISO_GPIO_PORT		GPIOA
#define LD3320SPIMISO_GPIO_CLK		RCC_APB2Periph_GPIOA

#define LD3320SPIMOSI_PIN					GPIO_Pin_7
#define LD3320SPIMOSI_GPIO_PORT		GPIOA
#define LD3320SPIMOSI_GPIO_CLK		RCC_APB2Periph_GPIOA

#define LD3320SPISCK_PIN					GPIO_Pin_5
#define LD3320SPISCK_GPIO_PORT		GPIOA
#define LD3320SPISCK_GPIO_CLK			RCC_APB2Periph_GPIOA

#endif
