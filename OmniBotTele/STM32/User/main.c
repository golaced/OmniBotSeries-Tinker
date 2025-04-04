#include "head.h"
#include "table.h"
#include "spi.h"
#include "nrf.h"
#include "adc.h"
#include "key.h"
#include "mpu9250.h"
#include "FLASH.h"
#include "imu.h"
#include "mymath.h"
#include "math.h"
#include "rc_mine.h"
#include "beep.h"
#include "gui_basic.h"
#include "hw_config.h"
#include "data_transfer.h"
#define USE_AS_MONITER 1
MODULE module;
u8 CHANNAL=44;
float time_fly[5]={0,0,0,0,2.68};
int moniter_sel=0;
int sub_sel[3]={0};
int ts[4];

float dt[10]={0},off_att[2];
u16 key_sel_down=0;
u8 rst_screen=0;
int main(void)
{	u8 i,j;
	static u16 cnt_flag[10];
	char temp[10][20]={'\0'};
  static u8 moniter_reg,fly_reg;
  static int cnt_get_data;
	u8 pid_sel[2],page,sub_page;

//------------------------------------//
	delay_init(72);		//—” ±≥ı ºªØ
	TIM3_Config();
	Time4ON();
	Cycle_Time_Init();
	Adc_Init();
	usb_vcp_init();
	delay_ms(1000);
	keyInit(); 
	LED_Init();
	USART_init();	
	__enable_irq();
	
	while(1){
		 if(flag_ms[2]==1)
		 {
      flag_ms[2]=0;
			dt[0] = Get_Cycle_T(0)/1000000.0f;	
			getFlyDataADCValue();
		 } 
		 
     if(flag_ms[5]==1)
		 {
      flag_ms[5]=0;
			dt[1] = Get_Cycle_T(1)/1000000.0f;	  
 
		 } 
  
		 if(flag_ms[10]==1)
		 {
				dt[2] = Get_Cycle_T(2)/1000000.0f;	

				flag_ms[10]=0;	
				Send_To_Wio();  
		 }
		 
		  if(flag_ms[20]==1)
		 {
			dt[3] = Get_Cycle_T(3)/1000000.0f;	   
      flag_ms[20]=0;
		 }
		 
		 if(flag_ms[25]==1)
		 {
			dt[4] = Get_Cycle_T(4)/1000000.0f;	   
      flag_ms[25]=0;
			KEY_Scan(0.05);	
		 }

		 if(flag_ms[50]==1)
		 {
				dt[5] = Get_Cycle_T(5)/1000000.0f;	   
				flag_ms[50]=0;

		 }
		 
	}
}
/*********************************************END OF FILE**********************/

