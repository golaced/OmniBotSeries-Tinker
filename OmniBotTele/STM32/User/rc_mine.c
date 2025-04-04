/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * ???  :RC.c
 * ??    :???????         
 * ????:Air Nano?????
 * ???  :ST3.5.0
 * ??    :Air Nano Team 
 * ??    :http://byd2.taobao.com
**********************************************************************************/
#include "rc_mine.h"
#include "nrf.h"
#include "key.h"
#include "data_transfer.h"
#define RX_DR			6		//????
#define TX_DS			5
#define MAX_RT		4
u8 send_pid=0;
int DEBUG[35];
int CH[5];
u8 key_rc[6]={1,1,1,1,1,1};
u16 Yaw_sb_rc=0;

u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;

 PID_STA HPIDt,SPIDt;	
 u16 ax,ay,az,gx,gy,gz,hx,hy,hz,YM,PWM1,PWM2,PWM3,PWM4,fix_pit,fix_rol;

u16 bat_fly=840,high_f;
float ypr[3];
double GPS_W,GPS_J;
u8 data_rate,fly_mode;
int rc_rate_cnt[10];
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
struct RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 height_mode=0,pos_mode=0;


void NRF_DataAnl(void)
{ 
u8 temp_key[7];
u8 temp,flag;
double zen,xiao;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))		
		return;		//??sum
	
	if(NRF24L01_RXDATA[0]==0x1)//send1								
	{ plane.rc_lost_cnt++;
		plane.att[0]= (float)((vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2])/10.;
		plane.att[1]= (float)((vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4])/10.;
		plane.att[2]= (float)((vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6])/10.;	
		plane.spd[0]= (float)((vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8])/100.;
		plane.spd[1]= (float)((vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10])/100.;
		plane.spd[2]= (float)((vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12])/100.;
		plane.pos[0]= (float)((vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14])/100.;
		plane.pos[1]= (float)((vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16])/100.;
		plane.pos[2]= (float)((vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18])/100.;
		plane.gps_sv=NRF24L01_RXDATA[19];
		plane.lock= NRF24L01_RXDATA[20];
		plane.mode= NRF24L01_RXDATA[21];
		plane.state_v= NRF24L01_RXDATA[22];
		plane.acc3d_step= NRF24L01_RXDATA[23];
		plane.bat=(vs16)(NRF24L01_RXDATA[24]<<8)|NRF24L01_RXDATA[25];
		plane.pos_sensor_state=NRF24L01_RXDATA[26];
		plane.module.gps=NRF24L01_RXDATA[27]>>7&0x0000001;
		plane.module.vision=NRF24L01_RXDATA[27]>>6&0x0000001;
		plane.module.flow=NRF24L01_RXDATA[27]>>5&0x0000001;
		plane.module.bmp=NRF24L01_RXDATA[27]>>4&0x0000001;
		plane.module.sonar=NRF24L01_RXDATA[27]>>3&0x0000001;
  }
	else if(NRF24L01_RXDATA[0]==0x02)//send2								
	{ 
		DEBUG[0]= (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		DEBUG[1]= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		DEBUG[2]= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		DEBUG[3]= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		DEBUG[4]= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		DEBUG[5]= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		DEBUG[6]= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		DEBUG[7]= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		DEBUG[8]= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		CH[0]= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		CH[1]= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		CH[2]= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		CH[3]= (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		CH[4]= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
  }
	else if(NRF24L01_RXDATA[0]==0x07)//send1								
	{ 
		plane.mission.t_att[0]= (float)((vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2])/10.;
		plane.mission.t_att[1]= (float)((vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4])/10.;
		plane.mission.t_att[2]= (float)((vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6])/10.;	
		plane.mission.t_spd[0]= (float)((vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8])/100.;
		plane.mission.t_spd[1]= (float)((vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10])/100.;
		plane.mission.t_spd[2]= (float)((vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12])/100.;
		plane.mission.t_pos[0]= (float)((vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14])/100.;
		plane.mission.t_pos[1]= (float)((vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16])/100.;
		plane.mission.t_pos[2]= (float)((vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18])/100.;
    plane.mission.mains= NRF24L01_RXDATA[19];
		plane.mission.subs= NRF24L01_RXDATA[20];
		plane.mission.smarts= NRF24L01_RXDATA[21];
		plane.mission.wayps= NRF24L01_RXDATA[22];
  }
	else if(NRF24L01_RXDATA[0]==0x08)//send1								
	{ 
		zen=(int16_t)((NRF24L01_RXDATA[1]<<8|NRF24L01_RXDATA[2]));
		xiao=(double)((u32)(NRF24L01_RXDATA[3]<<24)|(NRF24L01_RXDATA[4]<<16)|(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6])/1000000000.;
		plane.lon_d=zen+xiao;
		zen=(int16_t)((NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8]);
		xiao=(double)((u32)(NRF24L01_RXDATA[9]<<24)|(NRF24L01_RXDATA[10]<<16)|(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12])/1000000000.;
		plane.lat_d=zen+xiao;
		flag=NRF24L01_RXDATA[13];
		if((flag>>1&0x00000001)==0)
			plane.lon_d*=-1;
		if((flag&0x00000001)==0)
			plane.lat_d*=-1;
		
  }
		else if(NRF24L01_RXDATA[0]==0x03)//send3								
	{ 

		
		plane.PID_RX[0][0]= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		plane.PID_RX[0][1]= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		plane.PID_RX[0][2]= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		
		plane.PID_RX[1][0]= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		plane.PID_RX[1][1]= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		plane.PID_RX[1][2]= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		plane.PID_RX[2][0]= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		plane.PID_RX[2][1]= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		plane.PID_RX[2][2]= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];

		plane.PID_RX[3][0]= (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		plane.PID_RX[3][1]= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		plane.PID_RX[3][2]= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
  }
	else if(NRF24L01_RXDATA[0]==0x04)//send4								
	{ 
		plane.PID_RX[4][0]= (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		plane.PID_RX[4][1]= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		plane.PID_RX[4][2]= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		plane.PID_RX[5][0]= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		plane.PID_RX[5][1]= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		plane.PID_RX[5][2]= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		plane.PID_RX[6][0]= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		plane.PID_RX[6][1]= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		plane.PID_RX[6][2]= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		plane.PID_RX[7][0]= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		plane.PID_RX[7][1]= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		plane.PID_RX[7][2]= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		plane.PID_RX[8][0]= (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		plane.PID_RX[8][1]= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		plane.PID_RX[8][2]= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
		
  }
		else if(NRF24L01_RXDATA[0]==0x05)//send5								
	{ 
		plane.PID_RX[9][0]= (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		plane.PID_RX[9][1]= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		plane.PID_RX[9][2]= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		plane.PID_RX[10][0]= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		plane.PID_RX[10][1]= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		plane.PID_RX[10][2]= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		plane.PID_RX[11][0]= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		plane.PID_RX[11][1]= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		plane.PID_RX[11][2]= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		plane.PID_RX[12][0]= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		plane.PID_RX[12][1]= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		plane.PID_RX[12][2]= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
  }
	else if(NRF24L01_RXDATA[0]==0x06)//send6								
	{ 
		plane.PID_RX[13][0]= (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		plane.PID_RX[13][1]= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		plane.PID_RX[13][2]= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		plane.PID_RX[14][0]= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		plane.PID_RX[14][1]= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		plane.PID_RX[14][2]= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		plane.PID_RX[15][0]= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		plane.PID_RX[15][1]= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		plane.PID_RX[15][2]= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		plane.PID_RX[16][0]= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		plane.PID_RX[16][1]= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		plane.PID_RX[16][2]= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		plane.PID_RX[17][0]= (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		plane.PID_RX[17][1]= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		plane.PID_RX[17][2]= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
		
  }
}


int cnt_timer2_r=0;
u8 cnt_led_rx=0;
void Nrf_Check_Event(void)
{  static u8 ledtx;
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);		//??2401?????
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				
				
				rx_len= NRF_Read_Reg(R_RX_PL_WID);
				
				if(rx_len<33)	//??????33?????,???????
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	//??2401??????
				

				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff);//?????
				}
				if(cnt_led_rx<2)
				cnt_led_rx++;
				else 
				{
				cnt_led_rx=0;
				}
	}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>200 )//0.5ms
		 {	NRF_Write_Reg(FLUSH_RX,0xff);//?????
			
	  	//RX_CH[PIT]=RX_CH[ROL]=RX_CH[YAW]=1500;

			data_rate=0;
		 }
  }
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	if(sta & (1<<MAX_RT))//??,????
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}


void NRF_Send_RC(void)//????
{ 	static u8 cnt_s6;
	uint8_t i;
	vs16 _temp;	
	u8 cnt=0;
  u8 sum = 0;
	NRF24L01_TXDATA[cnt++] = 0x01;
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.THROTTLE);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.THROTTLE);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.YA);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.YA);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.ROL);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.ROL);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.PIT);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.PIT);
	
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.AUX1);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.AUX1);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.AUX2);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.AUX2);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.AUX3);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.AUX3);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.AUX4);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.AUX4);
	NRF24L01_TXDATA[cnt++] = BYTE1(Rc_Get.AUX5);
	NRF24L01_TXDATA[cnt++] = BYTE0(Rc_Get.AUX5);

  NRF24L01_TXDATA[cnt++] = key_o[4]<<4|key_o[3]<<3|key_o[2]<<2|key_o[1]<<1|key_o[0];
	NRF24L01_TXDATA[cnt++] = plane.mag_cal<<2|plane.acc_cal<<1|plane.gyro_cal;
	if(plane.acc_cal)
		plane.acc_cal=0;
	if(plane.gyro_cal)
		plane.gyro_cal=0;
	if(plane.mag_cal)
		plane.mag_cal=0;
	NRF24L01_TXDATA[cnt++] = plane.read_pid;

	if(acc_3d_step==6)
		cnt_s6++;
	if(cnt_s6>3){cnt_s6=0;
    		acc_3d_step=0;
	}
  plane.acc_3d_cal=	acc_3d_step;
	
	NRF24L01_TXDATA[cnt++] = plane.acc_3d_cal;
	if(plane.read_pid)
		plane.read_pid=0;
	
	
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}



void NRF_Send_PID1(void)
{
	uint8_t i=0,j=0;
	vs16 _temp;	
	u8 cnt=0;
  u8 sum = 0;
	NRF24L01_TXDATA[cnt++] = 0x02;
	for(i=0;i<5;i++){
		for(j=0;j<3;j++){
			NRF24L01_TXDATA[cnt++] = BYTE1(plane.PID[i][j]);
			NRF24L01_TXDATA[cnt++] = BYTE0(plane.PID[i][j]);
		}
	}
	
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void NRF_Send_PID2(void)
{
	uint8_t i=0,j;
	vs16 _temp;	
	u8 cnt=0;
  u8 sum = 0;
	NRF24L01_TXDATA[cnt++] = 0x03;
	for(i=5;i<10;i++){
		for(j=0;j<3;j++){
			NRF24L01_TXDATA[cnt++] = BYTE1(plane.PID[i][j]);
			NRF24L01_TXDATA[cnt++] = BYTE0(plane.PID[i][j]);
		}
	}
	
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void NRF_Send_PID3(void)
{
	uint8_t i=0,j;
	vs16 _temp;	
	u8 cnt=0;
  u8 sum = 0;
	NRF24L01_TXDATA[cnt++] = 0x04;
	for(i=10;i<15;i++){
		for(j=0;j<3;j++){
			NRF24L01_TXDATA[cnt++] = BYTE1(plane.PID[i][j]);
			NRF24L01_TXDATA[cnt++] = BYTE0(plane.PID[i][j]);
		}
	}
	
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void NRF_Send_PID4(void)
{
	uint8_t i=0,j;
	vs16 _temp;	
	u8 cnt=0;
  u8 sum = 0;
	NRF24L01_TXDATA[cnt++] = 0x05;
	for(i=15;i<18;i++){
		for(j=0;j<3;j++){
			NRF24L01_TXDATA[cnt++] = BYTE1(plane.PID[i][j]);
			NRF24L01_TXDATA[cnt++] = BYTE0(plane.PID[i][j]);
		}
	}
	
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void RC_Send_Task(void)
{
static u16 cnt[4]={0,0,0,0};
static u8 pid_flag=0;
	

	if(cnt[0]++>2-1){
	 cnt[0]=0;	

	}

	if(send_pid){
		if(pid_flag==0){pid_flag=1;
		NRF_Send_PID1();}
		else if(pid_flag==1){pid_flag=2;
		NRF_Send_PID2();
    }
		else if(pid_flag==2){pid_flag=3;
		NRF_Send_PID3();
    }
		else if(pid_flag==3){pid_flag=0;
		NRF_Send_PID4();
		cnt[1]++;
    }
		
		if(cnt[1]>5){cnt[1]=0;
		send_pid=0;	
		}
	}else 
	NRF_Send_RC();
}


