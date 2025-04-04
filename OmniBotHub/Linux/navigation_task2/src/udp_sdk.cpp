#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "sys_time.h"
#include "mem_node.h"
#include "base_struct.h"
#include "comm.h"
#include "can.h"
#include <time.h> 
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <pthread.h>

//----------------------------UDP 通讯------------
int usb_send_cnt_SDK=0;
char SendBuff_USB_SDK[500];
char RxBuffer_USB_SDK[255];
char RxStateUSB_SDK=0;
int _data_lenUSB_SDK=0;
int _data_cntUSB_SDK=0;

static float floatFromData( char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return *(float *)&i;
}

static char charFromData( char *data,int* anal_cnt)
{
	int temp=*anal_cnt ;
	*anal_cnt +=1;
	return *(data+temp);
}

static int intFromData( char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return i;
}

static void setDataInt_SDK(int i)
{
	#if 0
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = (i >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 8) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 16) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 24) >> 24);
	#else
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 24) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 16) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 8) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = (i >> 24);
	#endif
}

static void setDataFloat_SDK(float f)
{
	int i = *(int *)&f;
	#if 0
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = (i >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 8) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 16) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 24) >> 24);
	#else
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 24) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 16) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = ((i << 8) >> 24);
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = (i >> 24);
	#endif
}

void Anal_UDP_SDK(char *data_buf,char num)
{  
	static int link_cnt=0;
	char id;
	char temp_char=0,need_save=0;
	char sum = 0;
	char i,j;
	int anal_cnt=4;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1))){		
		i=0;
		return;		//判断sum
	}
	if(!(*(data_buf)==0xCA && *(data_buf+1)==0xCF))	{
		i=0;
		return;		//判断帧头
	}
	if(!moco_sdk.connect){
		moco_sdk.connect=1;	
		printf("Navigation::UDP SDK Link=%d!!\n",link_cnt++);	
	}
	
  if(*(data_buf+2)==0x93)//
  { 
	moco_sdk.loss_cnt=0;  
    moco_sdk.ros_connect=charFromData(data_buf,&anal_cnt);
	moco_sdk.gait_mode=charFromData(data_buf,&anal_cnt);
	moco_sdk.rc_spd_cmd[Xr]=floatFromData(data_buf,&anal_cnt);
	moco_sdk.rc_spd_cmd[Yr]=floatFromData(data_buf,&anal_cnt);
    moco_sdk.rc_spd_cmd[Zr]=floatFromData(data_buf,&anal_cnt);
    moco_sdk.rc_att_cmd[PITr]=floatFromData(data_buf,&anal_cnt);
	moco_sdk.rc_att_cmd[ROLr]=floatFromData(data_buf,&anal_cnt);
	moco_sdk.rc_att_cmd[YAWr]=floatFromData(data_buf,&anal_cnt);
    moco_sdk.rc_att_rate_cmd[Xr]=floatFromData(data_buf,&anal_cnt);
    moco_sdk.rc_att_rate_cmd[Yr]=floatFromData(data_buf,&anal_cnt);
	moco_sdk.rc_att_rate_cmd[Zr]=floatFromData(data_buf,&anal_cnt);
	moco_sdk.rc_pos_cmd[Xr]=floatFromData(data_buf,&anal_cnt);
    moco_sdk.rc_pos_cmd[Yr]=floatFromData(data_buf,&anal_cnt);
	moco_sdk.rc_pos_cmd[Zr]=floatFromData(data_buf,&anal_cnt);

	//printf("mode=%d spdx=%f spd_y=%f spd_yaw=%f\n",  moco_sdk.ros_connect*10+moco_sdk.gait_mode,moco_sdk.rc_spd_cmd[Xr],moco_sdk.rc_spd_cmd[Yr],moco_sdk.rc_att_rate_cmd[Zr]);
	}
}

void UDP_RX_PROCESS_SDK(char* Buf, int Len)//data coming from anal ROS
{
	int i=0;
	char com_data;
	for(i=0;i<Len;i++)
	{  
		com_data=Buf[i]; 
		if(RxStateUSB_SDK==0&&com_data==0xCA)
		{
			RxStateUSB_SDK=1;
			RxBuffer_USB_SDK[0]=com_data;
		}
		else if(RxStateUSB_SDK==1&&com_data==0xCF)
		{
			RxStateUSB_SDK=2;
			RxBuffer_USB_SDK[1]=com_data;
		}
		else if(RxStateUSB_SDK==2&&com_data>0&&com_data<0XF1)
		{
			RxStateUSB_SDK=3;
			RxBuffer_USB_SDK[2]=com_data;
		}
		else if(RxStateUSB_SDK==3&&com_data<255)
		{
			RxStateUSB_SDK= 4;
			RxBuffer_USB_SDK[3]=com_data;
			_data_lenUSB_SDK = com_data;
			_data_cntUSB_SDK = 0;
		}
		else if(RxStateUSB_SDK==4&&_data_lenUSB_SDK>0)
		{
			_data_lenUSB_SDK--;
			RxBuffer_USB_SDK[4+_data_cntUSB_SDK++]=com_data;
			if(_data_lenUSB_SDK==0)
				RxStateUSB_SDK= 5;
		}
		else if(RxStateUSB_SDK==5)
		{
			RxStateUSB_SDK = 0;
			RxBuffer_USB_SDK[4+_data_cntUSB_SDK]=com_data;
      		Anal_UDP_SDK(RxBuffer_USB_SDK,_data_cntUSB_SDK+5);
			for(i=0;i<255;i++)
				RxBuffer_USB_SDK[i]=0;
		}
		else
			RxStateUSB_SDK = 0;
	}
}

//==========================发送接口函数==================

void data_per_sdk_robot_state(void)//send odom to ros node
{ 
	u8 i,j;	u8 sum = 0;
	u16 cnt_reg;

	cnt_reg=usb_send_cnt_SDK;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0xBA;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0xBF;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0x91;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0;

    setDataFloat_SDK(nav_rx.att_now[0]);
    setDataFloat_SDK(nav_rx.att_now[1]);
    setDataFloat_SDK(nav_rx.att_now[2]);
    setDataFloat_SDK(nav_rx.datt_now[0]);
    setDataFloat_SDK(nav_rx.datt_now[1]);
    setDataFloat_SDK(nav_rx.datt_now[2]);

    setDataFloat_SDK(nav_rx.com_n_now[0]);
    setDataFloat_SDK(nav_rx.com_n_now[1]);
    setDataFloat_SDK(nav_rx.com_n_now[2]);	
    setDataFloat_SDK(nav_rx.dcom_n_now[0]);
    setDataFloat_SDK(nav_rx.dcom_n_now[1]);
    setDataFloat_SDK(nav_rx.dcom_n_now[2]);		
	setDataFloat_SDK(nav_rx.acc_n_now[0]);
	setDataFloat_SDK(nav_rx.acc_n_now[1]);
	setDataFloat_SDK(nav_rx.acc_n_now[2]);
	
	setDataFloat_SDK(nav_rx.epos_n_now[2].x);
	setDataFloat_SDK(nav_rx.epos_n_now[2].y);
	setDataFloat_SDK(nav_rx.epos_n_now[2].z);
	setDataFloat_SDK(nav_rx.epos_n_now[0].x);
	setDataFloat_SDK(nav_rx.epos_n_now[0].y);
	setDataFloat_SDK(nav_rx.epos_n_now[0].z);	
    setDataFloat_SDK(nav_rx.epos_n_now[3].x);
	setDataFloat_SDK(nav_rx.epos_n_now[3].y);
	setDataFloat_SDK(nav_rx.epos_n_now[3].z);
	setDataFloat_SDK(nav_rx.epos_n_now[1].x);
	setDataFloat_SDK(nav_rx.epos_n_now[1].y);
	setDataFloat_SDK(nav_rx.epos_n_now[1].z);	

	setDataFloat_SDK(nav_rx.sita_now[2][0]);
	setDataFloat_SDK(nav_rx.sita_now[2][1]);
	setDataFloat_SDK(nav_rx.sita_now[2][2]);	
	setDataFloat_SDK(nav_rx.sita_now[0][0]);
	setDataFloat_SDK(nav_rx.sita_now[0][1]);
	setDataFloat_SDK(nav_rx.sita_now[0][2]);	
	setDataFloat_SDK(nav_rx.sita_now[3][0]);
	setDataFloat_SDK(nav_rx.sita_now[3][1]);
	setDataFloat_SDK(nav_rx.sita_now[3][2]);	
	setDataFloat_SDK(nav_rx.sita_now[1][0]);
	setDataFloat_SDK(nav_rx.sita_now[1][1]);
	setDataFloat_SDK(nav_rx.sita_now[1][2]);	

	SendBuff_USB_SDK[cnt_reg+3] =(usb_send_cnt_SDK-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt_SDK;i++)
	sum += SendBuff_USB_SDK[i];
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = sum;
}


void data_per_sdk_robot_system(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 cnt_reg;

	cnt_reg=usb_send_cnt_SDK;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0xBA;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0xBF;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0x92;
	SendBuff_USB_SDK[usb_send_cnt_SDK++]=0;

    SendBuff_USB_SDK[usb_send_cnt_SDK++] =nav_rx.rc_mode;;
    SendBuff_USB_SDK[usb_send_cnt_SDK++] =nav_rx.gait_state;
    setDataFloat_SDK(0.5);	//bat
    setDataFloat_SDK(0.5);	//rc

	SendBuff_USB_SDK[cnt_reg+3] =(usb_send_cnt_SDK-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt_SDK;i++)
	sum += SendBuff_USB_SDK[i];
	SendBuff_USB_SDK[usb_send_cnt_SDK++] = sum;
}

void UDP_OCU_TX_SDK(float dt)//发送
{
	static float timer_param[10]={0};
    static int cnt_usb_send[5]={0};
	static char state=0,i;
	static char state_flag=0,flag1=0;
	int len,t;
	usb_send_cnt_SDK=0;

    for(i=0;i<10;i++)
			timer_param[i]+=dt;

    if(timer_param[3]>0.02){
        timer_param[3]=0;
        data_per_sdk_robot_system();
    }else{
        data_per_sdk_robot_state();
    }
    
}
