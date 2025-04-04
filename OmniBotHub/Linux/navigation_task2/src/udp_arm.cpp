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
int usb_send_cnt_ARM=0;
char SendBuff_USB_ARM[500];
char RxBuffer_USB_ARM[255];
char RxStateUSB_ARM=0;
int _data_lenUSB_ARM=0;
int _data_cntUSB_ARM=0;

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

static void setDataInt_ARM(int i)
{
	#if 0
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = (i >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 8) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 16) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 24) >> 24);
	#else
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 24) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 16) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 8) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = (i >> 24);
	#endif
}

static void setDataFloat_ARM(float f)
{
	int i = *(int *)&f;
	#if 0
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = (i >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 8) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 16) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 24) >> 24);
	#else
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 24) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 16) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = ((i << 8) >> 24);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = (i >> 24);
	#endif
}

void Anal_UDP_ARM(char *data_buf,char num)
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
	
    if(*(data_buf+2)==0x01)//
    {

    nav_tx.arm_ocu.power=charFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.arm_mode=charFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.control_mode=charFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.cap=charFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.pos_set_n.x=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.pos_set_n.y=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.pos_set_n.z=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.spd_set_n.x=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.spd_set_n.y=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.spd_set_n.z=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.att_set_n.x=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.att_set_n.y=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.att_set_n.z=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.rate_set_n.x=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.rate_set_n.y=floatFromData(data_buf,&anal_cnt);
    nav_tx.arm_ocu.rate_set_n.z=floatFromData(data_buf,&anal_cnt);
#if 0
    printf("power=%d arm=%d con=%d cap=%d spdx=%f spd_y=%f spd_z=%f\n",
           nav_tx.arm_ocu.power,
           nav_tx.arm_ocu.arm_mode,
           nav_tx.arm_ocu.control_mode,
           nav_tx.arm_ocu.cap,
           nav_tx.arm_ocu.spd_set_n.x,
           nav_tx.arm_ocu.spd_set_n.y,
           nav_tx.arm_ocu.spd_set_n.z);
#endif
    }
}

void UDP_RX_PROCESS_ARM(char* Buf, int Len)//data coming from anal ROS
{
	int i=0;
	char com_data;
	for(i=0;i<Len;i++)
	{  
		com_data=Buf[i]; 
        if(RxStateUSB_ARM==0&&com_data==0xCA)
		{
            RxStateUSB_ARM=1;
            RxBuffer_USB_ARM[0]=com_data;
		}
        else if(RxStateUSB_ARM==1&&com_data==0xCF)
		{
            RxStateUSB_ARM=2;
            RxBuffer_USB_ARM[1]=com_data;
		}
        else if(RxStateUSB_ARM==2&&com_data>0&&com_data<0XF1)
		{
            RxStateUSB_ARM=3;
            RxBuffer_USB_ARM[2]=com_data;
		}
        else if(RxStateUSB_ARM==3&&com_data<255)
		{
            RxStateUSB_ARM= 4;
            RxBuffer_USB_ARM[3]=com_data;
            _data_lenUSB_ARM = com_data;
            _data_cntUSB_ARM = 0;
		}
        else if(RxStateUSB_ARM==4&&_data_lenUSB_ARM>0)
		{
            _data_lenUSB_ARM--;
            RxBuffer_USB_ARM[4+_data_cntUSB_ARM++]=com_data;
            if(_data_lenUSB_ARM==0)
                RxStateUSB_ARM= 5;
		}
        else if(RxStateUSB_ARM==5)
		{
            RxStateUSB_ARM = 0;
            RxBuffer_USB_ARM[4+_data_cntUSB_ARM]=com_data;
            Anal_UDP_ARM(RxBuffer_USB_ARM,_data_cntUSB_ARM+5);
			for(i=0;i<255;i++)
                RxBuffer_USB_ARM[i]=0;
		}
		else
            RxStateUSB_ARM = 0;
	}
}

//==========================发送接口函数==================

void data_per_ARM_robot_state(void)//send odom to ros node
{ 
	u8 i,j;	u8 sum = 0;
	u16 cnt_reg;

    cnt_reg=usb_send_cnt_ARM;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0xBA;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0xBF;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0x91;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0;

    setDataFloat_ARM(nav_rx.att_now[0]);
    setDataFloat_ARM(nav_rx.att_now[1]);
    setDataFloat_ARM(nav_rx.att_now[2]);
    setDataFloat_ARM(nav_rx.datt_now[0]);
    setDataFloat_ARM(nav_rx.datt_now[1]);
    setDataFloat_ARM(nav_rx.datt_now[2]);

    setDataFloat_ARM(nav_rx.com_n_now[0]);
    setDataFloat_ARM(nav_rx.com_n_now[1]);
    setDataFloat_ARM(nav_rx.com_n_now[2]);
    setDataFloat_ARM(nav_rx.dcom_n_now[0]);
    setDataFloat_ARM(nav_rx.dcom_n_now[1]);
    setDataFloat_ARM(nav_rx.dcom_n_now[2]);
    setDataFloat_ARM(nav_rx.acc_n_now[0]);
    setDataFloat_ARM(nav_rx.acc_n_now[1]);
    setDataFloat_ARM(nav_rx.acc_n_now[2]);
	
    setDataFloat_ARM(nav_rx.epos_n_now[2].x);
    setDataFloat_ARM(nav_rx.epos_n_now[2].y);
    setDataFloat_ARM(nav_rx.epos_n_now[2].z);
    setDataFloat_ARM(nav_rx.epos_n_now[0].x);
    setDataFloat_ARM(nav_rx.epos_n_now[0].y);
    setDataFloat_ARM(nav_rx.epos_n_now[0].z);
    setDataFloat_ARM(nav_rx.epos_n_now[3].x);
    setDataFloat_ARM(nav_rx.epos_n_now[3].y);
    setDataFloat_ARM(nav_rx.epos_n_now[3].z);
    setDataFloat_ARM(nav_rx.epos_n_now[1].x);
    setDataFloat_ARM(nav_rx.epos_n_now[1].y);
    setDataFloat_ARM(nav_rx.epos_n_now[1].z);

    setDataFloat_ARM(nav_rx.sita_now[2][0]);
    setDataFloat_ARM(nav_rx.sita_now[2][1]);
    setDataFloat_ARM(nav_rx.sita_now[2][2]);
    setDataFloat_ARM(nav_rx.sita_now[0][0]);
    setDataFloat_ARM(nav_rx.sita_now[0][1]);
    setDataFloat_ARM(nav_rx.sita_now[0][2]);
    setDataFloat_ARM(nav_rx.sita_now[3][0]);
    setDataFloat_ARM(nav_rx.sita_now[3][1]);
    setDataFloat_ARM(nav_rx.sita_now[3][2]);
    setDataFloat_ARM(nav_rx.sita_now[1][0]);
    setDataFloat_ARM(nav_rx.sita_now[1][1]);
    setDataFloat_ARM(nav_rx.sita_now[1][2]);

    SendBuff_USB_ARM[cnt_reg+3] =(usb_send_cnt_ARM-cnt_reg)-4;
        for( i=cnt_reg;i<usb_send_cnt_ARM;i++)
    sum += SendBuff_USB_ARM[i];
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = sum;
}


void data_per_ARM_robot_system(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 cnt_reg;

    cnt_reg=usb_send_cnt_ARM;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0xBA;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0xBF;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0x92;
    SendBuff_USB_ARM[usb_send_cnt_ARM++]=0;

    setDataFloat_ARM(0.3);
    setDataFloat_ARM(0.5);
    setDataFloat_ARM(0.1);
    setDataFloat_ARM(0.2);
    setDataFloat_ARM(0.02);


    setDataFloat_ARM(nav_rx.dcom_n_tar[0]);
    setDataFloat_ARM(nav_rx.dcom_n_tar[1]);
    setDataFloat_ARM(nav_rx.datt_tar[2]);
    setDataFloat_ARM(nav_rx.com_n_tar[2]);

    // moco.system.bat=floatFromDataf(data_buf,&anal_cnt);	
    // moco.system.rc_weight=floatFromDataf(data_buf,&anal_cnt);	

    // moco.param.rc_mode=charFromDataf(data_buf,&anal_cnt);	
    // moco.param.robot_mode=charFromDataf(data_buf,&anal_cnt);	
    // moco.param.imu_mode=charFromDataf(data_buf,&anal_cnt);	
    setDataFloat_ARM(1);
    setDataFloat_ARM(1);
    setDataFloat_ARM(nav_rx.com_n_tar[2]);
    SendBuff_USB_ARM[usb_send_cnt_ARM++] =1;
    SendBuff_USB_ARM[usb_send_cnt_ARM++] =nav_rx.gait_state;
    SendBuff_USB_ARM[usb_send_cnt_ARM++] =1;

    SendBuff_USB_ARM[cnt_reg+3] =(usb_send_cnt_ARM-cnt_reg)-4;
        for( i=cnt_reg;i<usb_send_cnt_ARM;i++)
    sum += SendBuff_USB_ARM[i];
    SendBuff_USB_ARM[usb_send_cnt_ARM++] = sum;
}

void UDP_OCU_TX_ARM(float dt)//发送
{
	static float timer_param[10]={0};
    static int cnt_usb_send[5]={0};
	static char state=0,i;
	static char state_flag=0,flag1=0;
	int len,t;
    usb_send_cnt_ARM=0;

    for(i=0;i<10;i++)
			timer_param[i]+=dt;

    if(timer_param[3]>0.02){
        timer_param[3]=0;
        data_per_ARM_robot_system();
    }else{
        data_per_ARM_robot_state();
    }
    
}
