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
int usb_send_cnt=0;
char SendBuff_USB[500];
char RxBuffer_USB[255];
char RxStateUSB=0;
int _data_lenUSB=0;
int _data_cntUSB=0;
_MEMS mems;
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

static void setDataInt(int i)
{
	#if 0
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	#else
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	#endif
}

static void setDataFloat(float f)
{
	int i = *(int *)&f;
	#if 0
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	#else
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	#endif
}

void Anal_UDP(char *data_buf,char num)
{  
	static int link_cnt=0;
    float temp_float;
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
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	{
		i=0;
		return;		//判断帧头
	}
	if(!ocu.connect){
		ocu.connect=1;	
		printf("Navigation::UDP OCU Link=%d!!\n",link_cnt++);	
	}
	
  if(*(data_buf+2)==0x31)//ocu speed keyboard
  { 
		
	ocu.mode=2;
	ocu.loss_cnt=0;
	ocu.up_mode=charFromData(data_buf,&anal_cnt);
	ocu.rc_spd_b[Xr]=floatFromData(data_buf,&anal_cnt);
	ocu.rc_spd_b[Yr]=floatFromData(data_buf,&anal_cnt);
	ocu.rc_spd_b[Zr]=floatFromData(data_buf,&anal_cnt);
	ocu.rc_rate_b[PITr]=floatFromData(data_buf,&anal_cnt);
	ocu.rc_rate_b[ROLr]=floatFromData(data_buf,&anal_cnt);
	ocu.rc_rate_b[YAWr]=floatFromData(data_buf,&anal_cnt);
	ocu.record.en_record=charFromData(data_buf,&anal_cnt);
	
	//printf("%f %f %f\n", ocu.rc_spd_b[0], ocu.rc_spd_b[1], ocu.rc_spd_b[2]);
	}else if(*(data_buf+2)==0x32)//ocu speed remote
  { 
		ocu.connect=1;
		ocu.loss_cnt=0;
		ocu.mode=2;
		
		ocu.up_mode=charFromData(data_buf,&anal_cnt);
		ocu.record.en_record=charFromData(data_buf,&anal_cnt);
		//printf("2\n");
		ocu.rc_spd_w[Xr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_spd_w[Yr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_att_w[PITr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_att_w[ROLr]=floatFromData(data_buf,&anal_cnt);
		ocu.rate_yaw_w=floatFromData(data_buf,&anal_cnt);
		ocu.key_st=charFromData(data_buf,&anal_cnt);
		ocu.key_back=charFromData(data_buf,&anal_cnt);
		ocu.key_lr=intFromData(data_buf,&anal_cnt);
		ocu.key_ud=intFromData(data_buf,&anal_cnt);
		ocu.key_x=charFromData(data_buf,&anal_cnt);
		ocu.key_y=charFromData(data_buf,&anal_cnt);
		ocu.key_b=charFromData(data_buf,&anal_cnt);
		ocu.key_a=charFromData(data_buf,&anal_cnt);
		ocu.key_ll=charFromData(data_buf,&anal_cnt);
		ocu.key_rr=charFromData(data_buf,&anal_cnt);
//        printf("%f %f %f\n", ocu.rc_spd_w[0], ocu.rc_spd_w[1], ocu.rc_spd_b[2]);
//        printf("%f %f %f\n", ocu.rc_spd_w[0], ocu.rc_spd_w[1], ocu.rc_spd_b[2]);
//        printf("connect=%d st=%d back=%d x=%d a=%d\n",
//            ocu.connect ,  ocu.key_st,ocu.key_back, ocu.key_x,ocu.key_a );
//        printf("b=%d y=%d ll=%d rr=%d lr=%d up=%d\n",
//            ocu.key_b ,  ocu.key_y,ocu.key_ll, ocu.key_rr,ocu.key_lr ,ocu.key_ud);
//        printf("spd0=%f spd1=%f att0=%f att1=%f yaw=%f\n",
//            ocu.rc_spd_w[0],ocu.rc_spd_w[1],ocu.rc_att_w[0],ocu.rc_att_w[1],ocu.rate_yaw_w);
	}
	else if(*(data_buf+2)==0x21)//sensor config IMU 传感器标定
  { 
		
	  	ocu.loss_cnt=0;
		need_save=charFromData(data_buf,&anal_cnt);//need_save
		temp_char=charFromData(data_buf,&anal_cnt);//cal_mode
		mems.imu_pos.x=floatFromData(data_buf,&anal_cnt);
		mems.imu_pos.y=floatFromData(data_buf,&anal_cnt);
		mems.imu_pos.z=floatFromData(data_buf,&anal_cnt);
//		mems.imu_att.x=floatFromData(data_buf,&anal_cnt);
//		mems.imu_att.y=floatFromData(data_buf,&anal_cnt);
//		mems.imu_att.z=floatFromData(data_buf,&anal_cnt);
        temp_float=floatFromData(data_buf,&anal_cnt);
        temp_float=floatFromData(data_buf,&anal_cnt);
        temp_float=floatFromData(data_buf,&anal_cnt);

		mems.gps_pos.x=floatFromData(data_buf,&anal_cnt);
		mems.gps_pos.y=floatFromData(data_buf,&anal_cnt);
		mems.gps_pos.z=floatFromData(data_buf,&anal_cnt);
		if(need_save){
		if(temp_char==1)
			mems.Acc_CALIBRATE=1;
		else if(temp_char==2)
			mems.Gyro_CALIBRATE=1;
		else if(temp_char==3)
			mems.Mag_CALIBRATE=1;
		}else
		{
		mems.Acc_Offset.x=floatFromData(data_buf,&anal_cnt)*4096;
		mems.Acc_Offset.y=floatFromData(data_buf,&anal_cnt)*4096;
		mems.Acc_Offset.y=floatFromData(data_buf,&anal_cnt)*4096;
		mems.Acc_Scale.x=floatFromData(data_buf,&anal_cnt);
		mems.Acc_Scale.y=floatFromData(data_buf,&anal_cnt);
		mems.Acc_Scale.z=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Offset.x=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Offset.y=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Offset.z=floatFromData(data_buf,&anal_cnt);		
		mems.Gyro_Scale.x=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Scale.y=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Scale.z=floatFromData(data_buf,&anal_cnt);	
		mems.Mag_Offset.x=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Offset.y=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Offset.z=floatFromData(data_buf,&anal_cnt);				
		mems.Mag_Gain.x=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Gain.y=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Gain.z=floatFromData(data_buf,&anal_cnt);		
		}
//        printf("mems.Acc_CALIBRATE=%d\n",mems.Acc_CALIBRATE);
//        printf("mems.Gyro_CALIBRATE=%d\n",mems.Gyro_CALIBRATE);
	}
}

void UDP_RX_PROCESS(char* Buf, int Len)
{
	int i=0;
	char com_data;
	for(i=0;i<Len;i++)
	{  
		com_data=Buf[i]; 
		if(RxStateUSB==0&&com_data==0xAA)
		{
			RxStateUSB=1;
			RxBuffer_USB[0]=com_data;
		}
		else if(RxStateUSB==1&&com_data==0xAF)
		{
			RxStateUSB=2;
			RxBuffer_USB[1]=com_data;
		}
		else if(RxStateUSB==2&&com_data>0&&com_data<0XF1)
		{
			RxStateUSB=3;
			RxBuffer_USB[2]=com_data;
		}
		else if(RxStateUSB==3&&com_data<255)
		{
			RxStateUSB= 4;
			RxBuffer_USB[3]=com_data;
			_data_lenUSB = com_data;
			_data_cntUSB = 0;
		}
		else if(RxStateUSB==4&&_data_lenUSB>0)
		{
			_data_lenUSB--;
			RxBuffer_USB[4+_data_cntUSB++]=com_data;
			if(_data_lenUSB==0)
				RxStateUSB= 5;
		}
		else if(RxStateUSB==5)
		{
			RxStateUSB = 0;
			RxBuffer_USB[4+_data_cntUSB]=com_data;
      		Anal_UDP(RxBuffer_USB,_data_cntUSB+5);
			for(i=0;i<255;i++)
				RxBuffer_USB[i]=0;
		}
		else
			RxStateUSB = 0;
	}
}

//==========================发送接口函数==================

void data_per_usb_record1_config(char sel)//记录数据
{ 
	u8 i,j;	u8 sum = 0;
	u16 cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x71+sel;
	SendBuff_USB[usb_send_cnt++]=0;
	if(sel<4){//leg data
		SendBuff_USB[usb_send_cnt++]=nav_rx.is_ground[sel];
		SendBuff_USB[usb_send_cnt++]=nav_rx.is_touch[sel];
		SendBuff_USB[usb_send_cnt++]=nav_rx.force_en_flag[sel];
		SendBuff_USB[usb_send_cnt++]=nav_rx.leg_state[sel];

		setDataFloat(nav_rx.epos_n_tar[sel].x);
		setDataFloat(nav_rx.epos_n_tar[sel].y);
		setDataFloat(nav_rx.epos_n_tar[sel].z);
		
		setDataFloat(nav_rx.epos_n_now[sel].x);
		setDataFloat(nav_rx.epos_n_now[sel].y);
		setDataFloat(nav_rx.epos_n_now[sel].z);
		
		setDataFloat(nav_rx.depos_n_tar[sel].x);
		setDataFloat(nav_rx.depos_n_tar[sel].y);
		setDataFloat(nav_rx.depos_n_tar[sel].z);	
		
		setDataFloat(nav_rx.depos_n_now[sel].x);
		setDataFloat(nav_rx.depos_n_now[sel].y);
		setDataFloat(nav_rx.depos_n_now[sel].z);	
		
		setDataFloat(nav_rx.sita_tar[sel][0]);
		setDataFloat(nav_rx.sita_tar[sel][1]);
		setDataFloat(nav_rx.sita_tar[sel][2]);	

		setDataFloat(nav_rx.sita_now[sel][0]);
		setDataFloat(nav_rx.sita_now[sel][1]);
		setDataFloat(nav_rx.sita_now[sel][2]);	
		
		
		setDataFloat(nav_rx.GRF_n_tar[sel].x);
		setDataFloat(nav_rx.GRF_n_tar[sel].y);
		setDataFloat(nav_rx.GRF_n_tar[sel].z);	
		
		setDataFloat(nav_rx.GRF_n_now[sel].x);
		setDataFloat(nav_rx.GRF_n_now[sel].y);
		setDataFloat(nav_rx.GRF_n_now[sel].z);	
	}else{//com
		setDataFloat(nav_rx.att_tar[0]);
		setDataFloat(nav_rx.att_tar[1]);
		setDataFloat(nav_rx.att_tar[2]);
		
		setDataFloat(nav_rx.att_now[0]);
		setDataFloat(nav_rx.att_now[1]);
		setDataFloat(nav_rx.att_now[2]);	
		
		setDataFloat(nav_rx.datt_now[0]);
		setDataFloat(nav_rx.datt_now[1]);
		setDataFloat(nav_rx.datt_now[2]);		

		
		setDataFloat(nav_rx.com_n_tar[0]);
		setDataFloat(nav_rx.com_n_tar[1]);
		setDataFloat(nav_rx.com_n_tar[2]);		
		
		setDataFloat(nav_rx.com_n_now[0]);
		setDataFloat(nav_rx.com_n_now[1]);
		setDataFloat(nav_rx.com_n_now[2]);	

		setDataFloat(nav_rx.dcom_n_tar[0]);
		setDataFloat(nav_rx.dcom_n_tar[1]);
		setDataFloat(nav_rx.dcom_n_tar[2]);	
		
		setDataFloat(nav_rx.dcom_n_now[0]);
		setDataFloat(nav_rx.dcom_n_now[1]);
		setDataFloat(nav_rx.dcom_n_now[2]);		

		setDataFloat(nav_rx.ground_att_now[0]);
		setDataFloat(nav_rx.ground_att_now[1]);
		setDataFloat(nav_rx.ground_att_now[2]);	
	}

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state1(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	static float timer=0;
	timer+=0.2;
	//printf("ss\n");
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x04;
	SendBuff_USB[usb_send_cnt++]=0;
	//printf("att0=%f att1=%f att2=%f dt=%f\n",nav_rx.att_now[0],nav_rx.att_now[1],nav_rx.att_now[2], Get_Cycle_T(0));
	setDataFloat(nav_rx.att_now[0]);//vmc_all.att[PITr]);//setDataFloat(5*sin(timer));//vmc_all.att[PITr]);
	setDataFloat(nav_rx.att_now[1]);//vmc_all.att[ROLr]);
	setDataFloat(nav_rx.att_now[2]);//vmc_all.att[YAWr]);
	setDataFloat(nav_rx.datt_now[0]);//vmc_all.att[PITr]);
	setDataFloat(nav_rx.datt_now[1]);//vmc_all.att[ROLr]);
	setDataFloat(nav_rx.datt_now[2]);//vmc_all.att[YAWr]);
	
    setDataFloat(nav_rx.com_n_now[0]);
    setDataFloat(nav_rx.com_n_now[1]);
    setDataFloat(nav_rx.com_n_now[2]);
    setDataFloat(nav_rx.dcom_n_now[0]);
    setDataFloat(nav_rx.dcom_n_now[1]);
    setDataFloat(nav_rx.dcom_n_now[2]);
	setDataFloat(nav_rx.acc_n_now[0]);
	setDataFloat(nav_rx.acc_n_now[1]);
	setDataFloat(nav_rx.acc_n_now[2]);
	
	setDataFloat(nav_rx.epos_n_now[2].x);
	setDataFloat(nav_rx.epos_n_now[2].y);
	setDataFloat(nav_rx.epos_n_now[2].z);
	setDataFloat(nav_rx.epos_n_now[0].x);
	setDataFloat(nav_rx.epos_n_now[0].y);
	setDataFloat(nav_rx.epos_n_now[0].z);	


	setDataFloat(0);//(bat.percent);
	setDataFloat(0);//Rc_Get.signal_rate);
	
	setDataFloat(nav_rx.dcom_n_tar[0]);	
	setDataFloat(nav_rx.dcom_n_tar[1]);	
	setDataFloat(nav_rx.dcom_n_tar[2]);	
	setDataFloat(fabs(nav_rx.com_n_tar[2]));	
		
    SendBuff_USB[usb_send_cnt++]=nav_rx.gait_state;
    //printf("nav_rx.gait_state=%d\n",nav_rx.gait_state);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state2(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x41;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(nav_rx.epos_n_now[3].x);
	setDataFloat(nav_rx.epos_n_now[3].y);
	setDataFloat(nav_rx.epos_n_now[3].z);
	setDataFloat(nav_rx.epos_n_now[1].x);
	setDataFloat(nav_rx.epos_n_now[1].y);
	setDataFloat(nav_rx.epos_n_now[1].z);	

	char id=2;

	if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][0]&&nav_rx.motor_ready[id][0])
		setDataFloat(nav_rx.sita_now[id][0]);
	else{
		if(!nav_rx.leg_connect[id])
			setDataFloat(999);
		else if(!nav_rx.motor_connect[id][0])
			setDataFloat(666);
		else
			setDataFloat(333);
	}
	
    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][1]&&nav_rx.motor_ready[id][1])
        setDataFloat(nav_rx.sita_now[id][1]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][1])
            setDataFloat(666);
        else
            setDataFloat(333);
    }


    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][2]&&nav_rx.motor_ready[id][2])
        setDataFloat(nav_rx.sita_now[id][2]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][2])
            setDataFloat(666);
        else
            setDataFloat(333);
    }
	

	id=0;

	if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][0]&&nav_rx.motor_ready[id][0])
		setDataFloat(nav_rx.sita_now[id][0]);
	else{
		if(!nav_rx.leg_connect[id])
			setDataFloat(999);
		else if(!nav_rx.motor_connect[id][0])
			setDataFloat(666);
		else
			setDataFloat(333);
	}

    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][1]&&nav_rx.motor_ready[id][1])
        setDataFloat(nav_rx.sita_now[id][1]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][1])
            setDataFloat(666);
        else
            setDataFloat(333);
    }

    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][2]&&nav_rx.motor_ready[id][2])
        setDataFloat(nav_rx.sita_now[id][2]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][2])
            setDataFloat(666);
        else
            setDataFloat(333);
    }
	
	id=3;

	if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][0]&&nav_rx.motor_ready[id][0])
		setDataFloat(nav_rx.sita_now[id][0]);
	else{
		if(!nav_rx.leg_connect[id])
			setDataFloat(999);
		else if(!nav_rx.motor_connect[id][0])
			setDataFloat(666);
		else
			setDataFloat(333);
	}

    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][1]&&nav_rx.motor_ready[id][1])
        setDataFloat(nav_rx.sita_now[id][1]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][1])
            setDataFloat(666);
        else
            setDataFloat(333);
    }

    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][2]&&nav_rx.motor_ready[id][2])
        setDataFloat(nav_rx.sita_now[id][2]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][2])
            setDataFloat(666);
        else
            setDataFloat(333);
    }
	
	id=1;

	if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][0]&&nav_rx.motor_ready[id][0])
		setDataFloat(nav_rx.sita_now[id][0]);
	else{
		if(!nav_rx.leg_connect[id])
			setDataFloat(999);
		else if(!nav_rx.motor_connect[id][0])
			setDataFloat(666);
		else
			setDataFloat(333);
	}

    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][1]&&nav_rx.motor_ready[id][1])
        setDataFloat(nav_rx.sita_now[id][1]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][1])
            setDataFloat(666);
        else
            setDataFloat(333);
    }

    if(nav_rx.leg_connect[id] &&nav_rx.motor_connect[id][2]&&nav_rx.motor_ready[id][2])
        setDataFloat(nav_rx.sita_now[id][2]);
    else{
        if(!nav_rx.leg_connect[id])
            setDataFloat(999);
        else if(!nav_rx.motor_connect[id][2])
            setDataFloat(666);
        else
            setDataFloat(333);
    }
	
	setDataFloat(vmc[2].param.sita1_off);
	setDataFloat(vmc[2].param.sita2_off);
	setDataFloat(vmc[2].param.sita3_off);
	setDataFloat(vmc[0].param.sita1_off);
	setDataFloat(vmc[0].param.sita2_off);
	setDataFloat(vmc[0].param.sita3_off);
	setDataFloat(vmc[3].param.sita1_off);
	setDataFloat(vmc[3].param.sita2_off);
	setDataFloat(vmc[3].param.sita3_off);
	setDataFloat(vmc[1].param.sita1_off);
	setDataFloat(vmc[1].param.sita2_off);
	setDataFloat(vmc[1].param.sita3_off);
if((vmc_all.gait_mode==STAND_IMU||vmc_all.gait_mode==STAND_RC||vmc_all.gait_mode==TROT)){
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_ground[2];
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_ground[0];
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_ground[1];
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_ground[3];
}
else{
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_touch[2];
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_touch[0];
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_touch[1];
	SendBuff_USB[usb_send_cnt++]=nav_rx.is_touch[3];
}

	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state3(void)//joint
{
    int i,id=0;	u8 sum = 0;
    u16 _cnt=0,cnt_reg;

    cnt_reg=usb_send_cnt;
    SendBuff_USB[usb_send_cnt++]=0xBA;
    SendBuff_USB[usb_send_cnt++]=0xBF;
    SendBuff_USB[usb_send_cnt++]=0x90;
    SendBuff_USB[usb_send_cnt++]=0;

    for(id=0;id<14;id++){
            setDataFloat(nav_rx.q_set[id]);
    }
    for(id=0;id<14;id++){
        if(nav_rx.motor_connect_all[id])
            setDataFloat(nav_rx.q_now[id]);
        else
            setDataFloat(333);
    }

    SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
        for( i=cnt_reg;i<usb_send_cnt;i++)
    sum += SendBuff_USB[i];
    SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state4(void)//tau
{
    int i,id=0;	u8 sum = 0;
    u16 _cnt=0,cnt_reg;

    cnt_reg=usb_send_cnt;
    SendBuff_USB[usb_send_cnt++]=0xBA;
    SendBuff_USB[usb_send_cnt++]=0xBF;
    SendBuff_USB[usb_send_cnt++]=0x91;
    SendBuff_USB[usb_send_cnt++]=0;

    for(id=0;id<14;id++){
            setDataFloat(nav_rx.t_set[id]);
    }
    for(id=0;id<14;id++){
        if(nav_rx.motor_connect_all[id])
            setDataFloat(nav_rx.t_now[id]);
        else
            setDataFloat(333);
    }

    SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
        for( i=cnt_reg;i<usb_send_cnt;i++)
    sum += SendBuff_USB[i];
    SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_controller_config(void)
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x01;
	SendBuff_USB[usb_send_cnt++]=0;

  	setDataFloat(vmc_all.W);
	setDataFloat(vmc_all.H);
	setDataFloat(vmc_all.mess);
  	setDataFloat(vmc_all.l1);
	setDataFloat(vmc_all.l2);
	setDataFloat(vmc_all.l3);
	setDataFloat(vmc_all.param.MAX_Z);
	setDataFloat(vmc_all.param.MIN_Z);
	setDataFloat(vmc_all.param.MAX_X);
	setDataFloat(vmc_all.param.MIN_X);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[0]);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[1]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[0]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[1]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[0]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[1]);
	setDataFloat(vmc_all.param.param_vmc.side_off[0]);
	setDataFloat(vmc_all.param.param_vmc.side_off[1]);
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.en_gait_switch;//gait switch
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.en_fall_protect;//fall switch
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.rc_type;
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.dj_type;//dj type
	SendBuff_USB[usb_send_cnt++]=nav_rx.gait_state;//机器人配置
    SendBuff_USB[usb_send_cnt++]=MOCO12_ANYMAL;//vmc_all.param.robot_type;
	SendBuff_USB[usb_send_cnt++]=(0);//module.acc_imu;
    SendBuff_USB[usb_send_cnt++]=LOOP;//vmc_all.param.leg_type;
	SendBuff_USB[usb_send_cnt++]=(vmc[0].param.invert_knee_epos[Xr]==-1);
    SendBuff_USB[usb_send_cnt++]=(1);//vmc[1].param.invert_knee_epos[Xr]==-1);
	SendBuff_USB[usb_send_cnt++]=(vmc[2].param.invert_knee_epos[Xr]==-1);
    SendBuff_USB[usb_send_cnt++]=(1);//vmc[3].param.invert_knee_epos[Xr]==-1);
	
	setDataFloat(vmc_all.version[0]);
	setDataFloat(vmc_all.version[1]);
	setDataInt(vmc_all.board_id[0]*10000+vmc_all.board_id[1]*100+vmc_all.board_id[2]);
	SendBuff_USB[usb_send_cnt++]=(vmc_all.key_right);
	setDataInt(vmc_all.your_key[0]*10000+vmc_all.your_key[1]*100+vmc_all.your_key[2]);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_sensor_config(void)
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x21;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(mems.imu_pos.x);
	setDataFloat(mems.imu_pos.y);
	setDataFloat(mems.imu_pos.z);
	setDataFloat(mems.imu_att.x);
	setDataFloat(mems.imu_att.y);
	setDataFloat(mems.imu_att.z);
	setDataFloat(mems.gps_pos.x);
	setDataFloat(mems.gps_pos.y);
	setDataFloat(mems.gps_pos.z);
	
  	setDataFloat(0);//mems.Acc_Offset.x/4096.);
	setDataFloat(0);//mems.Acc_Offset.y/4096.);
	setDataFloat(0);//mems.Acc_Offset.z/4096.);
	setDataFloat(1);
	setDataFloat(1);
	setDataFloat(1);
  	setDataFloat(0);//mems.Gyro_Offset.x);
	setDataFloat(0);//mems.Gyro_Offset.y);
	setDataFloat(0);//mems.Gyro_Offset.z);
	setDataFloat(1);
	setDataFloat(1);
	setDataFloat(1);
	setDataFloat(0);//mems.Mag_Offset.x);
	setDataFloat(0);//mems.Mag_Offset.y);
	setDataFloat(0);//mems.Mag_Offset.z);
	setDataFloat(0);//mems.Mag_Gain.x);
	setDataFloat(0);//mems.Mag_Gain.y);
	setDataFloat(0);//mems.Mag_Gain.z);

	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_pwm_config(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x02;
	SendBuff_USB[usb_send_cnt++]=0;

	for(i=0;i<4;i++){
  	setDataInt(0);//(vmc[i].param.PWM_OUT[0]);
	setDataInt(0);//(vmc[i].param.PWM_OUT[1]);
	setDataInt(0);//(vmc[i].param.PWM_OUT[2]);
	}
	for(i=0;i<4;i++){
  	setDataInt(0);//(vmc[i].param.PWM_OFF[0]-1500);
	setDataInt(0);//(vmc[i].param.PWM_OFF[1]-1500);
	setDataInt(0);//(vmc[i].param.PWM_OFF[2]-1500);
	}
	for(i=0;i<4;i++){
		for(j=0;j<3;j++){
			if(vmc[i].param.sita_flag[j]==-1)//转换符号
				SendBuff_USB[usb_send_cnt++]=1;
			else
				SendBuff_USB[usb_send_cnt++]=0;
		}
	}
	setDataFloat(0);//(vmc[0].param.PWM_PER_DEGREE[0]);
	setDataFloat(0);//(vmc[0].param.PWM_PER_DEGREE[1]);
	setDataFloat(0);//(vmc[0].param.PWM_PER_DEGREE[2]);
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_curve_config(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x09;
	SendBuff_USB[usb_send_cnt++]=0;

	for(i=0;i<9;i++)
		setDataFloat(nav_rx.temp_record[i]);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<usb_send_cnt;i++)
		sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_param_config(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x51;
	SendBuff_USB[usb_send_cnt++]=0;

  if(vmc_all.param.send_mask==1)vmc_all.param.send_mask=0;
	setDataFloat(vmc_all.param.param_vmc.pid_pit[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_pit[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_pit[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_rol[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_rol[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_rol[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_yaw[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_yaw[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_yaw[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_vx[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_vx[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_vx[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_vy[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_vy[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_vy[2]);	
	setDataFloat(vmc_all.param.param_vmc.pid_posxy[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_posxy[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_posxy[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_posz[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_posz[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_posz[2]);	
	setDataFloat(vmc_all.param.param_vmc.stance_xy_kp[0]);
	setDataFloat(vmc_all.param.param_vmc.stance_xy_kp[1]);
	setDataFloat(vmc_all.param.param_vmc.stance_zoff_kp);
	setDataFloat(vmc_all.param.param_vmc.stance_time[0]);
	setDataFloat(vmc_all.param.param_vmc.stance_time[1]);
	setDataFloat(vmc_all.param.param_vmc.swing_hight);
	setDataFloat(vmc_all.param.param_vmc.swing_spdkp[0]);
	setDataFloat(vmc_all.param.safe_sita[0]);
	setDataFloat(vmc_all.param.safe_sita[1]);
	setDataFloat(vmc_all.param.safe_sita[2]);		
	setDataFloat(vmc_all.param.param_vmc.slip_p[0]);
	setDataFloat(vmc_all.param.param_vmc.slip_p[1]);
	setDataFloat(vmc_all.param.param_vmc.slip_p[2]);		
	setDataFloat(0);//(vmc_all.param.param_vmc.servo_pd[0]);
	setDataFloat(0);//(vmc_all.param.param_vmc.servo_pd[1]);
	setDataFloat(0);//vmc_all.param.param_vmc.servo_flt);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[0]);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[1]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[PITr]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[ROLr]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[0]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[1]);
	setDataFloat(vmc_all.param.param_vmc.side_off[0]);
	setDataFloat(vmc_all.param.param_vmc.side_off[1]);
	setDataFloat(vmc_all.tar_att_bias[PITr]);
	setDataFloat(vmc_all.tar_att_bias[ROLr]);		

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void UDP_OCU_TX(float dt)//发送
{
	static float timer_param[10]={0};
    static int cnt_usb_send[5]={0};
	static char state=0,i;
	static char state_flag=0,flag1=0;
	int len,t;
	usb_send_cnt=0;

	if(ocu.record.en_record==1)//记录模式
	{
		switch(cnt_usb_send[2]){
		case 0:
		//if(flag1)
		data_per_usb_record1_config(0);
		//else
		//data_per_usb_record1_config(1);
		cnt_usb_send[2]++;
		break;
		case 1:
		//if(flag1)
		//data_per_usb_record1_config(2);
		//else
		data_per_usb_record1_config(2);
		cnt_usb_send[2]++;
		break;
		case 2:
		data_per_usb_record1_config(4);//com
		flag1=!flag1;
		cnt_usb_send[2]=0;	
		break;
		}
	}
	else{//-------------------正常模式---------------
	
		for(i=0;i<10;i++)
			timer_param[i]+=dt;

		if(timer_param[0]>0.5){
			timer_param[0]=0;
			data_per_usb_controller_config();
		}else if(timer_param[1]>0.4){
			timer_param[1]=0;
			data_per_usb_param_config(); 
		}else{
			//printf("cnt_usb_send[2]=%d\n",cnt_usb_send[2]);
			switch(cnt_usb_send[2]){
                case 0:
                data_per_usb_robot_state1();//com
                cnt_usb_send[2]++;
                break;
                case 1:
                data_per_usb_robot_state3();//joint
				cnt_usb_send[2]++;
				break;
                case 2:
                data_per_usb_robot_state4();//tau
                cnt_usb_send[2]=0;
                break;
				default:cnt_usb_send[2]=0;	
				break;
			}
		}
	}
}
