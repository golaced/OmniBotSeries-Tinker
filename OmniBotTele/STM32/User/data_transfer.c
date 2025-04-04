#include "data_transfer.h"
#include "usart.h"
#include "imu.h"
#include "mpu9250.h"
#include "rc_mine.h"
#include "adc.h"
#include "hw_config.h"
dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 checkdata_to_send,checksum_to_send;

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	usbsendData(data_to_send, length);
	
}
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{ u8 sum = 0,i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	
	for( i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}
static void ANO_DT_Send_Msg(u8 id, u8 data)
{ u8 sum = 0,i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	 sum = 0;
	for( i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
extern float ultra_dis_lpf;

float test_up[5];
float off_flow_pos[2]={0};//{-2.5,-2.5};
void ANO_DT_Data_Exchange(void)
{ static float pos_off[2];
	static u8 cnt1;
	static float x,y,z;
	static u8 state_mine;
	static u8 sel[10];
	long temp_j;
	long temp_w;
	#if USE_HT_GROUND 
	#define rate 10
	#else
	#define rate 10
	#endif
	#if USE_HT_GROUND 
	#define rate1 1
	#else
	#define rate1 1
	#endif
	
	static u8 cnt = 0;
	static u8 senser_cnt 	= rate1*10/rate;
	static u8 senser2_cnt = rate1*50/rate;
	static u8 user_cnt 	  = rate1*10/rate;
	static u8 status_cnt 	= rate1*15/rate;
	static u8 rcdata_cnt 	= rate1*20/rate;
	static u8 motopwm_cnt	= rate1*20/rate;
	static u8 power_cnt		=	rate1*50/rate;
	static u8 speed_cnt   = rate1*50/rate;
	static u8 location_cnt   = rate1*200/rate;
	static u8 qr_cnt=rate1*20/rate;
	
	if((cnt % qr_cnt) == (qr_cnt-1))
		f.send_qr = 1;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.send_power = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.send_speed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.send_location += 1;		
	}
	
	
// 	if(height_ctrl_mode!=0){
// 	if(!mode_oldx.flow_hold_position)	
// 	fly_mode=2;//定高
// 	else
// 	fly_mode=3;//定点
// 	}
// 	else
// 	fly_mode=1;//姿态	

	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.msg_id)
	{
		#if !USE_HT_GROUND 
		ANO_DT_Send_Msg(f.msg_id,f.msg_data);
		#else
		#endif
		f.msg_id = 0;
	}
	

// 		if(circle.x!=0)
// 			x=(float)circle.x/100.;
// 		if(circle.y!=0)
// 			y=(float)-circle.y/100.;
// 		if(circle.z!=0)
// 			z=(float)circle.z/100.;
			
/////////////////////////////////////////////////////////////////////////////////////

	if(f.send_check)//for PID
	{
		f.send_check = 0;
		#if !USE_HT_GROUND 
		ANO_DT_Send_Check(checkdata_to_send,checksum_to_send);
		#endif
	}
	else 
	{ 
	switch(state_mine)
	 {
		case 0:		
		 if(sel[0]==0){sel[0]=1;
		 if(cnt1){cnt1=0;	 
		 ANO_DT_Send_Status(plane.att[0],plane.att[1],plane.att[2],(plane.pos[2]*100),plane.mode,plane.lock);	
		 }
		 else{cnt1=1;
			  ANO_DT_Send_Senser( DEBUG[0],DEBUG[1],DEBUG[2],
										       	DEBUG[3],DEBUG[4],DEBUG[5],
								            DEBUG[6],DEBUG[7],DEBUG[8]);
		 }
		 }
		 else if(sel[0]==1){sel[0]=2;
		 
				ANO_DT_Send_Senser( DEBUG[0],DEBUG[1],DEBUG[2],
										       	DEBUG[3],DEBUG[4],DEBUG[5],
								            DEBUG[6],DEBUG[7],DEBUG[8]);

		 }else {sel[0]=0; 
		  ANO_DT_Send_RCData(CH[2]+1500,CH[3]+1500,CH[0]+1500,CH[1]+1500,CH[4]+1500,CH[5]+1500,CH[6]+1500,CH[7]+1500,0 +1500,0 +1500);
		 }
	
		 
		state_mine=1;
		break;
	 case 1:
	
	
		 if(sel[1]==0){sel[1]=1;	 
		 ANO_DT_Send_Speed(plane.spd[0]*1000,plane.spd[1]*1000,plane.spd[2]*1000);
			 }
		 else{sel[1]=0;
	   ANO_DT_Send_QR1((plane.pos[0]-pos_off[0]),(plane.pos[1]-pos_off[1]),plane.pos[2]);
// 			 if(NS==2)
// 			 {
// 				pos_off[0]=plane.pos[0];
// 				pos_off[1]=plane.pos[1];
// 			 }			 
		 }
		 
	   state_mine=2;
	  break;
	 case 2:
		if(sel[2]==0){sel[2]=1;		
			ANO_DT_Send_Senser2(0,plane.pos[2]*100);//原始数据
			}else if(sel[2]==1){sel[2]=2;	 
			ANO_DT_Send_Senser( DEBUG[0],DEBUG[1],DEBUG[2],
										       	DEBUG[3],DEBUG[4],DEBUG[5],
								            DEBUG[6],DEBUG[7],DEBUG[8]);
		 }
	  else {sel[2]=0;

		 //if(!m100.Lat&&!m100.Lon)	
		 {
		 temp_j=(int)(116.39122*10000000)	;
		 temp_w=(int)(39.90736*10000000)	;
		 } 
// 		 else{
// 		 temp_j=(int)(m100.Lon*10000000)	;
// 		 temp_w=(int)(m100.Lat*10000000)	;
// 		 }
		 ANO_DT_Send_Location(plane.state_v,0,temp_j,temp_w,0);
		 ANO_DT_Send_Power(plane.bat, 0);
		}
		if(f.send_pid1){
	  state_mine=3;}
		else
		state_mine=0;	
	 break;
	 case 3: 
		if(f.send_pid1){
		 if(sel[3]==0){sel[3]=1;
		ANO_DT_Send_PID(1,plane.PID_RX[0][0],plane.PID_RX[0][1],plane.PID_RX[0][2],
											plane.PID_RX[1][0],plane.PID_RX[1][1],plane.PID_RX[1][2],
											plane.PID_RX[2][0],plane.PID_RX[2][1],plane.PID_RX[2][2]);}
    else if(sel[3]==1){sel[3]=2;
		ANO_DT_Send_PID(2,plane.PID_RX[3][0],plane.PID_RX[3][1],plane.PID_RX[3][2],
											plane.PID_RX[4][0],plane.PID_RX[4][1],plane.PID_RX[4][2],
											plane.PID_RX[5][0],plane.PID_RX[5][1],plane.PID_RX[5][2]);}
	 else if(sel[3]==2){sel[3]=3;//H spd       H   pos     Pos spd
	  ANO_DT_Send_PID(3,plane.PID_RX[6][0],plane.PID_RX[6][1],plane.PID_RX[6][2],
											plane.PID_RX[7][0],plane.PID_RX[7][1],plane.PID_RX[7][2],
											plane.PID_RX[8][0],plane.PID_RX[8][1],plane.PID_RX[8][2]);
	  }
    else if(sel[3]==3){sel[3]=4;//Pos pos    Pos pos eso Pos spd eso
		ANO_DT_Send_PID(4,plane.PID_RX[9][0],plane.PID_RX[9][1],plane.PID_RX[9][2],
											plane.PID_RX[10][0],plane.PID_RX[10][1],plane.PID_RX[10][2],
											plane.PID_RX[11][0],plane.PID_RX[11][1],plane.PID_RX[11][2]);
		}
		else if(sel[3]==4){sel[3]=5;//					 Pos spd fp  Pos acc
		ANO_DT_Send_PID(5,plane.PID_RX[12][0],plane.PID_RX[12][1],plane.PID_RX[12][2],
											plane.PID_RX[13][0],plane.PID_RX[13][1],plane.PID_RX[13][2],
											plane.PID_RX[14][0],plane.PID_RX[14][1],plane.PID_RX[14][2]);								
		}
		else {sel[3]=0;
		ANO_DT_Send_PID(6,plane.PID_RX[15][0],plane.PID_RX[15][1],plane.PID_RX[15][2],
											plane.PID_RX[16][0],plane.PID_RX[16][1],plane.PID_RX[16][2],
											plane.PID_RX[17][0],plane.PID_RX[17][1],plane.PID_RX[17][2]);
		f.send_pid1=0;
		}
	 }
		if(f.send_pid1==0)
	   state_mine =0;
		break;
	 }	
	}

}



/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 flash_save_en_cnt = 0;
u8 acc_3d_calibrate_f,acc_3d_step;
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{ 
	u8 mode;
	u8 sum = 0,i;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{  mode=*(data_buf+4);
		if(*(data_buf+4)==0X01)
		{
			plane.acc_cal = 1;
		}
		else if(*(data_buf+4)==0X02)
			plane.gyro_cal = 1;
// 		else if(*(data_buf+4)==0X05)
// 			mode_oldx.cal_rc=!mode_oldx.cal_rc;
		else if(*(data_buf+4)==0X03)
		{
			plane.acc_cal = 1;		
			plane.gyro_cal = 1;			
		}
		else if(*(data_buf+4)==0X04)
		{
			plane.mag_cal = 1;
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			if(acc_3d_calibrate_f==0)
			{	acc_3d_calibrate_f=1;acc_3d_step++;}
			else if(acc_3d_calibrate_f==1){
			if(acc_3d_step<6)	
				acc_3d_step++;
			else
				acc_3d_step=0;}
          
		}
		else if(*(data_buf+4)==0X20)
		{
			acc_3d_step = 0; //退出，6面校准步清0
		}
	}
	

	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{ plane.read_pid=1;
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			//Para_ResetToFactorySetup();
		}
	}
	
	if(*(data_buf+2)==0X10)								//PID1 att in
    {  
       plane.PID[0][0]  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       plane.PID[0][1]  =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
       plane.PID[0][2]  =( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
       plane.PID[1][0]  =( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
       plane.PID[1][1]  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       plane.PID[1][2]  =( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       plane.PID[2][0]  =( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
       plane.PID[2][1]  =( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
       plane.PID[2][2]  =( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
    if(*(data_buf+2)==0X11)								//PID2  att out
    {
       plane.PID[3][0]  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       plane.PID[3][1]  =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
       plane.PID[3][2]  =( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
       plane.PID[4][0]  =( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
       plane.PID[4][1]  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       plane.PID[4][2]  =( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       plane.PID[5][0]  =( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
       plane.PID[5][1]  =( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
       plane.PID[5][2]  =( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
    if(*(data_buf+2)==0X12)								//PID3 height
    {	
       plane.PID[6][0]  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       plane.PID[6][1]  =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
       plane.PID[6][2]  =( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
       plane.PID[7][0]  =( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
       plane.PID[7][1]  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       plane.PID[7][2]  =( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       plane.PID[8][0]  =( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
       plane.PID[8][1]  =( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
       plane.PID[8][2]  =( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
	if(*(data_buf+2)==0X13)								//PID4  pos
	{
		   plane.PID[9][0]  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       plane.PID[9][1]  =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
       plane.PID[9][2]  =( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
       plane.PID[10][0]  =( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
       plane.PID[10][1]  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       plane.PID[10][2]  =( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       plane.PID[11][0]  =( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
       plane.PID[11][1]  =( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
       plane.PID[11][2]  =( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
	}	
	if(*(data_buf+2)==0X14)								//PID5 for Pos spd &acc
	{
		   plane.PID[12][0]  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       plane.PID[12][1]  =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
       plane.PID[12][2]  =( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
       plane.PID[13][0]  =( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
       plane.PID[13][1]  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       plane.PID[13][2]  =( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       plane.PID[14][0]  =( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
       plane.PID[14][1]  =( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
       plane.PID[14][2]  =( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
	}
	if(*(data_buf+2)==0X15)								//PID6 for imu set
	{
		   plane.PID[15][0]  = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       plane.PID[15][1]  =( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
       plane.PID[15][2]  =( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
       plane.PID[16][0]  =( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
       plane.PID[16][1]  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
       plane.PID[16][2]  =( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       plane.PID[17][0]  =( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
       plane.PID[17][1]  =( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
       plane.PID[17][2]  =( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		   send_pid=1;
		if(f.send_check == 0)
		{ 
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
	}
	
		if(*(data_buf+2)==0X21)								//way_point 
	{
	
		
		
	}
		if(*(data_buf+2)==0X81)								//DJ_CONTROL
	{
	  //att_test[0]=0.1*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		//att_test[1]=0.1*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		
	}
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

}

void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;
	
	_temp2 = lon;//经度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;//纬度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

}


void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0,i;
	vs16 _temp;
		u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;

	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d  * 1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


extern float yaw_mag,airframe_x_sp,airframe_y_sp,wx_sp,wy_sp;
extern float werr_x_gps,werr_y_gps,aerr_x_gps,aerr_y_gps;

void ANO_DT_Send_User()
{ u8 i;
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0;
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1; //用户数据
	data_to_send[_cnt++]=0;
	
	for(i=0;i<9;i++){
	//_temp = (s16)BLE_DEBUG[i+1];            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  }

	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_Send_QR1(float x,float y,float z)
{ u8 i;
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0;
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x32; //用户数据
	data_to_send[_cnt++]=0;
	
	
	_temp = (s16)(x*100);            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)(y*100);            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)(z*100);            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_QR2(float x,float y,float z)
{ u8 i;
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0;
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x33; //用户数据
	data_to_send[_cnt++]=0;
	
	
	_temp = (s16)(x*100);            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)(y*100);            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (s16)(z*100);            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	data_to_send[3] = _cnt-4;
	
	 sum = 0;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

