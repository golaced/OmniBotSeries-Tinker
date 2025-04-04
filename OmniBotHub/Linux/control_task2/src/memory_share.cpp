
#include <sys/shm.h>
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
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "sys_time.h"
#include "include.h"
#include "gait_math.h"
#include "can.h"
#include "locomotion_header.h"
#include <pthread.h>
#include <qpOASES.hpp>


// ps -ef | grep control_task   ps a全部查看
// ps -ef | grep hardware_task
// kill －9 324  ps aux | grep "MOCO_ML"
// sudo nano /etc/rc.local
#define EN_THREAD_LOCK 1
_SPI_RX spi_rx;
_SPI_TX spi_tx;

_NAV_RX nav_rx;
_NAV_TX nav_tx;
_MEMS mems;

int mem_connect=0;
float mem_loss_cnt=0;
int mem_connect_c=0;
float mem_loss_cnt_c=0;
struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi,shareMemory_control;

unsigned char mem_read_buf[MEM_SIZE];
unsigned char mem_write_buf[MEM_SIZE];
int mem_read_cnt=0;
int mem_write_cnt=0;

unsigned char mem_read_buf_c[MEM_SIZE];
unsigned char mem_write_buf_c[MEM_SIZE];
int mem_read_cnt_c=0;
int mem_write_cnt_c=0;

static void setDataFloat_mem(float f, int *anal_cnt)
{
    int i = *(int *)&f;
    mem_write_buf[*anal_cnt+0] = ((i << 24) >> 24);
    mem_write_buf[*anal_cnt+1] = ((i << 16) >> 24);
    mem_write_buf[*anal_cnt+2] = ((i << 8) >> 24);
    mem_write_buf[*anal_cnt+3] = (i >> 24);

    *anal_cnt += 4;
}

static void setDataFloat_mem_c(float f, int *anal_cnt)
{
    int i = *(int *)&f;
    mem_write_buf_c[*anal_cnt+0] = ((i << 24) >> 24);
    mem_write_buf_c[*anal_cnt+1] = ((i << 16) >> 24);
    mem_write_buf_c[*anal_cnt+2] = ((i << 8) >> 24);
    mem_write_buf_c[*anal_cnt+3] = (i >> 24);

    *anal_cnt += 4;
}

static float floatFromData_spi(unsigned char *data, int *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));

    *anal_cnt += 4;
    return *(float *)&i;
}

static char charFromData_spi(unsigned char *data, int *anal_cnt)
{
    int temp = *anal_cnt;
    *anal_cnt += 1;
    return *(data + temp);
}


void memory_write_c(int st_mem)//写入内存 interface  写入里程计 状态估计 OCU状态
{
    int i;
    static float temp=0;
    mem_write_cnt_c=st_mem;

    nav_rx.att_now[0]=robotwb.now_att.pitch;
    nav_rx.att_now[1]=robotwb.now_att.roll;
    nav_rx.att_now[2]=robotwb.now_att.yaw;

    nav_rx.datt_now[0]=robotwb.now_rate.pitch;
    nav_rx.datt_now[1]=robotwb.now_rate.roll;
    nav_rx.datt_now[2]=robotwb.now_rate.yaw;

    nav_rx.acc_n_now[0]=robotwb.cog_acc_n.x;
    nav_rx.acc_n_now[1]=robotwb.cog_acc_n.y;
    nav_rx.acc_n_now[2]=robotwb.cog_acc_n.z;

    nav_rx.att_tar[0]=robotwb.exp_att.pitch;
    nav_rx.att_tar[1]=robotwb.exp_att.roll;
    nav_rx.att_tar[2]=robotwb.exp_att.yaw;

    nav_rx.com_n_tar[0]=robotwb.exp_pos_n.x;
    nav_rx.com_n_tar[1]=robotwb.exp_pos_n.y;
    nav_rx.com_n_tar[2]=robotwb.exp_pos_n.z;

    nav_rx.com_n_now[0]=vmc_all.pos_n.x;
    nav_rx.com_n_now[1]=vmc_all.pos_n.y;
    nav_rx.com_n_now[2]=vmc_all.pos_n.z;

    nav_rx.dcom_n_tar[0]=robotwb.exp_spd_n.x;
    nav_rx.dcom_n_tar[1]=robotwb.exp_spd_n.y;
    nav_rx.dcom_n_tar[2]=robotwb.exp_spd_n.z;
 if(vmc_all.gait_mode==TROT){
    nav_rx.dcom_n_now[0]=vmc_all.spd_n.x;
    nav_rx.dcom_n_now[1]=vmc_all.spd_n.y;
    nav_rx.dcom_n_now[2]=vmc_all.spd_n.z;
 }else{
     nav_rx.dcom_n_now[0]=0;
     nav_rx.dcom_n_now[1]=0;
     nav_rx.dcom_n_now[2]=0;
  }
    nav_rx.ground_att_now[0]=vmc_all.ground_att_est[0];
    nav_rx.ground_att_now[1]=vmc_all.ground_att_est[1];
    nav_rx.ground_att_now[2]=vmc_all.ground_att_est[2];

    nav_rx.gait_state=vmc_all.param.robot_mode;
    nav_rx.rc_mode=sdk.sdk_mode;
    //--------------------自定义记录

     nav_rx.temp_record[0]=robotwb.Leg[2].est_u_fxy;
     nav_rx.temp_record[1]=robotwb.Leg[2].est_u_fz;

     nav_rx.temp_record[6]= robotwb.Leg[0].espd_norm_b;


//new
for (i=0;i<14;i++)
{
   mem_write_buf_c[mem_write_cnt_c++] = leg_motor_all.connect_motor[i];
   setDataFloat_mem_c( leg_motor_all.q_set[i],&mem_write_cnt_c);
   setDataFloat_mem_c( leg_motor_all.set_t[i],&mem_write_cnt_c);
   setDataFloat_mem_c( leg_motor_all.q_now[i],&mem_write_cnt_c);
   setDataFloat_mem_c( leg_motor_all.t_now[i],&mem_write_cnt_c);
}

setDataFloat_mem_c( nav_rx.com_n_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_tar[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_now[2],&mem_write_cnt_c);

nav_rx.acc_n_now[0]=robotwb.cog_acc_b.x;
nav_rx.acc_n_now[1]=robotwb.cog_acc_b.y;
nav_rx.acc_n_now[2]=robotwb.cog_acc_b.z;
setDataFloat_mem_c( nav_rx.acc_n_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.acc_n_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.acc_n_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.dcom_n_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_tar[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.ground_att_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.ground_att_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.ground_att_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.att_now[0],&mem_write_cnt_c);//-----------
setDataFloat_mem_c( nav_rx.att_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_now[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.att_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_tar[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_tar[2],&mem_write_cnt_c);

mem_write_buf_c[mem_write_cnt_c++] = nav_rx.gait_state;
mem_write_buf_c[mem_write_cnt_c++] = nav_rx.rc_mode;

for (i=0;i<10;i++)
    setDataFloat_mem_c( nav_rx.temp_record[i],&mem_write_cnt_c);
}

//------------------------------内存管理--Hardware----------------------
void memory_read(void){//读取STM32 read hardware 回传传感器和 编码器 力矩
char temp;
mem_read_cnt=0;
spi_rx.att[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_rate[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_rate[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_rate[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_b[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_b[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_b[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_n[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_n[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_n[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);

spi_rx.imu_mems_connect = charFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_usb[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_usb[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_usb[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_rate_usb[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_rate_usb[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.att_rate_usb[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_b_usb[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_b_usb[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
spi_rx.acc_b_usb[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);

if(spi_rx.imu_mems_connect
       //&&vmc_all.side_flip
        ){//&&vmc_all.side_flip){
    for(int i=0;i<3;i++){
        if(fabs( spi_rx.att_usb[0])<180&&fabs( spi_rx.att_usb[1])<180&&fabs( spi_rx.att_usb[2])<360){
            spi_rx.att[i]=spi_rx.att_usb[i];
            spi_rx.att_rate[i]=spi_rx.att_rate_usb[i];
            spi_rx.acc_b[i]=spi_rx.acc_b_usb[i];
        }
    }
}

for (int i = 0; i < 14; i++)
{
    spi_rx.q[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.dq[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.tau[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    temp= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_rx.connect[i] = temp/100%10;
    spi_rx.connect_motor[i]= (temp-spi_rx.connect[i] *100)/10;
    spi_rx.ready[i] = temp%10;
}

for (int i = 0; i < 14; i++)
{
    spi_rx.q_servo[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.tau_servo[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    temp= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_rx.connect_servo[i] = temp;
}
    spi_rx.bat_v = floatFromData_spi(mem_read_buf, &mem_read_cnt);

#if 0
    static int cnt_p=0;
    if(cnt_p++>300){cnt_p=0;
        printf("att :%.1f %.1f %.1f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
        printf("rate:%.1f %.1f %.1f\n",spi_rx.att_rate[0],spi_rx.att_rate[1],spi_rx.att_rate[2]);
        printf("q0 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f \n", spi_rx.q[0], spi_rx.q[1], spi_rx.q[2], spi_rx.q[3], spi_rx.q[4], spi_rx.q[5],spi_rx.q[6]);
        printf("dq0:%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", spi_rx.dq[0], spi_rx.dq[1], spi_rx.dq[2], spi_rx.dq[3], spi_rx.dq[4], spi_rx.dq[5], spi_rx.dq[6]);
        printf("q1 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",  spi_rx.q[7], spi_rx.q[8], spi_rx.q[9], spi_rx.q[10], spi_rx.q[11], spi_rx.q[12], spi_rx.q[13]);
        printf("dq1:%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", spi_rx.dq[7], spi_rx.dq[8], spi_rx.dq[9], spi_rx.dq[10], spi_rx.dq[11], spi_rx.dq[12], spi_rx.dq[13]);
        printf("t0 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", spi_rx.tau[0], spi_rx.tau[1], spi_rx.tau[2], spi_rx.tau[3], spi_rx.tau[4], spi_rx.tau[5], spi_rx.tau[6]);
        printf("t1 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",spi_rx.tau[7], spi_rx.tau[8], spi_rx.tau[9], spi_rx.tau[10], spi_rx.tau[11], spi_rx.tau[12], spi_rx.tau[13]);
    }
#endif
#if 0//servo
    static int cnt_p=0;
    if(cnt_p++>300){cnt_p=0;
        printf("att :%.1f %.1f %.1f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
        printf("rate:%.1f %.1f %.1f\n",spi_rx.att_rate[0],spi_rx.att_rate[1],spi_rx.att_rate[2]);
        printf("q0 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f \n", spi_rx.q_servo[0], spi_rx.q_servo[1], spi_rx.q_servo[2], spi_rx.q_servo[3], spi_rx.q_servo[4], spi_rx.q_servo[5],spi_rx.q_servo[6]);
        printf("dq0:%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", spi_rx.dq[0], spi_rx.dq[1], spi_rx.dq[2], spi_rx.dq[3], spi_rx.dq[4], spi_rx.dq[5], spi_rx.dq[6]);
        printf("q1 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",  spi_rx.q_servo[7], spi_rx.q_servo[8], spi_rx.q_servo[9], spi_rx.q_servo[10], spi_rx.q_servo[11], spi_rx.q_servo[12], spi_rx.q_servo[13]);
        printf("dq1:%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", spi_rx.dq[7], spi_rx.dq[8], spi_rx.dq[9], spi_rx.dq[10], spi_rx.dq[11], spi_rx.dq[12], spi_rx.dq[13]);
        printf("t0 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", spi_rx.tau[0], spi_rx.tau[1], spi_rx.tau[2], spi_rx.tau[3], spi_rx.tau[4], spi_rx.tau[5], spi_rx.tau[6]);
        printf("t1 :%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",spi_rx.tau[7], spi_rx.tau[8], spi_rx.tau[9], spi_rx.tau[10], spi_rx.tau[11], spi_rx.tau[12], spi_rx.tau[13]);
    }
#endif
#if 1//new rc wireless
    spi_rx.ocu.connect= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_st= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_back= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_lr= charFromData_spi(mem_read_buf, &mem_read_cnt);
    if(spi_rx.ocu.key_lr==255)spi_rx.ocu.key_lr=-1;
    spi_rx.ocu.key_ud= charFromData_spi(mem_read_buf, &mem_read_cnt);
    if(spi_rx.ocu.key_ud==255)spi_rx.ocu.key_ud=-1;
    spi_rx.ocu.key_x= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_a= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_b= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_y= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_ll= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.key_rr= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.rc_spd_w[0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.rc_spd_w[1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.rc_att_w[0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.rc_att_w[1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.rate_yaw_w=floatFromData_spi(mem_read_buf, &mem_read_cnt);
#endif

#if 0
printf("connect=%d st=%d back=%d x=%d a=%d\n",
    spi_rx.ocu.connect ,  spi_rx.ocu.key_st,spi_rx.ocu.key_back, spi_rx.ocu.key_x,spi_rx.ocu.key_a );
printf("b=%d y=%d ll=%d rr=%d lr=%d up=%d\n",
    spi_rx.ocu.key_b ,  spi_rx.ocu.key_y,spi_rx.ocu.key_ll, spi_rx.ocu.key_rr,spi_rx.ocu.key_lr ,spi_rx.ocu.key_ud);
printf("spd0=%f spd1=%f att0=%f att1=%f yaw=%f\n",
    spi_rx.ocu.rc_spd_w[0],spi_rx.ocu.rc_spd_w[1],spi_rx.ocu.rc_att_w[0],spi_rx.ocu.rc_att_w[1],spi_rx.ocu.rate_yaw_w);
#endif
//----------------------数据转化--------------------
robotwb.IMU_now_o.pitch=spi_rx.att[0]+vmc_all.att_measure_bias[PITr];
if(vmc_all.side_flip)
    robotwb.IMU_now_o.roll=spi_rx.att[1]+vmc_all.att_measure_bias_flip[ROLr];
else
    robotwb.IMU_now_o.roll=spi_rx.att[1]+vmc_all.att_measure_bias[ROLr];

robotwb.IMU_now_o.yaw=spi_rx.att[2];
robotwb.IMU_dot_o.pitch=spi_rx.att_rate[0];
robotwb.IMU_dot_o.roll=spi_rx.att_rate[1];
robotwb.IMU_dot_o.yaw=spi_rx.att_rate[2];
robotwb.cog_acc_b.x=spi_rx.acc_b[0];
robotwb.cog_acc_b.y=spi_rx.acc_b[1];
robotwb.cog_acc_b.z=spi_rx.acc_b[2];

for (int i = 0; i < 14; i++){
    leg_motor_all.connect=spi_rx.connect[i];//node connect
    leg_motor_all.connect_motor[i]=spi_rx.connect_motor[i];
    leg_motor_all.ready[i]=spi_rx.ready[i];
    leg_motor_all.q_now[i]=spi_rx.q[i];
    leg_motor_all.qd_now[i]=spi_rx.dq[i];
    leg_motor_all.t_now[i]=spi_rx.tau[i];

    arm_motor_all.q_now[i]=spi_rx.q_servo[i];
}
//printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
}

void memory_write(void)//写入Memeory to STM32 期望力 角度和系统参数
{
static float t_temp=0;
static float temp=0;
mem_write_cnt=MEM_SIZE/2;
char id_swap_flig[4]={2,3,0,1};
float q_tar_temp[3]={0};
//-------------------------数据转化------------------------
#if 0//debug
for(int id=0;id<14;id++)
{
    spi_tx.q_set[id]=0;
    spi_tx.q_reset[id]=0;
    spi_tx.tau_ff[id]=0;
    spi_tx.max_i[id]=0;
    spi_tx.kp[id]=0.5;
    spi_tx.ki[id]=0;
    spi_tx.kd[id]=0.1;
    spi_tx.stiff[id]=0.1;
}
#else
for(int id=0;id<14;id++)
{
    spi_tx.q_set[id]=leg_motor_all.q_set[id];
    spi_tx.q_reset[id]=leg_motor_all.q_reset[id];
    spi_tx.tau_ff[id]=leg_motor_all.set_t[id]*mem_connect*vmc_all.param.soft_weight;
    spi_tx.max_i[id]=leg_motor_all.max_t[id];
    spi_tx.kp[id]=(leg_motor_all.kp[id])*vmc_all.param.soft_weight;
    spi_tx.ki[id]=leg_motor_all.ki[id]*vmc_all.param.soft_weight;
    spi_tx.kd[id]=(leg_motor_all.kd[id])*vmc_all.param.soft_weight;
    spi_tx.stiff[id]=leg_motor_all.stiff[id];
    spi_tx.reser_q_div[id]=leg_motor_all.reset_q_div[id];//new doghome
}

for(int id=0;id<14;id++)
{
    spi_tx.q_set_servo[id]=leg_motor_all.q_set_servo_out[id];
    spi_tx.q_reset_servo[id]=leg_motor_all.q_reset_servo[id];
    spi_tx.stiff_servo[id]=leg_motor_all.stiff_servo[id];
    spi_tx.reser_q_div_servo[id]=leg_motor_all.reset_q_div_servo[id];//new doghome
    spi_tx.tau_ff_servo[id]=leg_motor_all.t_set_servo[id]*mem_connect*vmc_all.param.soft_weight;
}

#endif
    spi_tx.en_motor=leg_motor_all.motor_en;
    spi_tx.en_servo=leg_motor_all.servo_en;
    spi_tx.reser_q=leg_motor_all.reset_q;
    spi_tx.reset_err=leg_motor_all.reset_err;

    if(ocu.esp32_connect==1){
        if(leg_motor_all.reset_imu==1)
            mems.Acc_CALIBRATE=1;
        else if(leg_motor_all.reset_imu==2)
            mems.Gyro_CALIBRATE=1;
        else if(leg_motor_all.reset_imu==4)
            mems.Mag_CALIBRATE=1;
        else
            mems.Acc_CALIBRATE=mems.Gyro_CALIBRATE=mems.Mag_CALIBRATE=0;
    }else{
        if(ocu.sbus_cal_mems==1)
            mems.Acc_CALIBRATE=mems.Gyro_CALIBRATE=1;
    }

//------------------------写入内存-------------------------------------

    mem_write_buf[mem_write_cnt++] = spi_tx.en_motor;
    mem_write_buf[mem_write_cnt++] = spi_tx.en_servo;
    mem_write_buf[mem_write_cnt++] = spi_tx.reser_q;//reset all
    mem_write_buf[mem_write_cnt++] = spi_tx.reset_err;
    mem_write_buf[mem_write_cnt++] = mems.Acc_CALIBRATE;
    mem_write_buf[mem_write_cnt++] = mems.Gyro_CALIBRATE;
    mem_write_buf[mem_write_cnt++] = mems.Mag_CALIBRATE;
    mem_write_buf[mem_write_cnt++] = spi_tx.beep_state= robotwb.beep_state;

    for (int i = 0; i < 14; i++)
    {
        setDataFloat_mem( spi_tx.q_set[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.q_reset[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.tau_ff[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.kp[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.kd[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.stiff[i],&mem_write_cnt);
        mem_write_buf[mem_write_cnt++] = spi_tx.reser_q_div[i];//reset div new doghome
        //printf("id=%d set_q=%f kp=%f kd=%f stiif=%f\n",i,spi_tx.q_set[i],spi_tx.kp[i],spi_tx.kd[i],spi_tx.stiff[i]);
    }
    //printf("bldc:%d %d %d\n",spi_tx.reser_q_div[0],spi_tx.reser_q_div[1],spi_tx.reser_q_div[2]);
    for (int i = 0; i < 14; i++)
    {
        setDataFloat_mem( spi_tx.q_set_servo[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.q_reset_servo[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.tau_ff_servo[i],&mem_write_cnt);
        setDataFloat_mem( spi_tx.kp_servo[i],&mem_write_cnt);//unuse now 0303
        setDataFloat_mem( spi_tx.kd_servo[i],&mem_write_cnt);//unuse now 0303
        setDataFloat_mem( spi_tx.stiff_servo[i],&mem_write_cnt);
        mem_write_buf[mem_write_cnt++] = spi_tx.reser_q_div_servo[i];//reset div new doghome
        //printf("id=%d kp=%f kd=%f stiif=%f\n",i,spi_tx.kp[i],spi_tx.kd[i],spi_tx.stiff[i]);
    }
}

//-------------------------内存管理-OCU----------------------------
void memory_read_c(int mem_st){//读取内存 OCU  读取遥控  模式等等 from UDP thread
char temp;
char i=0;
mem_read_cnt_c=mem_st;

nav_tx.connect = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.ocu_mode = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.rc_spd_b[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_spd_b[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_spd_b[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;

nav_tx.rc_rate_b[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_rate_b[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_rate_b[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;

nav_tx.key_ud = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_lr = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_x = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_y = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_a = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_b = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_ll = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_rr = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_st = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_back = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
static int cnt_p;

//SDK-------外部直接力矩控制接口
nav_tx.request_gait = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_spd_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_spd_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_spd_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_att_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_att_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_att_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_datt_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_datt_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_datt_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_pos_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_pos_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_pos_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

for(i=0;i<4;i++){
    nav_tx.exp_q_o[i][0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_q_o[i][1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_q_o[i][2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_tx.exp_GRF_o[i].x = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_GRF_o[i].y = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_GRF_o[i].z = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
}

//-----------------------OCU param-------------------
mems.imu_pos.x= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_pos.y= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_pos.z= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_att.x= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_att.y= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_att.z= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.gps_pos.x= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.gps_pos.y= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.gps_pos.z= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.Acc_CALIBRATE=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
mems.Gyro_CALIBRATE=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
mems.Mag_CALIBRATE=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;

//-----------------------数据转化---------------------
if(!ocu.sbus_conncect&&!spi_rx.ocu.connect_lock&&!ocu.esp32_connect){
    ocu.connect=nav_tx.connect;
    ocu.mode=nav_tx.ocu_mode;
#if 0
    ocu.rc_spd_w[Xr]=-nav_tx.rc_spd_b[0];
    ocu.rc_spd_w[Yr]=-nav_tx.rc_spd_b[1];
    ocu.rc_att_w[PITr]=-nav_tx.rc_rate_b[0];
    ocu.rc_att_w[ROLr]=-nav_tx.rc_rate_b[1];
    ocu.rate_yaw_w=nav_tx.rc_rate_b[2];
#else
    ocu.rc_spd_w[Xr]=nav_tx.rc_spd_b[0];
    ocu.rc_spd_w[Yr]=0;//nav_tx.rc_spd_b[1];
    ocu.rc_att_w[PITr]=nav_tx.rc_rate_b[0];
    ocu.rc_att_w[ROLr]=nav_tx.rc_rate_b[1];
    ocu.rate_yaw_w=-nav_tx.rc_spd_b[1];//-nav_tx.rc_rate_b[2];
#if 0
    printf("rc_spd_b:=%f %f %f\n",nav_tx.rc_spd_b[0],nav_tx.rc_spd_b[1],nav_tx.rc_spd_b[2]);
    printf("rc_rate_b:=%f %f %f\n",nav_tx.rc_rate_b[0],nav_tx.rc_rate_b[1],nav_tx.rc_rate_b[2]);
#endif
#endif
    ocu.key_st=nav_tx.key_st;
    ocu.key_back=nav_tx.key_back;
    ocu.key_lr=nav_tx.key_lr;
    ocu.key_ud=nav_tx.key_ud;
    ocu.key_x=nav_tx.key_x;
    ocu.key_y=nav_tx.key_y;
    ocu.key_b=nav_tx.key_b;
    ocu.key_a=nav_tx.key_a;
    ocu.key_ll=nav_tx.key_ll;
    ocu.key_rr=nav_tx.key_rr;
}else
    ocu.connect=ocu.sbus_conncect;
//nav_tx.request_gait	=		0;//ocu.up_mode;
ocu.rc_spd_b[Xr]=sdk.cmd_vx=nav_tx.exp_spd_o[0];
ocu.rc_spd_b[Yr]=sdk.cmd_vy=nav_tx.exp_spd_o[1];
ocu.rc_spd_b[Zr]=sdk.cmd_vz=nav_tx.exp_spd_o[2];

ocu.rc_rate_b[PITr]=nav_tx.exp_att_o[0];
ocu.rc_rate_b[ROLr]=nav_tx.exp_att_o[1];
ocu.rc_rate_b[YAWr]=sdk.cmd_vyaw=nav_tx.exp_datt_o[2];


if(spi_rx.ocu.connect||spi_rx.ocu.sbus_conncect){
    if(spi_rx.ocu.connect){//spi_rx.ocu.mode==RC_REMOTE){
        ocu.connect=spi_rx.ocu.connect;
        spi_rx.ocu.connect_lock=1;
        ocu.mode=2;
        ocu.rc_spd_w[Xr]=spi_rx.ocu.rc_spd_w[0];
        ocu.rc_spd_w[Yr]=spi_rx.ocu.rc_spd_w[1];
        ocu.rc_att_w[PITr]=-spi_rx.ocu.rc_att_w[0];
        ocu.rc_att_w[ROLr]= -spi_rx.ocu.rc_att_w[1];
        ocu.rate_yaw_w= dead(spi_rx.ocu.rate_yaw_w*2.3,0.1);
        ocu.key_st=spi_rx.ocu.key_st;
        ocu.key_back=spi_rx.ocu.key_back;
        ocu.key_lr=spi_rx.ocu.key_lr;
        ocu.key_ud=spi_rx.ocu.key_ud;
        ocu.key_x=spi_rx.ocu.key_x;
        ocu.key_y=spi_rx.ocu.key_y;
        ocu.key_b=spi_rx.ocu.key_b;
        ocu.key_a= spi_rx.ocu.key_a;
        ocu.key_ll= spi_rx.ocu.key_ll;
        ocu.key_rr=spi_rx.ocu.key_rr;
    }
}
#if 0
printf("connect=%d st=%d back=%d x=%d a=%d\n",
    spi_rx.ocu.connect ,  spi_rx.ocu.key_st,spi_rx.ocu.key_back, spi_rx.ocu.key_x,spi_rx.ocu.key_a );
printf("b=%d y=%d ll=%d rr=%d lr=%d up=%d\n",
    spi_rx.ocu.key_b ,  spi_rx.ocu.key_y,spi_rx.ocu.key_ll, spi_rx.ocu.key_rr,spi_rx.ocu.key_lr ,spi_rx.ocu.key_ud);
printf("spd0=%f spd1=%f att0=%f att1=%f yaw=%f\n",
    spi_rx.ocu.rc_spd_w[0],spi_rx.ocu.rc_spd_w[1],spi_rx.ocu.rc_att_w[0],spi_rx.ocu.rc_att_w[1],spi_rx.ocu.rate_yaw_w);
#endif
sdk.cmd_pit=nav_tx.exp_att_o[0];
sdk.cmd_rol=nav_tx.exp_att_o[1];
sdk.cmd_yaw=nav_tx.exp_att_o[2];

sdk.cmd_x=nav_tx.exp_pos_o[0];
sdk.cmd_y=nav_tx.exp_pos_o[1];
sdk.cmd_z=nav_tx.exp_pos_o[2];
}

void* Thread_Mem_Servo(void*)//内存管理线程
{
    float timer[5]={0};
    float sys_dt = 0,dT=0;
    static int mem_init_cnt=0;
    int i=0;
    int link_cnt=0;
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功
    void *shm_rx = shmat(shmid_rx, 0, 0);
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    printf("Control::Memory Hardware attached at %p\n",shm_rx);

    while(1)
    {
       sys_dt = Get_Cycle_T(0);
       timer[0]+=sys_dt;

       //共享内存读取 to Servo task
        if(pshm_rx->flag == 1)
        {
            if(!mem_connect){
            mem_init_cnt++;
            if(mem_init_cnt>2){
                printf("Control::Memory Hardware Link=%d!!!\n",link_cnt++);
                mem_connect=1;
            }
            }
            mem_loss_cnt=0;
#if EN_THREAD_LOCK
            pthread_mutex_lock(&lock);
#endif
            for(int k=0;k<MEM_SIZE/2-1;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];

            memory_read();
            memory_write();
#if EN_THREAD_LOCK
            pthread_mutex_unlock(&lock);
#endif
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];

            pshm_rx->flag = 0;
        }

        mem_loss_cnt+=sys_dt;
        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 14; i++){
                spi_tx.q_set[i] = spi_rx.q[i];
                spi_tx.tau_ff[i] = 0;
                spi_tx.kp[i]= spi_tx.ki[i]= spi_tx.kd[i]= spi_tx.en_motor= 0;
            }
            printf("Control::Memery Hardware Loss!!!\n");
        }
        //printf("%f\n",mem_loss_cnt);
        usleep(500);
    }

    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;
}


void* Thread_Mem_Navigation(void*)//内存管理线程
{
    float timer[5]={0};
    float sys_dt = 0,dT=0;
    static int  mem_init_cnt=0;
    int i=0;
    int link_cnt=0;
    int shmid_rx_c = shmget((key_t)MEM_CONTROL, sizeof(shareMemory_control), 0666|IPC_CREAT); //失败返回-1，假设成功
    void *shm_rx_c = shmat(shmid_rx_c, 0, 0);
    shareMemory *pshm_rx_c = (shareMemory*)shm_rx_c;
    pshm_rx_c->flag = 0;
    printf("Control::Memory Control attached at %p\n",shm_rx_c);

    while(1)
    {
       sys_dt = Get_Cycle_T(1);
       timer[0]+=sys_dt;

         //共享内存读取 to Nav task
        if(pshm_rx_c->flag == 0)
        {
            if(!mem_connect_c){
            mem_init_cnt++;
                if(mem_init_cnt>2){
                printf("Control::Memory Navigaition Link=%d!!!\n",link_cnt++);
                mem_connect_c=1;
                }
            }
            mem_loss_cnt_c=0;
            memory_write_c(0);
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx_c->szMsg[k]=mem_write_buf_c[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf_c[k]=pshm_rx_c->szMsg[k];
            memory_read_c(MEM_SIZE/2);
            pshm_rx_c->flag = 1;
        }else
            mem_loss_cnt_c+=sys_dt;

        if(mem_loss_cnt_c>1&&mem_connect_c==1){
            mem_connect_c=0;
            mem_loss_cnt_c=0;
            mem_init_cnt=0;
            printf("Control::Memory Navigaition Loss!!!\n");
        }
        usleep(1000*10);
    }

    shmdt(shm_rx_c);  //失败返回-1，假设成功
    shmctl(shmid_rx_c, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;
}
