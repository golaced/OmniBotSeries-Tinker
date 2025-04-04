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
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include "sys_time.h"
#include "mem_node.h"
#include "base_struct.h"
#include "udp_ocu.h"
#include "comm.h"
#include "can.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

YAML::Node config_hardware=YAML::LoadFile("/home/odroid/Tinker/Param/param_hardware.yaml");
//  ps -ef | grep navigation_task
// kill －9 324
#define SERV_PORT       8889
#define SERV_PORT_ARM   9000
#define SERV_PORT_SDK   8888
#define SERV_PORT_MAP   6666

#define MEM_CONTROL 4999
#define MEM_SIZE 2048

#define EN_MAP 0
#define DEMO_KEY 0
_NAV_RX nav_rx;
_NAV_TX nav_tx;

int mem_connect_c=0;
float mem_loss_cnt_c=0;

struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_control;
unsigned char mem_read_buf_c[MEM_SIZE];
unsigned char mem_write_buf_c[MEM_SIZE];
int mem_read_cnt_c=0;
int mem_write_cnt_c=0;

pthread_mutex_t lock;

_OCU ocu;
VMC vmc[4];
VMC_ALL vmc_all;
_LEG_MOTOR leg_motor[4];
_SDK moco_sdk;

void Perror(const char *s)
{
    perror(s);
    exit(EXIT_FAILURE);
}

static void setnonblocking(int sockfd) {
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        Perror("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        Perror("fcntl F_SETFL fail");
    }
}

struct gait_msg
{
   char gait_mode;
   float att_cmd[3];
   float rate_cmd[3];
   float speed_cmd[3];
   float pos_cmd[3];
} _gait_msg;

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

//--------------------------内存管理--------------------------------
void memory_write_c(int mem_st){//写入  遥控  模式 和 OCU操控 to control task
    char temp;
    char i=0;
    //数据转化

        nav_tx.connect	= ocu.connect;
        nav_tx.ocu_mode	= ocu.mode;
        nav_tx.rc_spd_b[0]=			ocu.rc_spd_w[Xr];
        nav_tx.rc_spd_b[1]=			ocu.rc_spd_w[Yr];
        nav_tx.rc_rate_b[0]	=		ocu.rc_att_w[PITr];
        nav_tx.rc_rate_b[1]	=		ocu.rc_att_w[ROLr];
        nav_tx.rc_rate_b[2]	=		ocu.rate_yaw_w;
        nav_tx.key_st=			ocu.key_st;
        nav_tx.key_back=		ocu.key_back;
        nav_tx.key_lr=			ocu.key_lr;
        nav_tx.key_ud=			ocu.key_ud;
        nav_tx.key_x=			ocu.key_x;
        nav_tx.key_y=			ocu.key_y;
        nav_tx.key_b=			ocu.key_b;
        nav_tx.key_a=			ocu.key_a;
        nav_tx.key_ll=			ocu.key_ll;
        nav_tx.key_rr=			ocu.key_rr;
//        printf("rc_spd_b:=%f %f %f\n",nav_tx.rc_spd_b[0],nav_tx.rc_spd_b[1],nav_tx.rc_spd_b[2]);
//        printf("rc_rate_b:=%f %f %f\n",nav_tx.rc_rate_b[0],nav_tx.rc_rate_b[1],nav_tx.rc_rate_b[2]);
    if(moco_sdk.ros_connect){//ROS
        nav_tx.request_gait	=		moco_sdk.gait_mode;
        nav_tx.exp_spd_o[0]	=		moco_sdk.rc_spd_cmd[Xr];
        nav_tx.exp_spd_o[1]	=		moco_sdk.rc_spd_cmd[Yr];
        nav_tx.exp_spd_o[2]	=		moco_sdk.rc_spd_cmd[Zr];
        nav_tx.exp_att_o[0]	=		moco_sdk.rc_att_cmd[PITr];
        nav_tx.exp_att_o[1]	=		moco_sdk.rc_att_cmd[ROLr];
        nav_tx.exp_att_o[2]	=		moco_sdk.rc_att_cmd[YAWr];
        nav_tx.exp_datt_o[0]	=   moco_sdk.rc_att_rate_cmd[PITr];
        nav_tx.exp_datt_o[1]	=	moco_sdk.rc_att_rate_cmd[ROLr];
        nav_tx.exp_datt_o[2]	=	moco_sdk.rc_att_rate_cmd[YAWr];
        nav_tx.exp_pos_o[0] =		moco_sdk.rc_pos_cmd[Xr];
        nav_tx.exp_pos_o[1] =		moco_sdk.rc_pos_cmd[Yr];
        nav_tx.exp_pos_o[2] =		moco_sdk.rc_pos_cmd[Zr];
    }else{
        nav_tx.request_gait	=		99;
        nav_tx.exp_spd_o[0]	=		0;
        nav_tx.exp_spd_o[1]	=		0;
        nav_tx.exp_spd_o[2]	=		0;
        nav_tx.exp_att_o[0]	=		0;
        nav_tx.exp_att_o[1]	=		0;
        nav_tx.exp_att_o[2]	=		0;
    }
    //----------------------写入内存-----------------
    mem_write_cnt_c=mem_st;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.connect;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.ocu_mode;

    setDataFloat_mem_c( nav_tx.rc_spd_b[0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.rc_spd_b[1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.rc_spd_b[2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_tx.rc_rate_b[0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.rc_rate_b[1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.rc_rate_b[2],&mem_write_cnt_c);

    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_ud+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_lr+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_x+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_y+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_a+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_b+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_ll+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_rr+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_st+1;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.key_back+1;
    //SDK
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.request_gait;

    setDataFloat_mem_c( nav_tx.exp_spd_o[0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_spd_o[1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_spd_o[2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_tx.exp_att_o[0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_att_o[1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_att_o[2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_tx.exp_datt_o[0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_datt_o[1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_datt_o[2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_tx.exp_pos_o[0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_pos_o[1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_tx.exp_pos_o[2],&mem_write_cnt_c);

    for(i=0;i<4;i++){
        setDataFloat_mem_c( nav_tx.exp_q_o[i][0],&mem_write_cnt_c);
        setDataFloat_mem_c( nav_tx.exp_q_o[i][1],&mem_write_cnt_c);
        setDataFloat_mem_c( nav_tx.exp_q_o[i][2],&mem_write_cnt_c);

        setDataFloat_mem_c( nav_tx.exp_GRF_o[i].x,&mem_write_cnt_c);
        setDataFloat_mem_c( nav_tx.exp_GRF_o[i].y,&mem_write_cnt_c);
        setDataFloat_mem_c( nav_tx.exp_GRF_o[i].z,&mem_write_cnt_c);
    }
    //--------------------Param From OCU-------------
    setDataFloat_mem_c( mems.imu_pos.x,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.imu_pos.y,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.imu_pos.z,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.imu_att.x,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.imu_att.y,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.imu_att.z,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.gps_pos.x,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.gps_pos.y,&mem_write_cnt_c);
    setDataFloat_mem_c( mems.gps_pos.z,&mem_write_cnt_c);
    mem_write_buf_c[mem_write_cnt_c++] = mems.Acc_CALIBRATE;
    mem_write_buf_c[mem_write_cnt_c++] = mems.Gyro_CALIBRATE;
    mem_write_buf_c[mem_write_cnt_c++] = mems.Mag_CALIBRATE;

    //-----------------------ARM-----------------
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.arm_ocu.power;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.arm_ocu.arm_mode;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.arm_ocu.control_mode;
    mem_write_buf_c[mem_write_cnt_c++] = nav_tx.arm_ocu.cap;
    setDataFloat_mem_c(  nav_tx.arm_ocu.pos_set_n.x,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.pos_set_n.y,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.pos_set_n.z,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.spd_set_n.x,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.spd_set_n.y,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.spd_set_n.z,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.att_set_n.x,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.att_set_n.y,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.att_set_n.z,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.rate_set_n.x,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.rate_set_n.y,&mem_write_cnt_c);
    setDataFloat_mem_c(  nav_tx.arm_ocu.rate_set_n.z,&mem_write_cnt_c);

}

void memory_read_c(int mem_st)//读取 control task memory
{
    int i;
    uint8_t temp_u8;
    static float temp=0;
    mem_read_cnt_c=mem_st;

    for (i=0;i<14;i++)
    {
        nav_rx.motor_connect_all[i] = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
        nav_rx.q_now[i] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
        nav_rx.t_now[i] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
        nav_rx.q_set[i] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
        nav_rx.t_set[i] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    }
    #if 0
        printf("q0:%.2f %.2f %.2f %.2f %.2f %.2f\n", nav_rx.q_now[0], nav_rx.q_now[1], nav_rx.q_now[2], nav_rx.q_now[3], nav_rx.q_now[4], nav_rx.q_now[5]);
        printf("q1:%.2f %.2f %.2f %.2f %.2f %.2f\n", nav_rx.q_now[6], nav_rx.q_now[7], nav_rx.q_now[8], nav_rx.q_now[9], nav_rx.q_now[10], nav_rx.q_now[11]);
    #endif

    //-------------Cog State
    nav_rx.com_n_tar[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.com_n_tar[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.com_n_tar[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.com_n_now[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.com_n_now[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.com_n_now[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.acc_n_now[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.acc_n_now[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.acc_n_now[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_rx.dcom_n_tar[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.dcom_n_tar[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.dcom_n_tar[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.dcom_n_now[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.dcom_n_now[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.dcom_n_now[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_rx.ground_att_now[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.ground_att_now[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.ground_att_now[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_rx.att_now[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.att_now[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.att_now[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    //printf("%f %f %f\n",nav_rx.att_now[0] ,nav_rx.att_now[1] ,nav_rx.att_now[2] );
    nav_rx.datt_now[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.datt_now[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.datt_now[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_rx.att_tar[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.att_tar[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.att_tar[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.datt_tar[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.datt_tar[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_rx.datt_tar[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_rx.gait_state = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);//Gait State
    nav_rx.rc_mode = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);//
    for (i=0;i<10;i++)
        nav_rx.temp_record[i] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

}

void* Thread_Mem(void*)//内存管理线程
{
    static int cnt = 0;
    float sys_dt = 0;
    float mem_cal_flag_t[3]={0};
    int flag = 0;
    float timer[5]={0};
    int i=0;
    int link_cnt=0;
    int shmid_rx_c = shmget((key_t)MEM_CONTROL, sizeof(shareMemory_control), 0666|IPC_CREAT); //失败返回-1，假设成功
    void *shm_rx_c = shmat(shmid_rx_c, 0, 0);
    shareMemory *pshm_rx_c = (shareMemory*)shm_rx_c;
    printf("Navigation::Memory Navigation attached at %p\n",shm_rx_c);
    while (1)
    {
        sys_dt = Get_Cycle_T(0);
        timer[0]+=sys_dt;

        if(ocu.loss_cnt++>1.0/sys_dt&&ocu.connect){
            ocu.connect=0;
            printf("Navigation::UDP OCU Loss\n");
        }
         //共享内存读取 to Nav task
        if(pshm_rx_c->flag == 1)
        {
            if(!mem_connect_c){
            printf("Navigation::Memory Control Link=%d!!!\n",link_cnt++);
            mem_connect_c=1;
            }
            mem_loss_cnt_c=0;

            for(int k=0;k<MEM_SIZE/2-1;k++)
                mem_read_buf_c[k]=pshm_rx_c->szMsg[k];
#if 1
            pthread_mutex_lock(&lock);
#endif
            memory_read_c(0);
            memory_write_c(MEM_SIZE/2);
#if 1
            pthread_mutex_unlock(&lock);
#endif
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                pshm_rx_c->szMsg[k]=mem_write_buf_c[k];

            pshm_rx_c->flag = 0;
        }

        mem_loss_cnt_c+=sys_dt;
        if(mem_loss_cnt_c>1&&mem_connect_c==1){
            mem_connect_c=0;
            mem_loss_cnt_c=0;
            printf("Navigation::Memory Control Loss!!!\n");
        }

        if(mems.Acc_CALIBRATE)
            mem_cal_flag_t[0]+=sys_dt;
        else
            mem_cal_flag_t[0]=0;
        if(mem_cal_flag_t[0]>2)
            mems.Acc_CALIBRATE=0;
        if(mems.Gyro_CALIBRATE)
            mem_cal_flag_t[1]+=sys_dt;
        else
            mem_cal_flag_t[1]=0;
        if(mem_cal_flag_t[1]>2)
            mems.Gyro_CALIBRATE=0;
        if(mems.Mag_CALIBRATE)
            mem_cal_flag_t[2]+=sys_dt;
        else
            mem_cal_flag_t[2]=0;
        if(mem_cal_flag_t[2]>2)
            mems.Mag_CALIBRATE=0;
        usleep(1000*5);
    }
    shmdt(shm_rx_c);  //失败返回-1，假设成功
    shmctl(shmid_rx_c, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;
}


void param_init(void)
{
    //读取Ymal
    //mems.imu_att.z=90;//imu fix rotation
    mems.imu_att.x=config_hardware["mems_param"]["imu_set_att"][0].as<float>();//
    mems.imu_att.y=config_hardware["mems_param"]["imu_set_att"][1].as<float>();//
    mems.imu_att.z=config_hardware["mems_param"]["imu_set_att"][2].as<float>();//
    printf("IMU_SET=%f %f %f\n", mems.imu_att.x, mems.imu_att.y, mems.imu_att.z);
}

FILE *fp1, *fp2, *fp3;
#define LEN_RECORD 120//记录位置
float data_record1[LEN_RECORD], data_record2[LEN_RECORD], data_record3[LEN_RECORD];

void record_thread(char sel, float dt)
{
    int cnt = 0;
    switch (sel) {
    case 0:
        cnt = 0;
        data_record1[cnt++] = nav_rx.att_tar[0];
        data_record1[cnt++] = nav_rx.att_now[0];

        data_record1[cnt++] = nav_rx.att_tar[1];
        data_record1[cnt++] = nav_rx.att_now[1];

        data_record1[cnt++] = nav_rx.att_tar[2];
        data_record1[cnt++] = nav_rx.att_now[2];

        data_record1[cnt++] = nav_rx.datt_now[0];
        data_record1[cnt++] = nav_rx.datt_now[1];
        data_record1[cnt++] = nav_rx.datt_now[2];

        data_record1[cnt++] = nav_rx.com_n_tar[0];
        data_record1[cnt++] = nav_rx.com_n_now[0];
        data_record1[cnt++] = nav_rx.dcom_n_now[0];

        data_record1[cnt++] = nav_rx.com_n_tar[1];
        data_record1[cnt++] = nav_rx.com_n_now[1];
        data_record1[cnt++] = nav_rx.dcom_n_now[1];

        data_record1[cnt++] = nav_rx.com_n_tar[2];
        data_record1[cnt++] = nav_rx.com_n_now[2];
        data_record1[cnt++] = nav_rx.dcom_n_now[2];

        data_record1[cnt++] = nav_rx.ground_att_now[0];
        data_record1[cnt++] = nav_rx.ground_att_now[1];

        data_record1[cnt++] = nav_rx.dcom_n_tar[0];
        data_record1[cnt++] = nav_rx.dcom_n_tar[1];
        data_record1[cnt++] = nav_rx.dcom_n_tar[2];

        cnt = 0;
        data_record2[cnt++] = nav_rx.GRF_n_tar[0].x;
        data_record2[cnt++] = nav_rx.GRF_n_now[0].x;
        data_record2[cnt++] = nav_rx.GRF_n_tar[0].y;
        data_record2[cnt++] = nav_rx.GRF_n_now[0].y;
        data_record2[cnt++] = nav_rx.GRF_n_tar[0].z;
        data_record2[cnt++] = nav_rx.GRF_n_now[0].z;

        data_record2[cnt++] = nav_rx.GRF_n_tar[1].x;
        data_record2[cnt++] = nav_rx.GRF_n_now[1].x;
        data_record2[cnt++] = nav_rx.GRF_n_tar[1].y;
        data_record2[cnt++] = nav_rx.GRF_n_now[1].y;
        data_record2[cnt++] = nav_rx.GRF_n_tar[1].z;
        data_record2[cnt++] = nav_rx.GRF_n_now[1].z;

        data_record2[cnt++] = nav_rx.GRF_n_tar[2].x;
        data_record2[cnt++] = nav_rx.GRF_n_now[2].x;
        data_record2[cnt++] = nav_rx.GRF_n_tar[2].y;
        data_record2[cnt++] = nav_rx.GRF_n_now[2].y;
        data_record2[cnt++] = nav_rx.GRF_n_tar[2].z;
        data_record2[cnt++] = nav_rx.GRF_n_now[2].z;

        data_record2[cnt++] = nav_rx.GRF_n_tar[3].x;
        data_record2[cnt++] = nav_rx.GRF_n_now[3].x;
        data_record2[cnt++] = nav_rx.GRF_n_tar[3].y;
        data_record2[cnt++] = nav_rx.GRF_n_now[3].y;
        data_record2[cnt++] = nav_rx.GRF_n_tar[3].z;
        data_record2[cnt++] = nav_rx.GRF_n_now[3].z;

        data_record2[cnt++] = nav_rx.is_ground[0];
        data_record2[cnt++] = nav_rx.is_touch[0];
        data_record2[cnt++] = nav_rx.is_ground[1];
        data_record2[cnt++] = nav_rx.is_touch[1];
        data_record2[cnt++] = nav_rx.is_ground[2];
        data_record2[cnt++] = nav_rx.is_touch[2];
        data_record2[cnt++] = nav_rx.is_ground[3];
        data_record2[cnt++] = nav_rx.is_touch[3];

        data_record2[cnt++] = nav_rx.leg_state[0] ;
        data_record2[cnt++] = nav_rx.leg_state[1] ;
        data_record2[cnt++] = nav_rx.leg_state[2] ;
        data_record2[cnt++] = nav_rx.leg_state[3] ;

        data_record2[cnt++] = nav_rx.sita_tar[0][0];
        data_record2[cnt++] = nav_rx.sita_tar[0][1];
        data_record2[cnt++] = nav_rx.sita_tar[0][2];
        data_record2[cnt++] = nav_rx.sita_now[0][0];
        data_record2[cnt++] = nav_rx.sita_now[0][1];
        data_record2[cnt++] = nav_rx.sita_now[0][2];
        data_record2[cnt++] = nav_rx.epos_n_tar[0].x;
        data_record2[cnt++] = nav_rx.epos_n_tar[0].y;
        data_record2[cnt++] = nav_rx.epos_n_tar[0].z;
        data_record2[cnt++] = nav_rx.epos_n_now[0].x;
        data_record2[cnt++] = nav_rx.epos_n_now[0].y;
        data_record2[cnt++] = nav_rx.epos_n_now[0].z;
        data_record2[cnt++] = nav_rx.sita_tar[1][0];
        data_record2[cnt++] = nav_rx.sita_tar[1][1];
        data_record2[cnt++] = nav_rx.sita_tar[1][2];
        data_record2[cnt++] = nav_rx.sita_now[1][0];
        data_record2[cnt++] = nav_rx.sita_now[1][1];
        data_record2[cnt++] = nav_rx.sita_now[1][2];
        data_record2[cnt++] = nav_rx.epos_n_tar[1].x;
        data_record2[cnt++] = nav_rx.epos_n_tar[1].y;
        data_record2[cnt++] = nav_rx.epos_n_tar[1].z;
        data_record2[cnt++] = nav_rx.epos_n_now[1].x;
        data_record2[cnt++] = nav_rx.epos_n_now[1].y;
        data_record2[cnt++] = nav_rx.epos_n_now[1].z;

        data_record2[cnt++] = nav_rx.sita_tar[2][0];
        data_record2[cnt++] = nav_rx.sita_tar[2][1];
        data_record2[cnt++] = nav_rx.sita_tar[2][2];
        data_record2[cnt++] = nav_rx.sita_now[2][0];
        data_record2[cnt++] = nav_rx.sita_now[2][1];
        data_record2[cnt++] = nav_rx.sita_now[2][2];
        data_record2[cnt++] = nav_rx.epos_n_tar[2].x;
        data_record2[cnt++] = nav_rx.epos_n_tar[2].y;
        data_record2[cnt++] = nav_rx.epos_n_tar[2].z;
        data_record2[cnt++] = nav_rx.epos_n_now[2].x;
        data_record2[cnt++] = nav_rx.epos_n_now[2].y;
        data_record2[cnt++] = nav_rx.epos_n_now[2].z;

        data_record2[cnt++] = nav_rx.sita_tar[3][0];
        data_record2[cnt++] = nav_rx.sita_tar[3][1];
        data_record2[cnt++] = nav_rx.sita_tar[3][2];
        data_record2[cnt++] = nav_rx.sita_now[3][0];
        data_record2[cnt++] = nav_rx.sita_now[3][1];
        data_record2[cnt++] = nav_rx.sita_now[3][2];
        data_record2[cnt++] = nav_rx.epos_n_tar[3].x;
        data_record2[cnt++] = nav_rx.epos_n_tar[3].y;
        data_record2[cnt++] = nav_rx.epos_n_tar[3].z;
        data_record2[cnt++] = nav_rx.epos_n_now[3].x;
        data_record2[cnt++] = nav_rx.epos_n_now[3].y;
        data_record2[cnt++] = nav_rx.epos_n_now[3].z;

        data_record2[cnt++] = nav_rx.tau_tar[0][0];
        data_record2[cnt++] = nav_rx.tau_tar[0][1];
        data_record2[cnt++] = nav_rx.tau_tar[0][2];
        data_record2[cnt++] = nav_rx.tau_now[0][0];
        data_record2[cnt++] = nav_rx.tau_now[0][1];
        data_record2[cnt++] = nav_rx.tau_now[0][2];

        data_record2[cnt++] = nav_rx.tau_tar[1][0];
        data_record2[cnt++] = nav_rx.tau_tar[1][1];
        data_record2[cnt++] = nav_rx.tau_tar[1][2];
        data_record2[cnt++] = nav_rx.tau_now[1][0];
        data_record2[cnt++] = nav_rx.tau_now[1][1];
        data_record2[cnt++] = nav_rx.tau_now[1][2];

        data_record2[cnt++] = nav_rx.tau_tar[2][0];
        data_record2[cnt++] = nav_rx.tau_tar[2][1];
        data_record2[cnt++] = nav_rx.tau_tar[2][2];
        data_record2[cnt++] = nav_rx.tau_now[2][0];
        data_record2[cnt++] = nav_rx.tau_now[2][1];
        data_record2[cnt++] = nav_rx.tau_now[2][2];

        data_record2[cnt++] = nav_rx.tau_tar[3][0];
        data_record2[cnt++] = nav_rx.tau_tar[3][1];
        data_record2[cnt++] = nav_rx.tau_tar[3][2];
        data_record2[cnt++] = nav_rx.tau_now[3][0];
        data_record2[cnt++] = nav_rx.tau_now[3][1];
        data_record2[cnt++] = nav_rx.tau_now[3][2];
        cnt = 0;
        for(int i=0;i<10;i++){
         data_record3[cnt++] = nav_rx.temp_record[i];
        }

        break;
    }
}

void recorder(float dt) {
    static int state = 0;
    int i = 0;
    switch (state)
    {
    case 0:
        if(ocu.record.en_record){
            fp1 = fopen("/home/odroid/Tinker/Data/file1.txt", "w");
            fp2 = fopen("/home/odroid/Tinker/Data/file2.txt", "w");
            fp3 = fopen("/home/odroid/Tinker/Data/file3.txt", "w");

            if (fp1 != NULL) {
                printf("Navigation::Recording Data \n");
                state++;
            }
        }
        break;
    case 1:
        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp1, "%.6f ",//姿态数据
                data_record1[i]);
        fprintf(fp1, "\n");

        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp2, "%.6f ",//单腿力
                data_record2[i]);
        fprintf(fp2, "\n");

        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp3, "%.6f ",//单腿运动
                data_record3[i]);
        fprintf(fp3, "\n");

        if(ocu.record.en_record==0)
            state++;
        break;
    case 2:
        fclose(fp1); fclose(fp2); fclose(fp3);
        state=0;
        printf("Navigation::Record Finish \n");
        break;
    }

}

void* Thread_Record(void*)//内存管理线程
{
    static int cnt = 0;
    float sys_dt = 0;
    float timer[5]={0};

    while (1)
    {
        sys_dt = Get_Cycle_T(11);
        timer[0]+=sys_dt*2.5;
        if(!mem_connect_c)
        {
            nav_rx.att_now[0]=sin(timer[0])*5;
            nav_rx.att_now[1]=cos(timer[0])*5;
            nav_rx.att_now[2]=sin(timer[0])*2;
        }
        record_thread(0,sys_dt);
        record_thread(1,sys_dt);
        record_thread(2,sys_dt);
        recorder(timer[0]);
        usleep(5000);
    }
    return 0;
}



void* Thread_UDP_OCU(void*)//OCU UDP通讯线程  as master
{
    static int cnt = 0;
    float sys_dt = 0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
   setnonblocking(sock_fd);//非阻塞
   struct sockaddr_in addr_serv;
   int len;
   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(SERV_PORT);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;

    param_init();
    while (1)
    {
        sys_dt = Get_Cycle_T(10);
        timer[0]+=sys_dt;

        //读取客户端
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num <= 0)
        {
            //perror("OCU recvfrom error:");
            //exit(1);
        }
        else{
        //解码
            //printf("recv_num=%d\n",recv_num);
            UDP_RX_PROCESS(recv_buf,recv_num);

            //回传数据
            UDP_OCU_TX(sys_dt);
            for(i=0;i<usb_send_cnt;i++)
                send_buf[i] = SendBuff_USB[i];

            send_num = sendto(sock_fd, send_buf, usb_send_cnt, MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            //printf("usb_send_cnt=%d\n",usb_send_cnt);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
            //printf("sys_dt=%f\n",sys_dt);
        }
        //usleep(5000);
        usleep(5*1000);
    }
    close(sock_fd);
    return 0;
}

void* Thread_UDP_SDK(void*)//SDK UDP通讯线程-------------ROS controller is serve no IP set
{
    static int cnt = 0;
    float sys_dt = 0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
   setnonblocking(sock_fd);//非阻塞
   struct sockaddr_in addr_serv;
   int len;

   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(SERV_PORT_SDK);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    while (1)
    {
        sys_dt = Get_Cycle_T(10);
        timer[0]+=sys_dt;

        //读取客户端
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num <= 0)
        {
            moco_sdk.loss_cnt++;
        }
        else{
        //解码
            //printf("recv_num=%d\n",recv_num);
            memcpy(&_gait_msg,recv_buf,sizeof(_gait_msg));

            moco_sdk.loss_cnt=0;
            if(!moco_sdk.ros_connect){
                moco_sdk.connect=1;
                moco_sdk.ros_connect=1;
                printf("Navigation::UDP SDK Link!!\n");
            }
            moco_sdk.gait_mode=_gait_msg.gait_mode;
            moco_sdk.rc_spd_cmd[Xr]=_gait_msg.speed_cmd[0];
            moco_sdk.rc_spd_cmd[Yr]=_gait_msg.speed_cmd[1];
            moco_sdk.rc_spd_cmd[Zr]=_gait_msg.speed_cmd[2];
            moco_sdk.rc_att_cmd[PITr]=_gait_msg.att_cmd[0];
            moco_sdk.rc_att_cmd[ROLr]=_gait_msg.att_cmd[1];
            moco_sdk.rc_att_cmd[YAWr]=_gait_msg.att_cmd[2];
            moco_sdk.rc_att_rate_cmd[Xr]=_gait_msg.rate_cmd[0];
            moco_sdk.rc_att_rate_cmd[Yr]=_gait_msg.rate_cmd[1];
            moco_sdk.rc_att_rate_cmd[Zr]=_gait_msg.rate_cmd[2];
            moco_sdk.rc_pos_cmd[Xr]=_gait_msg.pos_cmd[0];
            moco_sdk.rc_pos_cmd[Yr]=_gait_msg.pos_cmd[1];
            moco_sdk.rc_pos_cmd[Zr]=_gait_msg.pos_cmd[2];
            printf("sdk::%f %f %f\n",_gait_msg.speed_cmd[0],_gait_msg.speed_cmd[1],_gait_msg.rate_cmd[2]);
            if(moco_sdk.ros_connect&&moco_sdk.gait_mode>=100&&!ocu.connect){
                ocu.connect=1;
                ocu.loss_cnt=0;
                ocu.mode=2;
                printf("Navigation::No Rc Dir Control By ROS!!\n");
            }

#if 0
            printf("[%d] speed %f %f %f\n",moco_sdk.gait_mode,_gait_msg.speed_cmd[0],_gait_msg.speed_cmd[1],_gait_msg.speed_cmd[2]);
            printf("%f %f %f\n",nav_rx.dcom_n_now[0],nav_rx.dcom_n_now[1],nav_rx.dcom_n_now[2]);
            printf("att %f %f %f\n",_gait_msg.att_cmd[0],_gait_msg.att_cmd[1],_gait_msg.att_cmd[2]);
            printf("rate %f %f %f\n",_gait_msg.rate_cmd[0],_gait_msg.rate_cmd[1],_gait_msg.rate_cmd[2]);
            //UDP_RX_PROCESS_SDK(recv_buf,recv_num);
#endif
            //回传数据 to 客户端
            UDP_OCU_TX_SDK(sys_dt);

            for(i=0;i<usb_send_cnt_SDK;i++)
                send_buf[i] = SendBuff_USB_SDK[i];

            send_num = sendto(sock_fd, send_buf, usb_send_cnt_SDK, MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            if(send_num < 0)
            {
                //perror("OCU sendto error:");
                //exit(1);
            }
            //printf("sys_dt=%f\n",sys_dt);
        }

        if(moco_sdk.loss_cnt>300&&moco_sdk.ros_connect==1){
            moco_sdk.ros_connect=0;
            moco_sdk.gait_mode=0;
            moco_sdk.loss_cnt=0;
            moco_sdk.rc_spd_cmd[Xr]=0;
            moco_sdk.rc_spd_cmd[Yr]=0;
            moco_sdk.rc_spd_cmd[Zr]=0;
            moco_sdk.rc_att_cmd[PITr]=0;
            moco_sdk.rc_att_cmd[ROLr]=0;
            moco_sdk.rc_att_cmd[YAWr]=0;
            moco_sdk.rc_att_rate_cmd[Xr]=0;
            moco_sdk.rc_att_rate_cmd[Yr]=0;
            moco_sdk.rc_att_rate_cmd[Zr]=0;
            moco_sdk.rc_pos_cmd[Xr]=0;
            moco_sdk.rc_pos_cmd[Yr]=0;
            moco_sdk.rc_pos_cmd[Zr]=0;
            printf("Navigation::UDP SDK Loss!!\n");
        }
       usleep(5000);
    }
    close(sock_fd);
    return 0;
}

void* Thread_UDP_ARM(void*)//SDK UDP通讯线程-------------OCU Quest
{
    static int cnt = 0;
    float sys_dt = 0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
   setnonblocking(sock_fd);//非阻塞
   struct sockaddr_in addr_serv;
   int len;

   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(SERV_PORT_ARM);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    while (1)
    {
        sys_dt = Get_Cycle_T(11);
        timer[0]+=sys_dt;

        //读取客户端
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        //printf("recv_num=%d\n",recv_num);
        if(recv_num <= 0)
        {
            //perror("OCU recvfrom error:");
            //exit(1);
        }
        else{


            send_num = sendto(sock_fd, send_buf, usb_send_cnt, MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            //printf("usb_send_cnt=%d\n",usb_send_cnt);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
        }
       usleep(10*1000);
    }
    close(sock_fd);
    return 0;
}


int main(int argc, char *argv[])
{
    printf("OmniRobHub-Interfacing!\n");
    pthread_t tida, tidb, tidc,tidd, tide;
    pthread_mutex_init(&lock, NULL);
    pthread_create(&tida, NULL, Thread_Mem, NULL);
    pthread_create(&tidb, NULL, Thread_UDP_OCU, NULL);
    pthread_create(&tidc, NULL, Thread_UDP_SDK, NULL);
    pthread_create(&tide, NULL, Thread_Record, NULL);
    pthread_join(tida, NULL);
    pthread_join(tidb, NULL);
    pthread_join(tidc, NULL);
    pthread_join(tide, NULL);
    return 1;
}

