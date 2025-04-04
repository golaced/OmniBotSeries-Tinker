
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
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include "eso_RL.h"

#define SERV_PORT_MAP   6666
pid_t pid_control,pid_servo,pid_mpc,pid_nav;
// ps -ef | grep control_task   ps a全部查看
// ps -ef | grep hardware_task
// kill －9 324  ps aux | grep "XXX"
// sudo nano /etc/rc.local
#define MUL_THREAD 1
#define USE_MAP    0
using namespace qpOASES;
pthread_mutex_t lock;
pthread_mutex_t lock_mpc;

void setCurrentThreadHighPriority(bool value) {
#if 1
  // Start out with a standard, low-priority setup for the sched params.
  struct sched_param sp;
  bzero((void*)&sp, sizeof(sp));
  int policy = SCHED_OTHER;

  // If desired, set up high-priority sched params structure.
  if (value) {
    // FIFO scheduler, ranked above default SCHED_OTHER queue
    policy = SCHED_FIFO;
    // The priority only compares us to other SCHED_FIFO threads, so we
    // just pick a random priority halfway between min & max.
    const int priority = (sched_get_priority_max(policy) + sched_get_priority_min(policy)) / 2;

    sp.sched_priority = priority;
  }

  // Actually set the sched params for the current thread.
  if (0 == pthread_setschedparam(pthread_self(), policy, &sp)) {
    printf("IO Thread using high-priority scheduler!\n");
  }
#endif
}

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

void rc_process(float T){
    float FLT_RC=0;
    static char key_st_reg=0,key_a_reg=0,key_selet_reg,request_mode_reg=0;
    static char aux_smart_reg=0;
    static char rc_interuped=0;
    static float rc_interuped_cnt=0;
    float weight_yaw,err_yaw;

     if(ocu.mode==RC_REMOTE){//auto mode
        if(key_st_reg==0&&ocu.key_st==1&&gait_ww.state_gait==2)
        {
            if(sdk.sdk_mode==0){
                sdk.sdk_mode=1;
                printf("Control:: Enable SDK command!\n");
            }else{
                sdk.sdk_mode=0;
                printf("Control:: Disable SDK command!\n");
            }
        }else if(key_a_reg==0&&sdk.sdk_mode==1&&(ocu.key_a==1||ocu.key_b==1||ocu.key_y==1)){
                sdk.sdk_mode=0;
                printf("Control:: Disable SDK command!\n");
        }
     }

     if(request_mode_reg!=nav_tx.request_gait){//control by ros
        if(nav_tx.request_gait==100){
            sdk.sdk_mode=1;
            printf("ROS API:: Enable SDK command!\n");
        }
        else if(nav_tx.request_gait==101){
            sdk.sdk_mode=0;
            printf("ROS API:: Disable SDK command!\n");
        }else if(nav_tx.request_gait==102){
            sdk.sdk_mode=0;
            ocu.key_ud=-1;
            printf("ROS API:: Force Safe command!\n");
        }
     }
     request_mode_reg=nav_tx.request_gait;

     if(ocu.esp32_connect){
         sdk.sdk_mode=ocu.auto_mode;
         if(ocu.auto_mode==1)//enable auto
            nav_tx.request_gait=100;
         else
            nav_tx.request_gait=99;
     }

     //printf("request_gait=%d ouc=%d sbus=%d uwb=%d en_sdk=%d\n",nav_tx.request_gait,ocu.connect,ocu.sbus_conncect,sdk_tar_way_point_uwb.check,sdk.sdk_mode) ;
     if((ocu.connect||sdk.sdk_mode==1||ocu.esp32_connect)&&sdk.sdk_mode==0){//nav_tx.request_gait==99){

          ocu.rc_spd_w[Yr]=dead(ocu.rc_spd_w[Yr],0.15);
          if(ocu.mode==RC_REMOTE||ocu.esp32_connect){
             switch(vmc_all.gait_mode){
                 default:
                    FLT_RC=0.68;
                    DigitalLPF(ocu.rc_spd_w[Xr]*MAX_SPD_X,&vmc_all.tar_spd.x,FLT_RC,T);
                    DigitalLPF(ocu.rc_spd_w[Yr]*MAX_SPD_Y,&vmc_all.tar_spd.y,FLT_RC/2,T);
                    DigitalLPF(ocu.rate_yaw_w*MAX_SPD_RAD,&vmc_all.tar_spd.z,FLT_RC/2,T);
                    DigitalLPF(ocu.rc_att_w[PITr]*vmc_all.param.MAX_PIT,&vmc_all.tar_att[PITr],FLT_RC,T);
                    DigitalLPF(ocu.rc_att_w[ROLr]*vmc_all.param.MAX_ROL, &vmc_all.tar_att[ROLr],FLT_RC/2,T);
                 break;
             }
          }
          //printf("%f %f %f\n",ocu.rc_spd_w[Xr],ocu.rc_spd_w[Yr],ocu.rate_yaw_w);
         if(ABS(vmc_all.tar_spd.x)>MIN_SPD_ST||ABS(vmc_all.tar_spd.y)>MIN_SPD_ST)
            vmc_all.param.have_cmd_rc[0]=1;
         else
            vmc_all.param.have_cmd_rc[0]=0;
         if(ABS(vmc_all.tar_spd.z)>MIN_SPD_ST_RAD)
            vmc_all.param.have_cmd_rc[1]=1;
         else
            vmc_all.param.have_cmd_rc[1]=0;

     }
     else if(sdk.sdk_mode==1)//nav_tx.request_gait!=99)//SDK commond Use ROS=============================
     {
        printf("ssrc_interuped=%d sdk.sdk_mode=%d sdk.cmd_vyaw=%f %f %f sdk.sdk_mode=%d\n",rc_interuped,sdk.sdk_mode,sdk.cmd_vx,sdk.cmd_vy,sdk.cmd_vyaw,sdk.sdk_mode);
        if(rc_interuped||sdk.cmd_vyaw==-1||sdk.sdk_mode==0){//rc interupte
            MAX_SPD=MAX_FSPD;
            switch(vmc_all.gait_mode){
                default:
                   FLT_RC=1.68;
                   DigitalLPF(ocu.rc_spd_w[Xr]*MAX_SPD,&vmc_all.tar_spd.x,FLT_RC,T);
                   DigitalLPF(ocu.rc_spd_w[Yr]*MAX_SPD,&vmc_all.tar_spd.y,FLT_RC/2,T);
                   DigitalLPF(ocu.rate_yaw_w*MAX_SPD_RAD,&vmc_all.tar_spd.z,FLT_RC,T);
                   DigitalLPF(ocu.rc_att_w[PITr]*vmc_all.param.MAX_PIT,&vmc_all.tar_att[PITr],FLT_RC,T);
                   DigitalLPF(ocu.rc_att_w[ROLr]*vmc_all.param.MAX_ROL, &vmc_all.tar_att[ROLr],FLT_RC/2,T);
                break;
            }

            if((fabs(ocu.rc_spd_w[Xr])>0.1||ocu.rc_spd_w[Yr]>0.1||ocu.rate_yaw_w>0.1)&&!rc_interuped)
                {rc_interuped=1;rc_interuped_cnt=0;printf("Control:: Disable SDK for RC interuped!\n");		}
            if((fabs(ocu.rc_spd_w[Xr])<0.1&&ocu.rc_spd_w[Yr]<0.1&&ocu.rate_yaw_w<0.1)&&rc_interuped==1)
                rc_interuped_cnt+=T;
            else
                rc_interuped_cnt=0;

            if(rc_interuped_cnt>2&&rc_interuped==1)
                {rc_interuped=0;printf("Control:: Re-enable SDK command without RC!\n");}
        }else{//SDK control====================================
            FLT_RC=1.0;
            DigitalLPF(sdk.cmd_vx,&vmc_all.tar_spd.x,FLT_RC/3,T);
            DigitalLPF(sdk.cmd_vy,&vmc_all.tar_spd.y,FLT_RC/2,T);
            DigitalLPF(sdk.cmd_vyaw*57.3,&vmc_all.tar_spd.z,FLT_RC,T);

            vmc_all.tar_spd.x=LIMIT(vmc_all.tar_spd.x,-MAX_SPD,MAX_SPD);
            vmc_all.tar_spd.y=LIMIT(vmc_all.tar_spd.y,-MAX_SPD,MAX_SPD);

            DigitalLPF(sdk.cmd_pit,&vmc_all.tar_att[PITr],FLT_RC,T);
            DigitalLPF(sdk.cmd_rol, &vmc_all.tar_att[ROLr],FLT_RC,T);
        }

         if(ABS(vmc_all.tar_spd.x)>MIN_SPD_ST||ABS(vmc_all.tar_spd.y)>MIN_SPD_ST)
            vmc_all.param.have_cmd_rc[0]=1;
         else
            vmc_all.param.have_cmd_rc[0]=0;
         if(ABS(vmc_all.tar_spd.z)>MIN_SPD_ST_RAD)
            vmc_all.param.have_cmd_rc[1]=1;
         else
            vmc_all.param.have_cmd_rc[1]=0;
     }
     else{
          FLT_RC=0.86;
          DigitalLPF(0,&vmc_all.tar_spd.x,FLT_RC/4,T);
          DigitalLPF(0,&vmc_all.tar_spd.y,FLT_RC/4,T);
          DigitalLPF(0,&vmc_all.tar_spd.z,FLT_RC,T);
          DigitalLPF(0,&vmc_all.tar_att[PITr],FLT_RC/2,T);
          DigitalLPF(0,&vmc_all.tar_att[ROLr],0.068,T);
      }

  //----------------------Beep Reset---------------
      static int beep_changing=0;
      static float beep_timer=0;
      if(robotwb.beep_state!=0&&!beep_changing)
         { beep_changing=1;beep_timer=0;}
      if(beep_changing)
          beep_timer+=T;
      if(beep_timer>2)
          robotwb.beep_state=beep_timer=beep_changing=0;

    key_a_reg=ocu.key_a;
    key_st_reg=ocu.key_st;
    key_selet_reg=ocu.key_back;
    aux_smart_reg=spi_rx.ocu.sbus_aux[3];
}

void* Thread_T1(void*)//控制线程
{
    float timer[5]={0};
    float sys_dt = 0,dT=0,T;
    int i=0;
    CAN_motor_init();
    for (i = 0; i < 4; i++) {
        cal_jacobi_new(&vmc[i], 0.002);	//计算雅克比
        cal_invjacobi(&vmc[i]);		//计算雅克比逆
    }

    setCurrentThreadHighPriority(11);

    while(1)
    {
        float leg_dt[2]={0};
        T = Get_Cycle_T(31);
        leg_dt[1]=T;
        subscribe_imu_to_webot(&robotwb, T);//赋值给robot结构体

        readAllMotorPos(&robotwb, leg_dt[1]);//读取角度

        locomotion_sfm(leg_dt[1]);//摆动控制

        if(vmc_all.gait_mode==G_RL )
            force_control_and_dis_rl(leg_dt[1]);//位力混控 TORT版本
        else//default
            force_control_and_dis_stand(leg_dt[1]);//位力混控 站立版本

        if(T>0.005)
            printf("OT-S::sys_dt=%f\n",T);

        usleep(2000);
    }
}

void* Thread_T5(void*)//控制线程
{

    float timer[5]={0};
    float sys_dt = 0,dT=0;
    int i=0;

    while(1)
    {
        dT = Get_Cycle_T(32);
        rc_process(dT);

        usleep(5*1000);
        if(dT>0.0055)
            printf("OT-C::sys_dt=%f\n",dT);
    }
}

//--------------------------------------------------------------------------------------------
struct recv_msg
{
      char flag_map;
      int16_t map2d_local[80*80];
      float odom_pos[3];
      float odom_spd[3];
      float odom_att[3];
      float odom_rate[3];
} _recv_msg;

struct send_msg
{
      float euler[3];
      float omega[3];
      float velxyz[3];
} _send_msg;


void* Thread_UDP_MAP(void*)//SDK UDP通讯线程-------------MAP Divide
{
    static int mode_reg=0;
    static int cnt = 0;
    int map_lsm=0;
    float odom_z_off=0;
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
   addr_serv.sin_port = htons(SERV_PORT_MAP);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    char send_buf[500]={0},recv_buf[1024*30]={0};
    struct sockaddr_in addr_client;
    while (1)
    {
        sys_dt = Get_Cycle_T(12);
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
        //解码
            memcpy(&_recv_msg,recv_buf,sizeof(_recv_msg));

            mode_reg=vmc_all.gait_mode;
            //send_num = sendto(sock_fd, send_buf, usb_send_cnt, MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            //printf("usb_send_cnt=%d\n",usb_send_cnt);
            if(send_num < 0)
            {
                perror("MAP sendto error:");
                exit(1);
            }

        }
        usleep(50*1000);
    }
    close(sock_fd);
    return 0;
}

//RL
struct _msg_request_tinymal
{
    float trigger;
    float command[4];
    float eu_ang[3];
    float omega[3];
    float acc[3];
    float q[12];
    float dq[12];
    float tau[12];
    float q_init[12];
} _msg_request_tinymal;

struct _msg_response_tinymal
{
    float q_exp[12];
    float dq_exp[12];
    float tau_exp[12];
} _msg_response_tinymal;

void* Thread_UDP_RL_Tinymal(void*)//RL Service
{
    static int cnt = 0;
    float sys_dt = 0;
    float loss_cnt_sdk=0;
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
   addr_serv.sin_port = htons(10000);//端口<-----------------Python
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }
    int id_list[12]={0,1,2,3,4,5, 7,8,9,10,11,12};
    int recv_num=0,send_num=0;
    int recording_flag=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    int cnt_p=0;
    printf("Thread UDP Master RL-Tinymal\n");
#if 0
    vmc_all.default_action[0]=-0.16;
    vmc_all.default_action[1]=0.68;
    vmc_all.default_action[2]=1.3;

    vmc_all.default_action[3]=0.16;
    vmc_all.default_action[4]=0.68;
    vmc_all.default_action[5]=1.3;

    vmc_all.default_action[6]=-0.16;
    vmc_all.default_action[7]=0.68;
    vmc_all.default_action[8]=1.3;

    vmc_all.default_action[9]=0.16;
    vmc_all.default_action[10]=0.68;
    vmc_all.default_action[11]=1.3;
    vmc_all.sita3_off=175;
#endif
    while (1)
    {
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        vmc_all.loss_rl++;
        if(vmc_all.loss_rl>100)
            vmc_all.rl_connect=0;
        if(recv_num >0)
        {   //--------------actions---------------
            vmc_all.rl_connect=1;
            vmc_all.loss_rl=0;
            memcpy(&_msg_response_tinymal,recv_buf,sizeof(_msg_response_tinymal));
            if(vmc_all.gait_mode==G_RL){
                for(int i=0;i<12;i++){
                    float temp=LIMIT(_msg_response_tinymal.q_exp[i],-5,5)*vmc_all.action_scale+vmc_all.default_action[id_list[i]];//action scale->0.25 good 50Hz
                    leg_motor_all.q_set[id_list[i]]=LIMIT(temp*57.3,leg_motor_all.q_min[id_list[i]],leg_motor_all.q_max[id_list[i]]);
                    leg_motor_all.q_set[id_list[i]]=temp*57.3;
                }
                //printf("action:%.3f %.3f %.3f\n",_msg_response_tinker.q_exp[0],_msg_response_tinker.q_exp[1],_msg_response_tinker.q_exp[2]);
            }
            //---------------obs------------------
            sys_dt = Get_Cycle_T(40);
            timer[0]+=sys_dt;
            if(timer[0]>vmc_all.net_run_dt){//50Hz to elav the net
                timer[0]=0;
                _msg_request_tinymal.trigger=1;
            }else
                _msg_request_tinymal.trigger=0;
            _msg_request_tinymal.command[0]=   LIMIT(vmc_all.tar_spd.x*2,-1,1);
            _msg_request_tinymal.command[1]=   LIMIT(-vmc_all.tar_spd.y*4,-1,1);

            if (fabs(vmc_all.tar_spd.z) > 1)
            {
                _msg_request_tinymal.command[2] = vmc_all.tar_spd.z/57.3;
                robotwb.exp_att.yaw = robotwb.now_att.yaw;
            }
            else {
                _msg_request_tinymal.command[2] = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.25)*1.4/57.3;
            }
            _msg_request_tinymal.command[2]=   LIMIT(_msg_request_tinymal.command[2],-1,1);

            _msg_request_tinymal.eu_ang[0]=vmc_all.att[ROLr]/57.3;
            _msg_request_tinymal.eu_ang[1]=-vmc_all.att[PITr]/57.3;
            _msg_request_tinymal.eu_ang[2]=-vmc_all.att[YAWr]/57.3;
            _msg_request_tinymal.omega[0]=vmc_all.att_rate[ROLr]/57.3;
            _msg_request_tinymal.omega[1]=-vmc_all.att_rate[PITr]/57.3;
            _msg_request_tinymal.omega[2]=vmc_all.att_rate[YAWr]/57.3;

            for(int i=0;i<12;i++){
                _msg_request_tinymal.q[i]=leg_motor_all.q_now[id_list[i]]/57.3;
                _msg_request_tinymal.dq[i]=leg_motor_all.qd_now[id_list[i]]/57.3;
                _msg_request_tinymal.q_init[i]=vmc_all.default_action[id_list[i]];
            }

            //--------------------------For switch
            if(vmc_all.gait_mode!=G_RL){
                _msg_request_tinymal.eu_ang[0]=0;
                _msg_request_tinymal.eu_ang[1]=0;
                _msg_request_tinymal.eu_ang[2]=0;
                _msg_request_tinymal.omega[0]=0;
                _msg_request_tinymal.omega[1]=0;
                _msg_request_tinymal.omega[2]=0;

                for(int i=0;i<12;i++){
                    _msg_request_tinymal.q[i]=vmc_all.default_action[id_list[i]];
                    _msg_request_tinymal.dq[i]=0;
                    _msg_request_tinymal.q_init[i]=vmc_all.default_action[id_list[i]];
                }
            }
            memcpy(send_buf,&_msg_request_tinymal,sizeof(_msg_request_tinymal));
            send_num = sendto(sock_fd, send_buf, sizeof(_msg_request_tinymal), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
        }
        usleep(2*1000);//
    }
    close(sock_fd);
    return 0;
}
//RL UDP
struct _msg_request_tinker
{
    float trigger;
    float command[4];
    float eu_ang[3];
    float omega[3];
    float acc[3];
    float q[10];
    float dq[10];
    float tau[10];
    float q_init[10];
} _msg_request_tinker;

struct _msg_response_tinker
{
    float q_exp[10];
    float dq_exp[10];
    float tau_exp[10];
} _msg_response_tinker;

void* Thread_UDP_RL_Tinker(void*)//RL Service
{
    static int cnt = 0;
    float sys_dt = 0;
    float loss_cnt_sdk=0;
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
   addr_serv.sin_port = htons(10000);//端口<-----------------Python
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    int recording_flag=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    int cnt_p=0;
    int id_list[10]={0,1,2,3,4, 7,8,9,10,11};
    printf("Thread UDP Master RL-Tinker14\n");
#if 0
    vmc_all.default_action[0]=0.0;
    vmc_all.default_action[1]=-0.07;
    vmc_all.default_action[2]=0.56;
    vmc_all.default_action[3]=-1.12;
    vmc_all.default_action[4]=0.57;

    vmc_all.default_action[7]=0.0;
    vmc_all.default_action[8]=0.07;
    vmc_all.default_action[9]=0.56;
    vmc_all.default_action[10]=-1.12;
    vmc_all.default_action[11]=0.57;
#endif
    while (1)//sim 0->left 1->right
    {
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        vmc_all.loss_rl++;
        if(vmc_all.loss_rl>100)
            vmc_all.rl_connect=0;
#if HEAD_USE_DC//tinker head control
        if(gait_ww.state_gait>=2){
            DigitalLPF(ocu.rc_att_w[PITr]*50,&leg_motor_all.q_set[5],0.9,0.005);//pitch
            leg_motor_all.q_set[5]=LIMIT(leg_motor_all.q_set[5],-35,60);
            DigitalLPF(-ocu.rc_att_w[ROLr]*70,&leg_motor_all.q_set[6],0.8,0.005);//yaw
        }
#endif
        if(recv_num >0)
        {   //--------------actions---------------
            vmc_all.rl_connect=1;
            vmc_all.loss_rl=0;
            memcpy(&_msg_response_tinker,recv_buf,sizeof(_msg_response_tinker));
            if(vmc_all.gait_mode==G_RL){
                for(int i=0;i<10;i++){
                    float temp=LIMIT(_msg_response_tinker.q_exp[i],-5,5)*vmc_all.action_scale+vmc_all.default_action[id_list[i]];//action scale->0.25 good 50Hz
                    //float temp=LIMIT(_msg_response_tinker.q_exp[i],-3,3)*0.3+vmc_all.default_action[i];//action scale->0.3  100Hz
                    leg_motor_all.q_set[id_list[i]]=LIMIT(temp*57.3,leg_motor_all.q_min[id_list[i]],leg_motor_all.q_max[id_list[i]]);
                    leg_motor_all.q_set[id_list[i]]=temp*57.3;
                }
                //printf("action:%.3f %.3f %.3f\n",_msg_response_tinker.q_exp[0],_msg_response_tinker.q_exp[1],_msg_response_tinker.q_exp[2]);
            }
            //
            //---------------obs------------------
            sys_dt = Get_Cycle_T(40);
            timer[0]+=sys_dt;
            if(timer[0]>vmc_all.net_run_dt){//50Hz to elav the net
                timer[0]=0;
                _msg_request_tinker.trigger=1;
            }else
                _msg_request_tinker.trigger=0;

            _msg_request_tinker.command[0]=   LIMIT(vmc_all.tar_spd.x*1.5,-0.15,1.0)+vmc_all.rl_commond_rl_rst[0]+
                    vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[0]*(vmc_all.gait_mode==G_RL);
            _msg_request_tinker.command[1]=   LIMIT(-vmc_all.tar_spd.y*2,-1.0,1.0)+
                    vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[1]*(vmc_all.gait_mode==G_RL);
            //printf("%f %f\n",_msg_request_tinker.command[0],_msg_request_tinker.command[1]);
            if (fabs(vmc_all.tar_spd.z) > 1)
            {
                _msg_request_tinker.command[2] = vmc_all.tar_spd.z/57.3*1.5;
                robotwb.exp_att.yaw = robotwb.now_att.yaw;
            }
            else {
                _msg_request_tinker.command[2] = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.25)*1.4/57.3;
            }
            _msg_request_tinker.command[2]=   LIMIT(_msg_request_tinker.command[2],-1,1);
           // printf("cmd:%f %f %f\n",_msg_request.command[0],_msg_request.command[1],_msg_request.command[2]);
           // printf("robotwb.exp_att.yaw=%f robotwb.now_att.yaw=%f\n",robotwb.exp_att.yaw, robotwb.now_att.yaw);
            _msg_request_tinker.eu_ang[0]=vmc_all.att[ROLr]/57.3;
            _msg_request_tinker.eu_ang[1]=-vmc_all.att[PITr]/57.3;
            _msg_request_tinker.eu_ang[2]=-vmc_all.att[YAWr]/57.3;
            _msg_request_tinker.omega[0]=vmc_all.att_rate[ROLr]/57.3;
            _msg_request_tinker.omega[1]=-vmc_all.att_rate[PITr]/57.3;
            _msg_request_tinker.omega[2]=vmc_all.att_rate[YAWr]/57.3;

            for(int i=0;i<10;i++){
                _msg_request_tinker.q[i]=leg_motor_all.q_now[id_list[i]]/57.3;
                _msg_request_tinker.dq[i]=leg_motor_all.qd_now[id_list[i]]/57.3;
                _msg_request_tinker.q_init[i]=vmc_all.default_action[id_list[i]];
            }

            //--------------------------For switch
            if(vmc_all.gait_mode!=G_RL){
                _msg_request_tinker.eu_ang[0]=0;
                _msg_request_tinker.eu_ang[1]=0;
                _msg_request_tinker.eu_ang[2]=0;
                _msg_request_tinker.omega[0]=0;
                _msg_request_tinker.omega[1]=0;
                _msg_request_tinker.omega[2]=0;

                for(int i=0;i<10;i++){
                    _msg_request_tinker.q[i]=vmc_all.default_action[id_list[i]];
                    _msg_request_tinker.dq[i]=0;
                    _msg_request_tinker.q_init[i]=vmc_all.default_action[id_list[i]];
                }
            }
#if 0
    static int cnt_p=0;
    //printf("temp :%.1f\n",leg_motor_all.q_now[id_list[0]]);
    if(cnt_p++>50){cnt_p=0;
        printf("att :%.1f %.1f %.1f\n",_msg_request_tinker.eu_ang[0],_msg_request_tinker.eu_ang[1],_msg_request_tinker.eu_ang[2]);
        printf("rate:%.1f %.1f %.1f\n",_msg_request_tinker.omega[0],_msg_request_tinker.omega[1],_msg_request_tinker.omega[2]);
        printf("q0 :%.1f %.1f %.1f %.1f %.1f \n", _msg_request_tinker.q[0], _msg_request_tinker.q[1], _msg_request_tinker.q[2], _msg_request_tinker.q[3], _msg_request_tinker.q[4]);
        printf("dq0:%.1f %.1f %.1f %.1f %.1f \n", _msg_request_tinker.dq[0], _msg_request_tinker.dq[1], _msg_request_tinker.dq[2], _msg_request_tinker.dq[3], _msg_request_tinker.dq[4]);
        printf("q1 :%.1f %.1f %.1f %.1f %.1f \n", _msg_request_tinker.q[5],_msg_request_tinker.q[6], _msg_request_tinker.q[7], _msg_request_tinker.q[8], _msg_request_tinker.q[9]);
        printf("dq1:%.1f %.1f %.1f %.1f %.1f \n", _msg_request_tinker.q[5],_msg_request_tinker.dq[6], _msg_request_tinker.dq[7], _msg_request_tinker.dq[8], _msg_request_tinker.dq[9]);
    }
#endif
            memcpy(send_buf,&_msg_request_tinker,sizeof(_msg_request_tinker));
            send_num = sendto(sock_fd, send_buf, sizeof(_msg_request_tinker), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
        }
        usleep(2*1000);//
    }
    close(sock_fd);
    return 0;
}

struct _msg_ocu_tinker
{
    char Head1;
    char Head2;
    char gait_mode;
    char auto_mode;
    char head_mode;
    char step_mode;
    char cal_mode;
    char cal_sel;
    float att_cmd[3];
    float vel_cmd[3];
    float pos_cmd[3];
    int power_cnt;
} _msg_ocu_tinker;

struct _msg_fb_tinker
{
    char gait_mode;
    char auto_mode;
    char head_mode;
    char step_mode;
    char heart_cnt;
    float att[3];
    float vel[3];
    float acc[3];
    float mag[3];
    float gyro[3];
    float baro;
    float pos[3];
    float q_leg[14];
    float q_arm[14];
} _msg_fb_tinker;


void* Thread_UDP_OCU(void*)//OCU UDP For ESP32
{
    static int cnt = 0;
    float sys_dt = 0;
    float loss_cnt_sdk=0;
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
   addr_serv.sin_port = htons(7070);//端口<-----------------Python
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
         perror("bind error:");
         exit(1);
    }

    int recv_num=0,send_num=0;
    int recording_flag=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    int cnt_p=0;
    printf("Thread UDP-ESP32 Master OCU\n");

    int cal_lsm=0;
    int cal_cnt=0;
    int loss_cnt=0;
    while (1){
        if(loss_cnt++>500)
            ocu.esp32_connect=0;
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num >0)
        {
            //printf("recv_num=%d\\n",recv_num);
            memcpy(&_msg_ocu_tinker,recv_buf,sizeof(_msg_ocu_tinker));
            if(_msg_ocu_tinker.Head1==0xFA&&_msg_ocu_tinker.Head2==0XFB){
                ocu.esp32_connect=1;
                loss_cnt=0;
                ocu.key_ud=0;
                ocu.key_x=0;
                ocu.key_y=0;
                ocu.key_a=0;
                ocu.key_b=0;
                ocu.auto_mode=_msg_ocu_tinker.auto_mode;
                if(_msg_ocu_tinker.gait_mode==99)//poweroff
                    ocu.key_ud=-1;
                else if(_msg_ocu_tinker.gait_mode==1 && gait_ww.state_gait>=0)//power on
                {

                    if(vmc_all.gait_mode==G_RL)
                        ;//ocu.key_a=1;
                    else
                        ocu.key_x=1;
                }else if(_msg_ocu_tinker.gait_mode==10 && gait_ww.state_gait>=0)//power on
                {

                    if(vmc_all.gait_mode==G_RL)
                        ocu.key_b=1;
                }
                else if(_msg_ocu_tinker.gait_mode==11 && gait_ww.state_gait>=0)//power on
                {

                    if(vmc_all.gait_mode==G_RL)
                         ocu.key_a=1;
                }

                vmc_all.tar_spd.x=_msg_ocu_tinker.vel_cmd[0]*0.7;
                vmc_all.tar_spd.y=_msg_ocu_tinker.vel_cmd[1]*0.5;
                vmc_all.tar_spd.z=_msg_ocu_tinker.vel_cmd[2]*57.3*4;
                //printf("%d  ocu.key_x=%d %f %f\n",_msg_ocu_tinker.gait_mode, ocu.key_x,ocu.rc_spd_w[Xr],ocu.rate_yaw_w);

                //calculation
                switch(cal_lsm){
                    case 0:
                        if(_msg_ocu_tinker.cal_mode==10){//cal leg all
                            cal_lsm++;cal_cnt=0;
                            leg_motor_all.reset_q=2;
                            robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                            printf("ESP32::leg cal all!\n");
                        }
                        else if(_msg_ocu_tinker.cal_mode==11){//cal leg sel
                            cal_lsm++;cal_cnt=0;
                            leg_motor_all.reset_q_div[_msg_ocu_tinker.cal_sel]=1;
                            robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                            printf("ESP32::leg cal sel=%d!\n",_msg_ocu_tinker.cal_sel);
                        }
                        else if(_msg_ocu_tinker.cal_mode==20){//cal arm all
                            cal_lsm++;cal_cnt=0;
                            arm_motor_all.reset_q=2;
                            robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                            printf("ESP32::arm cal all!\n");
                        }
                        else if(_msg_ocu_tinker.cal_mode==21){//cal arm sel
                            cal_lsm++;cal_cnt=0;
                            arm_motor_all.reset_q_div[_msg_ocu_tinker.cal_sel]=1;
                            robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                            printf("ESP32::arm cal sel=%d!\n",_msg_ocu_tinker.cal_sel);
                        }
                        else if(_msg_ocu_tinker.cal_mode==30){//cal imu
                            cal_lsm++;cal_cnt=0;
                            robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                            leg_motor_all.reset_imu=_msg_ocu_tinker.cal_sel+1;
                            printf("ESP32::imu cal sel=%d!\n",leg_motor_all.reset_imu);
                        }
                    break;
                    case 1:
                        cal_cnt++;
                        if(cal_cnt>300){
                            cal_cnt=0;
                            leg_motor_all.reset_q=0;
                            leg_motor_all.reset_imu=0;
                            for(int id=0;id<14;id++)
                                leg_motor_all.reset_q_div[id]=arm_motor_all.reset_q_div[id]=0;
                            cal_lsm=0;
                            printf("ESP32::system cal done!\n");
                            robotwb.beep_state=0;
                        }
                    break;
                }
            }
            //printf("%d %d\n",leg_motor_all.reset_q_div[0],leg_motor_all.reset_q_div[1]);
            //---------------------feed back robot observation
            _msg_fb_tinker.att[0]=vmc_all.att[0];
            _msg_fb_tinker.att[1]=vmc_all.att[1];
            _msg_fb_tinker.att[2]=vmc_all.att[2];

            _msg_fb_tinker.vel[0]=vmc_all.tar_spd.x;
            _msg_fb_tinker.vel[1]=vmc_all.tar_spd.y;
            _msg_fb_tinker.vel[2]=vmc_all.tar_spd.z;

            for(int i=0;i<14;i++){
                _msg_fb_tinker.q_leg[i]=leg_motor_all.q_now[i];
                _msg_fb_tinker.q_arm[i]=arm_motor_all.q_now[i];
            }

            _msg_fb_tinker.heart_cnt++;

            if(gait_ww.state_gait==0)
                _msg_fb_tinker.gait_mode=0;
            else if(vmc_all.gait_mode==STAND_RC)
                _msg_fb_tinker.gait_mode=1;
            else if(vmc_all.gait_mode==G_RL)
                _msg_fb_tinker.gait_mode=2;

            _msg_fb_tinker.auto_mode=sdk.sdk_mode;

            memcpy(send_buf,&_msg_fb_tinker,sizeof(_msg_fb_tinker));
            send_num = sendto(sock_fd, send_buf, sizeof(_msg_fb_tinker), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
        }
        usleep(10*1000);//
    }
    close(sock_fd);
    return 0;
}

#define USE_RL 1
int main(int argc, char *argv[])
{
    pthread_t tid_servo, tid_control, tid_nav, tid_mpc,tid_map,tid_rl,tid_ocu,tid_t[10];
#if 0
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(3, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask) < 0) {
    perror("sched_setaffinity");
    }
#endif
    pthread_mutex_init(&lock, NULL);
    pthread_mutex_init(&lock_mpc, NULL);

#if MUL_THREAD&&!THREAD_2
    pthread_create(&tid_servo, NULL, Thread_Mem_Servo, NULL);
#endif

#if !THREAD_2
    pthread_create(&tid_nav, NULL, Thread_Mem_Navigation, NULL);
#endif
#if !THREAD_1
    pthread_create(&tid_t[0], NULL, Thread_T5, NULL);
    pthread_create(&tid_t[1], NULL, Thread_T1, NULL);
#endif

#if USE_RL&&!RL_USE_TVM
    pthread_create(&tid_rl, NULL, Thread_UDP_RL_Tinker, NULL);
    //pthread_create(&tid_rl, NULL, Thread_UDP_RL_Tinymal, NULL);
#endif
    pthread_create(&tid_ocu, NULL, Thread_UDP_OCU, NULL);
#if MUL_THREAD
    pthread_join(tid_servo, NULL);
#endif

#if !THREAD_1
    pthread_join(tid_t[0], NULL);
    pthread_join(tid_t[1], NULL);
#endif
#if !THREAD_2
    pthread_join(tid_nav, NULL);
#endif
#if USE_RL
    pthread_join(tid_rl, NULL);
#endif
    pthread_join(tid_ocu, NULL);
    return 1;
}
