#include "include.h"
#include "locomotion_header.h"
#include "can.h"
#include "gait_math.h"
#if RUN_WEBOTS&&!RUN_PI
#include "wbInterface.h"
#endif
#if RUN_PI
#include "spi_node.h"
#endif
_TD4 bldc_td4[4][3];
float FLT_ATT_RATE=0;//WS
#define USE_TD4_RATE 1

void subscribe_imu_to_webot(robotTypeDef* rob,float dt)
{
    if (vmc_all.side_flip == 0) {
            robotwb.IMU_now.pitch= robotwb.IMU_now_o.pitch;
            robotwb.IMU_now.roll= robotwb.IMU_now_o.roll;
            robotwb.IMU_now.yaw= robotwb.IMU_now_o.yaw;
            robotwb.IMU_dot.pitch= robotwb.IMU_dot_o.pitch;
            robotwb.IMU_dot.roll= robotwb.IMU_dot_o.roll;
            robotwb.IMU_dot.yaw= robotwb.IMU_dot_o.yaw;
    }
    else {//翻转  机头方向不变
        if (robotwb.IMU_now_o.pitch > 0)
             robotwb.IMU_now.pitch =  (180 - robotwb.IMU_now_o.pitch);
        else
            robotwb.IMU_now.pitch  =  -(180 + robotwb.IMU_now_o.pitch);

        robotwb.IMU_now.roll= -robotwb.IMU_now_o.roll;

        if(robotwb.IMU_now_o.yaw>0)
            robotwb.IMU_now.yaw= -(180-robotwb.IMU_now_o.yaw);
        else
            robotwb.IMU_now.yaw= (180+robotwb.IMU_now_o.yaw);

        robotwb.IMU_dot.pitch= -robotwb.IMU_dot_o.pitch;
        robotwb.IMU_dot.roll= robotwb.IMU_dot_o.roll;
        robotwb.IMU_dot.yaw= -robotwb.IMU_dot_o.yaw;
    }

    vmc_all.att[PITr]= robotwb.IMU_now.pitch;
    vmc_all.att[ROLr]= robotwb.IMU_now.roll;
    vmc_all.att[YAWr]= robotwb.IMU_now.yaw;
#if FLIP_TEST&&0
    printf("pit=%f rol=%f yaw=%f\n",vmc_all.att[PITr],vmc_all.att[ROLr],vmc_all.att[YAWr]);
    printf("dpit=%f drol=%f dyaw=%f\n",vmc_all.att_rate[PITr],vmc_all.att_rate[ROLr],vmc_all.att_rate[YAWr]);
#endif
    vmc_all.att_rate[PITr]=robotwb.IMU_dot.pitch;
    vmc_all.att_rate[ROLr]=robotwb.IMU_dot.roll;
    vmc_all.att_rate[YAWr]=robotwb.IMU_dot.yaw;

    static float att_rt_use[3] = {0};
    DigitalLPF(vmc_all.att[PITr], &att_rt_use[PITr], 0, dt);
    DigitalLPF(vmc_all.att[ROLr], &att_rt_use[ROLr], 0, dt);
    DigitalLPF(vmc_all.att[YAWr], &att_rt_use[YAWr], 0, dt);

    vmc_all.Rnn_b[0][0] = cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rnn_b[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rnn_b[2][0] = sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);

    vmc_all.Rnn_b[0][1] = cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rnn_b[1][1] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rnn_b[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);

    vmc_all.Rnn_b[0][2] = -sind(-att_rt_use[PITr]);
    vmc_all.Rnn_b[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    vmc_all.Rnn_b[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(vmc_all.Rnn_b, vmc_all.Rb_nn);

    att_rt_use[YAWr] = 0;
    vmc_all.Rn_b[0][0] = cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rn_b[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rn_b[2][0] = sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);

    vmc_all.Rn_b[0][1] = cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rn_b[1][1] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rn_b[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);

    vmc_all.Rn_b[0][2] = -sind(-att_rt_use[PITr]);
    vmc_all.Rn_b[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    vmc_all.Rn_b[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(vmc_all.Rn_b, vmc_all.Rb_n);

    if(vmc_all.param.leg_dof==2)
        att_rt_use[ROLr]=0;
    att_rt_use[YAWr]=0;
    vmc_all.Rn_b_noroll[0][0] =  cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rn_b_noroll[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr])+sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])  ;
    vmc_all.Rn_b_noroll[2][0] =  sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr])+cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr])  ;

    vmc_all.Rn_b_noroll[0][1] =  cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rn_b_noroll[1][1] =  cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])+sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr])  ;
    vmc_all.Rn_b_noroll[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])+cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr])  ;

    vmc_all.Rn_b_noroll[0][2] = -sind(-att_rt_use[PITr]);
    vmc_all.Rn_b_noroll[1][2] =  sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    vmc_all.Rn_b_noroll[2][2] =  cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(vmc_all.Rn_b_noroll,vmc_all.Rb_n_noroll);

    if(vmc_all.param.leg_dof==3){
        att_rt_use[PITr] = vmc_all.ground_att_est[PITr] * -1;
        att_rt_use[ROLr] = vmc_all.ground_att_est[ROLr] * -1;
#if 1//no ground matrix convter
        att_rt_use[PITr] = vmc_all.ground_att_est[PITr] * 0;
        att_rt_use[ROLr] = vmc_all.ground_att_est[ROLr] * 0;
#endif
    }
    else{
        att_rt_use[PITr] = vmc_all.ground_att_est[PITr];
        att_rt_use[ROLr] = 0;
    }
    att_rt_use[YAWr] = 0;

    vmc_all.Rn_g[0][0] = cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rn_g[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]);
    vmc_all.Rn_g[2][0] = sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);

    vmc_all.Rn_g[0][1] = cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rn_g[1][1] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
    vmc_all.Rn_g[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);

    vmc_all.Rn_g[0][2] = -sind(-att_rt_use[PITr]);
    vmc_all.Rn_g[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    vmc_all.Rn_g[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(vmc_all.Rn_g, vmc_all.Rg_n);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++) {
                robotwb.Rb_n[i][j] = vmc_all.Rb_n[i][j];
                robotwb.Rn_b[i][j] = vmc_all.Rn_b[i][j];
                robotwb.Rb_n_noroll[i][j] = vmc_all.Rb_n_noroll[i][j];
                robotwb.Rn_b_noroll[i][j] = vmc_all.Rn_b_noroll[i][j];
                robotwb.Rg_n[i][j] = vmc_all.Rg_n[i][j];
                robotwb.Rn_g[i][j] = vmc_all.Rn_g[i][j];
            }
    }
    //convert acc
    Vect3 body_acc,world_acc;
    if (vmc_all.side_flip == 0) {
        body_acc.x=vmc_all.acc_b.x=spi_rx.acc_b[1]*9.81;
        body_acc.y=vmc_all.acc_b.y=spi_rx.acc_b[0]*9.81;
        body_acc.z=vmc_all.acc_b.z=spi_rx.acc_b[2]*9.81;
    }else{
        body_acc.x=vmc_all.acc_b.x=spi_rx.acc_b[1]*9.81;
        body_acc.y=vmc_all.acc_b.y=-spi_rx.acc_b[0]*9.81;
        body_acc.z=vmc_all.acc_b.z=-spi_rx.acc_b[2]*9.81;
    }
    converV_b_to_nw(body_acc,&world_acc);

    vmc_all.acc_n.x=world_acc.x;
    vmc_all.acc_n.y=world_acc.y;
    vmc_all.acc_n.z=world_acc.z-9.81;

    att_rt_use[PITr]=0;
    att_rt_use[ROLr]=0;
    att_rt_use[YAWr]=0;//-vmc_all.att[YAWr];

    vmc_all.acc_nn.x =  spi_rx.acc_n[0] * cosd(att_rt_use[YAWr]) -  (-spi_rx.acc_n[1]) * sind(att_rt_use[YAWr]);
    vmc_all.acc_nn.y =  spi_rx.acc_n[0] * sind(att_rt_use[YAWr]) +  (-spi_rx.acc_n[1]) * cosd(att_rt_use[YAWr]);
    vmc_all.acc_nn.z =  spi_rx.acc_n[2];

    if(!spi_rx.imu_mems_connect&&1)//fix acc-n use feedback from IMU onboard
    {
        vmc_all.acc_n=vmc_all.acc_nn;
    }
   // printf("body  acc=%f %f %f\n",vmc_all.acc_b.x,vmc_all.acc_b.y,vmc_all.acc_b.z);
   // printf("world acc=%f %f %f\n",vmc_all.acc_n.x,vmc_all.acc_n.y,vmc_all.acc_n.z);

    robotwb.now_att = robotwb.IMU_now;
    robotwb.now_rate =robotwb.IMU_dot;
#if 0
    vmc_all.att_measure_bias[PITr]+=dead(ocu.rc_att_w[PITr],0.1)*dt*0.1;
    vmc_all.att_measure_bias[ROLr]+=dead(ocu.rc_att_w[ROLr],0.1)*dt*0.1;
    vmc_all.att_measure_bias[PITr]=LIMIT(vmc_all.att_measure_bias[PITr],-3,3);
    vmc_all.att_measure_bias[ROLr]=LIMIT(vmc_all.att_measure_bias[ROLr],-3,3);
    printf("bais:%f %f\n",vmc_all.att_measure_bias[PITr],vmc_all.att_measure_bias[ROLr]);
#endif
}

void subscribe_webot_to_vmc(float dt)
{
  int i=0,j=0;
  for(i=0;i<4;i++)
  {
        #if Q_NOW_USE_SET
        vmc[i].sita1=vmc[i].tar_sita1;
        vmc[i].sita2=vmc[i].tar_sita2;
        #else
        if(vmc[i].param.q_now_use_tar){
            vmc[i].sita1=vmc[i].tar_sita1;
            vmc[i].sita2=vmc[i].tar_sita2;
            vmc[i].sita3=vmc[i].tar_sita3;
        }else{
            if(vmc_all.param.leg_dof==3){
                vmc[i].sita1 = robotwb.Leg[i].sita[0];
                vmc[i].sita2 = robotwb.Leg[i].sita[1];
                vmc[i].sita3 = robotwb.Leg[i].sita[2];
                vmc[i].dsita1= robotwb.Leg[i].sita_d_flt[0];
                vmc[i].dsita2= robotwb.Leg[i].sita_d_flt[1];
                vmc[i].dsita3= robotwb.Leg[i].sita_d_flt[2];
            }else{
                vmc[i].sita1=robotwb.Leg[i].sita[0];
                vmc[i].sita2=robotwb.Leg[i].sita[1];
                if(vmc_all.side_flip)
                {
                    if(i==0||i==1)
                        vmc[i].sita3=Q_SIDE_BIAS;//
                    else
                        vmc[i].sita3=-Q_SIDE_BIAS;
                }else{
                    if(i==0||i==1)
                        vmc[i].sita3=-Q_SIDE_BIAS;//
                    else
                        vmc[i].sita3=Q_SIDE_BIAS;
                }
                //vmc[i].sita3=robotwb.Leg[i].sita[2];
    #if USE_TD4_RATE
                vmc[i].dsita1=robotwb.Leg[i].sita_d_flt[0];
                vmc[i].dsita2=robotwb.Leg[i].sita_d_flt[1];
                vmc[i].dsita3=robotwb.Leg[i].sita_d_flt[2];
    #else
                vmc[i].dsita1=robotwb.Leg[i].sita_d[0];
                vmc[i].dsita2=robotwb.Leg[i].sita_d[1];
                vmc[i].dsita3=robotwb.Leg[i].sita_d[2];
    #endif
            }
        }
        #endif

    vmc[i].delta_ht=vmc_robot_p.sw_deltah;
    robotwb.Leg[i].is_ground= vmc[i].ground;
    robotwb.Leg[i].trig_state=vmc[i].param.trig_state;

    #if !GROUND_USE_EST//
    vmc[i].is_touch=robotwb.Leg[i].is_touch;
    #else
    vmc[i].is_touch=robotwb.Leg[i].is_touch_est;
    #endif

        robotwb.Leg[i].limit_tao[0]=pos_force_p.max_t_d[0];//pos_force_p.max_t;
        robotwb.Leg[i].limit_tao[1]=pos_force_p.max_t_d[1];//pos_force_p.max_t;
        robotwb.Leg[i].limit_tao[2]=pos_force_p.max_t_d[2];//pos_force_p.max_t;

         if(vmc_all.gait_mode==TROT){
             robotwb.Leg[i].f_pos_pid[Xr]=pos_force_p.f_pos_pid_st[Xr];
             robotwb.Leg[i].f_pos_pid[Yr]=pos_force_p.f_pos_pid_st[Yr];
             robotwb.Leg[i].f_pos_pid[Zr]=pos_force_p.f_pos_pid_st[Zr];
         }else
         {
             robotwb.Leg[i].f_pos_pid[Xr]=pos_force_p.f_pos_pid_st[Xr];
             robotwb.Leg[i].f_pos_pid[Yr]=pos_force_p.f_pos_pid_st[Yr];
             robotwb.Leg[i].f_pos_pid[Zr]=pos_force_p.f_pos_pid_st[Zr];
         }
        // printf("robotwb.Leg[i].f_pos_pid[Zr].kp=%f\n",robotwb.Leg[i].f_pos_pid[Zr].kp);
        robotwb.Leg[i].f_pid[Xr]=pos_force_p.f_pid_st[Xr];
        robotwb.Leg[i].f_pid[Yr]=pos_force_p.f_pid_st[Yr];
        robotwb.Leg[i].f_pid[Zr]=pos_force_p.f_pid_st[Zr];

        //-------------------------joint PID param all set
        if(vmc_all.gait_mode==TROT)
        {
                if(stand_force_enable_flag[4]&&vmc[i].ground){
                    robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_trot;
                }
                else
                {
                    robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;
                }
        }else{
            //rintf("vmc_all.pos_n.z=%f %f\n",vmc_all.pos_n.z,fabs(MAX_Z)*0.4);
            if(vmc_all.pos_n.z<fabs(MAX_Z)*0.6&&0){
                robotwb.Leg[i].q_pid=pos_force_p.q_pid_init;
            }else{
                if(robotwb.Leg[i].is_ground){
                robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_stance;
                }
                else
                {
                robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;
                }
            }
        }

        //param divde<<=====================New version===============================
        if(vmc_all.gait_mode==TROT)
        {
            robotwb.Leg[i].q_pid.kp_st=pos_force_p.q_pid_st_trot.kp;
            robotwb.Leg[i].q_pid.ki_st=pos_force_p.q_pid_st_trot.ki;
            robotwb.Leg[i].q_pid.kd_st=pos_force_p.q_pid_st_trot.kd;//pos_force_p.q_pid_st_trot  = pos_force_p.q_pid_st_stance; same as stance

            for(int j=0;j<3;j++){
                robotwb.Leg[i].q_pid.kp_st_d[j]=pos_force_p.q_pid_st_trot.kp_d[j];
                robotwb.Leg[i].q_pid.ki_st_d[j]=pos_force_p.q_pid_st_trot.ki_d[j];
                robotwb.Leg[i].q_pid.kd_st_d[j]=pos_force_p.q_pid_st_trot.kd_d[j];
            }

            if(stand_force_enable_flag[4]&&vmc[i].ground){
                robotwb.Leg[i].q_pid.param_sel=1;
            }
            else
            {
                robotwb.Leg[i].q_pid.param_sel=0;
            }

        }else{//Stand & init----------------------------------------------------------------------------------------
            if(vmc_all.pos_n.z<fabs(MAX_Z)*0.6&&0){
                robotwb.Leg[i].q_pid.kp_st=pos_force_p.q_pid_init.kp;
                robotwb.Leg[i].q_pid.ki_st=pos_force_p.q_pid_init.ki;
                robotwb.Leg[i].q_pid.kd_st=pos_force_p.q_pid_init.kd;

                for(int j=0;j<3;j++){
                    robotwb.Leg[i].q_pid.kp_st_d[j]=pos_force_p.q_pid_init.kp;
                    robotwb.Leg[i].q_pid.ki_st_d[j]=pos_force_p.q_pid_init.ki;
                    robotwb.Leg[i].q_pid.kd_st_d[j]=pos_force_p.q_pid_init.kd;
                }
            }else{
                robotwb.Leg[i].q_pid.kp_st=pos_force_p.q_pid_st_stance.kp;
                robotwb.Leg[i].q_pid.ki_st=pos_force_p.q_pid_st_stance.ki;
                robotwb.Leg[i].q_pid.kd_st=pos_force_p.q_pid_st_stance.kd;

                for(int j=0;j<3;j++){
                    robotwb.Leg[i].q_pid.kp_st_d[j]=pos_force_p.q_pid_st_stance.kp_d[j];
                    robotwb.Leg[i].q_pid.ki_st_d[j]=pos_force_p.q_pid_st_stance.ki_d[j];
                    robotwb.Leg[i].q_pid.kd_st_d[j]=pos_force_p.q_pid_st_stance.kd_d[j];
                }
            }

            if(stand_force_enable_flag[4]&&vmc[i].ground){
                robotwb.Leg[i].q_pid.param_sel=1;
            }
            else
            {
                robotwb.Leg[i].q_pid.param_sel=0;
            }
        }
//--------------------------------------sw param-------------------------------------------------
        float swing_pid_scale=1;
        if(vmc[i].param.trig_state>=4)//td sync & load
            swing_pid_scale=0.5;

        robotwb.Leg[i].q_pid.kp_sw=pos_force_p.q_pid_sw.kp*swing_pid_scale;
        robotwb.Leg[i].q_pid.ki_sw=pos_force_p.q_pid_sw.ki;
        robotwb.Leg[i].q_pid.kd_sw=pos_force_p.q_pid_sw.kd;

        for(int j=0;j<3;j++){
            robotwb.Leg[i].q_pid.kp_sw_d[j]=pos_force_p.q_pid_sw.kp_d[j];
            robotwb.Leg[i].q_pid.ki_sw_d[j]=pos_force_p.q_pid_sw.ki_d[j];
            robotwb.Leg[i].q_pid.kd_sw_d[j]=pos_force_p.q_pid_sw.kd_d[j];
        }

        leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=pos_force_p.max_t;
        leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=pos_force_p.max_i;

  }
  //---canshu
#if 0
  leg_motor_all.max_t[0]=leg_motor[0].max_t[0];
  leg_motor_all.max_t[1]=leg_motor[0].max_t[1];
  leg_motor_all.max_t[2]=leg_motor[0].max_t[2];
  leg_motor_all.max_t[3]=leg_motor[1].max_t[0];
  leg_motor_all.max_t[4]=leg_motor[1].max_t[1];
  leg_motor_all.max_t[5]=leg_motor[1].max_t[2];
  leg_motor_all.max_t[6]=leg_motor[2].max_t[0];
  leg_motor_all.max_t[7]=leg_motor[2].max_t[1];
  leg_motor_all.max_t[8]=leg_motor[2].max_t[2];
  leg_motor_all.max_t[9]=leg_motor[3].max_t[0];
  leg_motor_all.max_t[10]=leg_motor[3].max_t[1];
  leg_motor_all.max_t[11]=leg_motor[3].max_t[2];

  leg_motor_all.kp[0]=robotwb.Leg[0].q_pid.kp_sw_d[0];
  leg_motor_all.kp[1]=robotwb.Leg[0].q_pid.kp_sw_d[1];
  leg_motor_all.kp[2]=robotwb.Leg[0].q_pid.kp_sw_d[2];
  leg_motor_all.kp[3]=robotwb.Leg[1].q_pid.kp_sw_d[0];
  leg_motor_all.kp[4]=robotwb.Leg[1].q_pid.kp_sw_d[1];
  leg_motor_all.kp[5]=robotwb.Leg[1].q_pid.kp_sw_d[2];
  leg_motor_all.kp[6]=robotwb.Leg[2].q_pid.kp_sw_d[0];
  leg_motor_all.kp[7]=robotwb.Leg[2].q_pid.kp_sw_d[1];
  leg_motor_all.kp[8]=robotwb.Leg[2].q_pid.kp_sw_d[2];
  leg_motor_all.kp[9]=robotwb.Leg[3].q_pid.kp_sw_d[0];
  leg_motor_all.kp[10]=robotwb.Leg[3].q_pid.kp_sw_d[1];
  leg_motor_all.kp[11]=robotwb.Leg[3].q_pid.kp_sw_d[2];

  leg_motor_all.kd[0]=robotwb.Leg[0].q_pid.kd_sw_d[0];
  leg_motor_all.kd[1]=robotwb.Leg[0].q_pid.kd_sw_d[1];
  leg_motor_all.kd[2]=robotwb.Leg[0].q_pid.kd_sw_d[2];
  leg_motor_all.kd[3]=robotwb.Leg[1].q_pid.kd_sw_d[0];
  leg_motor_all.kd[4]=robotwb.Leg[1].q_pid.kd_sw_d[1];
  leg_motor_all.kd[5]=robotwb.Leg[1].q_pid.kd_sw_d[2];
  leg_motor_all.kd[6]=robotwb.Leg[2].q_pid.kd_sw_d[0];
  leg_motor_all.kd[7]=robotwb.Leg[2].q_pid.kd_sw_d[1];
  leg_motor_all.kd[8]=robotwb.Leg[2].q_pid.kd_sw_d[2];
  leg_motor_all.kd[9]=robotwb.Leg[3].q_pid.kd_sw_d[0];
  leg_motor_all.kd[10]=robotwb.Leg[3].q_pid.kd_sw_d[1];
  leg_motor_all.kd[11]=robotwb.Leg[3].q_pid.kd_sw_d[2];
#endif

  //地形角期望命令
  if(vmc_all.param.leg_dof==3){
      DigitalLPF( vmc_all.ground_att_est[PITr], &vmc_all.ground_att_cmd[PITr] , 1, dt);
      DigitalLPF(-my_deathzoom_2(vmc_all.ground_att_est[ROLr],3)*1, &vmc_all.ground_att_cmd[ROLr] , 1, dt);
  }else
     vmc_all.ground_att_cmd[PITr]=vmc_all.ground_att_est[PITr];

  robotwb.exp_att.pitch = vmc_all.tar_att[PITr] + LIMIT(dead(vmc_all.ground_att_cmd[PITr],0)  * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -35, 35);
  robotwb.exp_att.roll  = vmc_all.tar_att[ROLr] + LIMIT(dead(vmc_all.ground_att_cmd[ROLr],4)  * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -15, 15);
}

void publish_vmc_to_webot(float dt)
{
    int i=0;
    for(i=0;i<4;i++){
        float alfa=(robotwb.Leg[i].sita[1]-robotwb.Leg[i].sita[0])/2;
        float beta=(robotwb.Leg[i].sita[1]+robotwb.Leg[i].sita[0])/2;
        robotwb.Leg[i].beta=beta;
        robotwb.Leg[i].alfa=alfa;

        robotwb.Leg[i].r=vmc[i].r;
        robotwb.Leg[i].sita_r=vmc[i].sita;
        robotwb.Leg[i].epos_h.x=vmc[i].epos.x;
        robotwb.Leg[i].epos_h.y=vmc[i].epos.y;
        robotwb.Leg[i].epos_h.z=vmc[i].epos.z;

        robotwb.Leg[i].epos_b.x=vmc[i].epos_b.x;
        robotwb.Leg[i].epos_b.y=vmc[i].epos_b.y;
        robotwb.Leg[i].epos_b.z=vmc[i].epos_b.z;

        robotwb.Leg[i].epos_n.x=vmc[i].epos_n.x;
        robotwb.Leg[i].epos_n.y=vmc[i].epos_n.y;
        robotwb.Leg[i].epos_n.z=vmc[i].epos_n.z;

        robotwb.Leg[i].espd_h.x=vmc[i].spd.x;
        robotwb.Leg[i].espd_h.y=vmc[i].spd.y;
        robotwb.Leg[i].espd_h.z=vmc[i].spd.z;

        robotwb.Leg[i].espd_n.x=vmc[i].spd_n.x;
        robotwb.Leg[i].espd_n.y=vmc[i].spd_n.y;
        robotwb.Leg[i].espd_n.z=vmc[i].spd_n.z;

        if(vmc_all.param.leg_dof==3){
            for (int j = 0; j < 9; j++) {
                robotwb.Leg[i].jacobi[j] = vmc[i].jacobi33[j];
                robotwb.Leg[i].jacobi_inv[j] = vmc[i].ijacobi33[j];
            }

            for (int j = 0; j < 9; j++) {
                robotwb.Leg[i].jacobi_kin1[j] = vmc[i].jacobi33_kin1[j];
                robotwb.Leg[i].jacobi_inv_kin1[j] = vmc[i].ijacobi33_kin1[j];
            }

            for (int j = 0; j < 9; j++) {
                robotwb.Leg[i].jacobi_kin12[j] = vmc[i].jacobi33_kin12[j];
                robotwb.Leg[i].jacobi_inv_kin12[j] = vmc[i].ijacobi33_kin12[j];
            }

            for (int j = 0; j < 9; j++) {
                robotwb.Leg[i].jacobi_kin2[j] = vmc[i].jacobi33_kin2[j];
                robotwb.Leg[i].jacobi_inv_kin2[j] = vmc[i].ijacobi33_kin2[j];
            }

            for(int j=0;j<9;j++){
                robotwb.Leg[i].jacobi33[j]=vmc[i].jacobi33[j];
                robotwb.Leg[i].jacobi_inv33[j]=vmc[i].ijacobi33[j];
            }
        }else
        {
            robotwb.Leg[i].jacobi[0]=vmc[i].jacobi22[0];
            robotwb.Leg[i].jacobi[2]=vmc[i].jacobi22[2];

            robotwb.Leg[i].jacobi[1]=vmc[i].jacobi22[1];
            robotwb.Leg[i].jacobi[3]=vmc[i].jacobi22[3];

            robotwb.Leg[i].jacobi_inv[0]=vmc[i].ijacobi22[0];
            robotwb.Leg[i].jacobi_inv[2]=vmc[i].ijacobi22[2];

            robotwb.Leg[i].jacobi_inv[1]=vmc[i].ijacobi22[1];
            robotwb.Leg[i].jacobi_inv[3]=vmc[i].ijacobi22[3];

            for(int j=0;j<9;j++){
                robotwb.Leg[i].jacobi33[j]=vmc[i].jacobi33[j];
                robotwb.Leg[i].jacobi_inv33[j]=vmc[i].ijacobi33[j];
            }
        }
    }
}

void readAllMotorPos(robotTypeDef* rob,float dt)
{
    static float cnt;
    uint8_t i, j;
    static char init = 0;
    dt = LIMIT(dt, 0.001, 0.1);
    float taom[4][3] = { 0 };
    float temp_sita[4][3] = { 0 };
    char id_swap_flig[4]={2,3,0,1};
    float td4_rate = 330;
    float ff_rate_observe=1;
#if USE_TD4_RATE
    int MID_RATE=0;
#else
    int MID_RATE=3;
#endif
    if(!init){
        init=1;
        for (int i = 0; i < 4; i++){
         TD4_init(&bldc_td4[i][0], td4_rate, td4_rate, td4_rate, td4_rate);
         TD4_init(&bldc_td4[i][1], td4_rate, td4_rate, td4_rate, td4_rate);
         TD4_init(&bldc_td4[i][2], td4_rate, td4_rate, td4_rate, td4_rate);
        }
    }
    leg_motor[0].connect=leg_motor_all.connect;
    leg_motor[0].connect_motor[0]=leg_motor_all.connect_motor[0];
    leg_motor[0].connect_motor[1]=leg_motor_all.connect_motor[1];
    leg_motor[0].connect_motor[2]=leg_motor_all.connect_motor[2];
    leg_motor[1].connect_motor[0]=leg_motor_all.connect_motor[3];
    leg_motor[1].connect_motor[1]=leg_motor_all.connect_motor[4];
    leg_motor[1].connect_motor[2]=leg_motor_all.connect_motor[5];
    leg_motor[2].connect_motor[0]=leg_motor_all.connect_motor[6];
    leg_motor[2].connect_motor[1]=leg_motor_all.connect_motor[7];
    leg_motor[2].connect_motor[2]=leg_motor_all.connect_motor[8];
    leg_motor[3].connect_motor[0]=leg_motor_all.connect_motor[9];
    leg_motor[3].connect_motor[1]=leg_motor_all.connect_motor[10];
    leg_motor[3].connect_motor[2]=leg_motor_all.connect_motor[11];
    leg_motor[0].ready[0]=leg_motor_all.ready[0];
    leg_motor[0].ready[1]=leg_motor_all.ready[1];
    leg_motor[0].ready[2]=leg_motor_all.ready[2];
    leg_motor[1].ready[0]=leg_motor_all.ready[3];
    leg_motor[1].ready[1]=leg_motor_all.ready[4];
    leg_motor[1].ready[2]=leg_motor_all.ready[5];
    leg_motor[2].ready[0]=leg_motor_all.ready[6];
    leg_motor[2].ready[1]=leg_motor_all.ready[7];
    leg_motor[2].ready[2]=leg_motor_all.ready[8];
    leg_motor[3].ready[0]=leg_motor_all.ready[9];
    leg_motor[3].ready[1]=leg_motor_all.ready[10];
    leg_motor[3].ready[2]=leg_motor_all.ready[11];

    leg_motor[0].q_now[0]=leg_motor_all.q_now[0];
    leg_motor[0].q_now[1]=leg_motor_all.q_now[1];
    leg_motor[0].q_now[2]=leg_motor_all.q_now[2];
    leg_motor[1].q_now[0]=leg_motor_all.q_now[3];
    leg_motor[1].q_now[1]=leg_motor_all.q_now[4];
    leg_motor[1].q_now[2]=leg_motor_all.q_now[5];
    leg_motor[2].q_now[0]=leg_motor_all.q_now[6];
    leg_motor[2].q_now[1]=leg_motor_all.q_now[7];
    leg_motor[2].q_now[2]=leg_motor_all.q_now[8];
    leg_motor[3].q_now[0]=leg_motor_all.q_now[9];
    leg_motor[3].q_now[1]=leg_motor_all.q_now[10];
    leg_motor[3].q_now[2]=leg_motor_all.q_now[11];

    leg_motor[0].t_now[0]=leg_motor_all.t_now[0];
    leg_motor[0].t_now[1]=leg_motor_all.t_now[1];
    leg_motor[0].t_now[2]=leg_motor_all.t_now[2];
    leg_motor[1].t_now[0]=leg_motor_all.t_now[3];
    leg_motor[1].t_now[1]=leg_motor_all.t_now[4];
    leg_motor[1].t_now[2]=leg_motor_all.t_now[5];
    leg_motor[2].t_now[0]=leg_motor_all.t_now[6];
    leg_motor[2].t_now[1]=leg_motor_all.t_now[7];
    leg_motor[2].t_now[2]=leg_motor_all.t_now[8];
    leg_motor[3].t_now[0]=leg_motor_all.t_now[9];
    leg_motor[3].t_now[1]=leg_motor_all.t_now[10];
    leg_motor[3].t_now[2]=leg_motor_all.t_now[11];
    cnt+=dt;
    for (i = 0; i < 4; i++){//
                if(leg_motor[i].connect&&//q0
                    leg_motor[i].connect_motor[0]&&
                  leg_motor[i].ready[0]){
                    if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3) {
                        rob->Leg[i].sita[0]=(leg_motor[i].q_now[0]);//sita0
                        temp_sita[i][0]=leg_motor[i].q_now[0];
                        rob->Leg[i].taom[0]=leg_motor[i].t_now[0];
                    }else{//flip
                        rob->Leg[id_swap_flig[i]].sita[0]=(leg_motor[i].q_now[1]);//1->0

                        if (rob->Leg[id_swap_flig[i]].sita[0] < 180)
                            rob->Leg[id_swap_flig[i]].sita[0] = -rob->Leg[id_swap_flig[i]].sita[0];
                        else
                            rob->Leg[id_swap_flig[i]].sita[0] = 360 - rob->Leg[id_swap_flig[i]].sita[0];

                        temp_sita[id_swap_flig[i]][0]=rob->Leg[id_swap_flig[i]].sita[0];
                        rob->Leg[id_swap_flig[i]].taom[0]=-leg_motor[i].t_now[1];
                    }
                }else{//
                    if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3) {
                        rob->Leg[i].sita[0]=0;
                        rob->Leg[i].taom[0]=0;
                        temp_sita[i][0]=0;
                    }else{
                        rob->Leg[id_swap_flig[i]].sita[0]=0;
                        rob->Leg[id_swap_flig[i]].taom[0]=0;
                        temp_sita[id_swap_flig[i]][0]=0;
                    }
                }
            //------------------------------------------------------------------------
                if(leg_motor[i].connect&&//q1
                  leg_motor[i].connect_motor[1]&&
                  leg_motor[i].ready[1]){
                     if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3) {
                            rob->Leg[i].sita[1]=To_360_degreesw(leg_motor[i].q_now[1]);//sita1 0~360
                            temp_sita[i][1]=leg_motor[i].q_now[1];
                            rob->Leg[i].taom[1]=leg_motor[i].t_now[1];
                     }else{//flip
                         rob->Leg[id_swap_flig[i]].sita[1]=(leg_motor[i].q_now[0]);//0->1

                         if (rob->Leg[id_swap_flig[i]].sita[1] < 180)
                             rob->Leg[id_swap_flig[i]].sita[1] = -rob->Leg[id_swap_flig[i]].sita[1];
                         else
                             rob->Leg[id_swap_flig[i]].sita[1] = 360 - rob->Leg[id_swap_flig[i]].sita[1];

                         temp_sita[id_swap_flig[i]][1]=rob->Leg[id_swap_flig[i]].sita[1];
                         rob->Leg[id_swap_flig[i]].taom[1]=-leg_motor[i].t_now[0];
                     }
                  }else{//
                    if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3) {
                        rob->Leg[i].sita[1]=180;
                        rob->Leg[i].taom[1]=0;
                        temp_sita[i][1]=180;
                    }else{
                        rob->Leg[id_swap_flig[i]].sita[1]=180;
                        rob->Leg[id_swap_flig[i]].taom[1]=0;
                        temp_sita[id_swap_flig[i]][1]=180;
                    }
                   }
//------------------------------------------------------------------------
                if(leg_motor[i].connect&&//q2
                  leg_motor[i].connect_motor[2]&&
                  leg_motor[i].ready[2]){
                     if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3) {
                            rob->Leg[i].sita[2]=To_180_degreesw(leg_motor[i].q_now[2]);//sita1 0~360
                            temp_sita[i][2]=leg_motor[i].q_now[2];
                            rob->Leg[i].taom[2]=leg_motor[i].t_now[2];
                     }else{//flip
                         rob->Leg[id_swap_flig[i]].sita[2]=-To_180_degreesw(leg_motor[i].q_now[2]);//0->1
                         temp_sita[id_swap_flig[i]][2]=rob->Leg[id_swap_flig[i]].sita[2];
                         rob->Leg[id_swap_flig[i]].taom[2]=-leg_motor[i].t_now[2];
                     }
                  }else{//
                    if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3) {
                        rob->Leg[i].sita[2]=0;
                        rob->Leg[i].taom[2]=0;
                        temp_sita[i][2]=0;
                    }else{
                        rob->Leg[id_swap_flig[i]].sita[2]=0;
                        rob->Leg[id_swap_flig[i]].taom[2]=0;
                        temp_sita[id_swap_flig[i]][2]=0;
                    }
                }
                static int cnt_p[4];
//                if(cnt_p[i]++>25){cnt_p[i]=0;
//                printf("leg=%d q0=%f q1=%f q2=%f\n",i, rob->Leg[i].sita[0], rob->Leg[i].sita[1], rob->Leg[i].sita[2]);
//                printf("leg=%d m0=%f m1=%f m2=%f\n",i, rob->Leg[i].taom[0], rob->Leg[i].taom[1], rob->Leg[i].taom[2]);
//                }
        //rotate spd
        if (vmc_all.side_flip == 0||vmc_all.param.leg_dof==3){
                rob->Leg[i].sita_d[0]=Moving_Median(3*i,  MID_RATE,    To_180_degreesw(temp_sita[i][0]-rob->Leg[i].sita_reg[0])/dt);
                rob->Leg[i].sita_d[1]=Moving_Median(3*i+1,MID_RATE,	   To_180_degreesw(temp_sita[i][1]-rob->Leg[i].sita_reg[1])/dt);
                rob->Leg[i].sita_d[2]=Moving_Median(3*i+2,MID_RATE,	   To_180_degreesw(temp_sita[i][2]-rob->Leg[i].sita_reg[2])/dt);

                rob->Leg[i].sita_reg[0]=temp_sita[i][0];
                rob->Leg[i].sita_reg[1]=temp_sita[i][1];
                rob->Leg[i].sita_reg[2]=temp_sita[i][2];
#if USE_TD4_RATE
                TD4_track4( &bldc_td4[i][0] , rob->Leg[i].sita_d[0], dt);
                TD4_track4( &bldc_td4[i][1] , rob->Leg[i].sita_d[1], dt );
                TD4_track4( &bldc_td4[i][2] , rob->Leg[i].sita_d[2], dt );

                rob->Leg[i].sita_d_flt[0]=bldc_td4[i][0].x1+bldc_td4[i][0].x2*dt*ff_rate_observe;
                rob->Leg[i].sita_d_flt[1]=bldc_td4[i][1].x1+bldc_td4[i][1].x2*dt*ff_rate_observe;
                rob->Leg[i].sita_d_flt[2]=bldc_td4[i][2].x1+bldc_td4[i][2].x2*dt*ff_rate_observe;

                rob->Leg[i].sita_dd_flt[0]=bldc_td4[i][0].x2;
                rob->Leg[i].sita_dd_flt[1]=bldc_td4[i][1].x2;
                rob->Leg[i].sita_dd_flt[2]=bldc_td4[i][2].x2;
#endif
        }else{
                rob->Leg[id_swap_flig[i]].sita_d[0]=Moving_Median(3*id_swap_flig[i],  MID_RATE, To_180_degreesw(temp_sita[id_swap_flig[i]][0]-rob->Leg[id_swap_flig[i]].sita_reg[0])/dt);
                rob->Leg[id_swap_flig[i]].sita_d[1]=Moving_Median(3*id_swap_flig[i]+1,MID_RATE,	To_180_degreesw(temp_sita[id_swap_flig[i]][1]-rob->Leg[id_swap_flig[i]].sita_reg[1])/dt);
                rob->Leg[id_swap_flig[i]].sita_d[2]=Moving_Median(3*id_swap_flig[i]+2,MID_RATE,	To_180_degreesw(temp_sita[id_swap_flig[i]][2]-rob->Leg[id_swap_flig[i]].sita_reg[2])/dt);

                rob->Leg[id_swap_flig[i]].sita_reg[0]=temp_sita[id_swap_flig[i]][0];
                rob->Leg[id_swap_flig[i]].sita_reg[1]=temp_sita[id_swap_flig[i]][1];
                rob->Leg[id_swap_flig[i]].sita_reg[2]=temp_sita[id_swap_flig[i]][2];
#if USE_TD4_RATE
                TD4_track4( &bldc_td4[id_swap_flig[i]][0] , rob->Leg[id_swap_flig[i]].sita_d[0], dt);
                TD4_track4( &bldc_td4[id_swap_flig[i]][1] , rob->Leg[id_swap_flig[i]].sita_d[1], dt );
                TD4_track4( &bldc_td4[id_swap_flig[i]][2] , rob->Leg[id_swap_flig[i]].sita_d[2], dt );

                rob->Leg[id_swap_flig[i]].sita_d_flt[0]=bldc_td4[id_swap_flig[i]][0].x1+bldc_td4[id_swap_flig[i]][0].x2*dt*ff_rate_observe;
                rob->Leg[id_swap_flig[i]].sita_d_flt[1]=bldc_td4[id_swap_flig[i]][1].x1+bldc_td4[id_swap_flig[i]][1].x2*dt*ff_rate_observe;
                rob->Leg[id_swap_flig[i]].sita_d_flt[2]=bldc_td4[id_swap_flig[i]][2].x1+bldc_td4[id_swap_flig[i]][2].x2*dt*ff_rate_observe;
#endif
        }
    }
}

#if !RUN_WEBOTS||RUN_PI
void set_motor_q(int id)//nouse now
{
    if(vmc_all.param.leg_dof==3){
        robotwb.Leg[id].tar_sita[0] = LIMIT(robotwb.Leg[id].tar_sita[0], min_q1[id],max_q1[id]);//D
        robotwb.Leg[id].tar_sita[1] = LIMIT(robotwb.Leg[id].tar_sita[1], min_q2[id],max_q2[id]);//X
        robotwb.Leg[id].tar_sita[2] = LIMIT(robotwb.Leg[id].tar_sita[2], min_q3[id],max_q3[id]);//Z
    }else{
        robotwb.Leg[id].tar_sita[0] = LIMIT(robotwb.Leg[id].tar_sita[0],-robotwb.Leg[id].limit_sita[0],180+robotwb.Leg[id].limit_sita[0]);
        robotwb.Leg[id].tar_sita[1] = LIMIT(robotwb.Leg[id].tar_sita[1],-robotwb.Leg[id].limit_sita[1],180+robotwb.Leg[id].limit_sita[1]);
        robotwb.Leg[id].tar_sita[2] = LIMIT(robotwb.Leg[id].tar_sita[2],-robotwb.Leg[id].limit_sita[2],robotwb.Leg[id].limit_sita[2]);
    }
}

void set_motor_t(int id)
{

#if 0
    leg_motor_all.servo_en=1;
    leg_motor_all.q_set_servo[0]=0;
    leg_motor_all.q_set_servo[1]=0;
    leg_motor_all.q_set_servo[2]=0;
#endif
    float temp_in[14]={0};
    for(int i=0;i<14;i++)
        temp_in[i]=leg_motor_all.q_set_servo[i];

#if !HEAD_USE_DC//head for servo motor
    static float temp_att[3]={0};
    DigitalLPF(vmc_all.att_rate[YAWr]*0.45,&temp_att[2],0.1,0.001);
    DigitalLPF(vmc_all.att[PITr],&temp_att[0],0.5,0.001);
    //printf("temp_yaw=%f\n",temp_yaw);
    temp_in[0]+=LIMIT(temp_att[0],-25,25);
    temp_in[1]-=LIMIT(temp_att[0]/2,-25,25);
    temp_in[2]+=LIMIT(temp_att[2],-15,15);
#endif

    for(int i=0;i<14;i++){
        float temp=LIMIT(temp_in[i],leg_motor_all.q_servo_min[i],leg_motor_all.q_servo_max[i])
                +leg_motor_all.q_set_servo_off[i];

       DigitalLPF(temp,&leg_motor_all.q_set_servo_out[i],3,0.001);
    }

    for(int i=0;i<14;i++){
        if(leg_motor_all.servo_en){
            leg_motor_all.q_servo[i]=leg_motor_all.q_set_servo_out[i];
        }else{
            leg_motor_all.q_servo[i]=spi_rx.q_servo[i]-leg_motor_all.q_set_servo_off[i];
            leg_motor_all.t_servo[i]=spi_rx.tau_servo[i];
        }
    }
    //printf("%f %f %f\n",leg_motor_all.q_servo[0],leg_motor_all.q_servo[1],leg_motor_all.q_servo[2]);
}
#endif
