#ifndef __BASE_STRUCT_H__
#define __BASE_STRUCT_H__
#include "math.h"
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef signed   char  int8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef float  fp32;
typedef double fp64;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef int16_t s16;
typedef int8_t  s8;
extern double Pn_td[4][3];
extern float line_off[2];

#define USE_FF_N 0

#define TEST_KIN 0
#define TEST_KIN_ID 1

#define MISSION_DOG_FOLLOWING 0
#define MISSION_HUNMAN_GUIDE 1
//--------------------------------VMC库宏定义----------------------------------
#define EN_VISION                 0
#define USE_REAL_MAP              0
#define EN_KIN_DYN                1

#define WALK_USE_X                1

#define EN_SLOPE_HZ               1
#define EN_MOM_LIP                1
#define SW_USE_POS_N              0 //摆动转换全局
#define SWING_IN_NN               0 //摆动考虑里程计

#define USE_ZMP                   1
#define USE_IMP_PLAN              1

#define EN_SW_SLOP_ETH            0
#define FIX_SLOP_OFF              0  //度
#define SLOP_DEAD                 5  //度

#define EN_PLAN_USE_MIT           1
#define EN_KF_MIT                 1

#define SWING_USE_IK              0
#define USE_QP                    1
#define USE_MPC                   0
#define USE_QP_ST                 0
#define QP_PLUS_MPC               0

#define MPC_OUT_IN_THREAD         0

#define EN_SW_BOUND               0

#define EN_ROTATE_END_COMPASS_EST 1
#define EN_ROTATE_END_COMPASS_SW  1

#define EN_ST_TOUCH           1

#define EN_PLAN_USE_JERK      3

#define FIX_ST_TIME           1   //固定支撑时间  不开好点或者1
#if USE_MPC
    #define MIN_ST_TIME_RATE      0.8 //% 能保证摔倒起来
#else
    #define MIN_ST_TIME_RATE      0.6 //% 能保证摔倒起来
#endif
#define FORCE_FB_USE_REAL     1   //足底力使用真实IQ轴估计

#define EN_ONLINE_SW_PLAN     1   //在线修正X方向落足点
#define RE_PLAN_DT            0.01//s越短越好
#define SW_LOW_RATE1		  0.7
#define SW_LOW_RATE2		  0.8
#define SW_TD_CHECK_RATE      0.50
#define SW_TD_OVER_TIME       1.5//s
#define TD_NO_SHOCK_TIME      0.005//s

#define F_CONTROL_WITH_ROLL   0   //补偿横滚力的输出  仿真里没区别
#define F_EST_WITH_ROLL       0   //估计力使用横滚补偿  not good
#define F_SWING_WITH_ROLL     0
#define ROLL_LIMIT_COM		  25

#define USE_FORCE_REAL_GROUND 0   //TORT不会抖  0则高度保证但有姿态误差会明显大抖  采用老分配与2腿分配效果差不多仿真  实物2腿横滚没力

#define KIN_5LINE_FK          1
#define KIN_5LINE_IK          1  //Bug
#define KIN_5LINE_J           1  //相比老的不太行 not good
//新状态机：新规划 使用反馈差于不使用 但是下台阶时不太行    GROUND_AFTER_TRIG=0|| 老规划  不行和实物类似
//老状态机：：新规划 使用反馈好 GROUND_AFTER_TRIG=0  l||老规划 可以 GROUND_AFTER_TRIG=0
//->使用老规划和老状态机 不使用反馈   ->新状态机+老规划
#define GROUND_AFTER_TRIG       0   //TROT 着地就有力控 不等待另一个腿落地   规划和老状态机--使用后估计高度不跳变
#define EN_TORT_LOAD_FORCE4 	1   //TROT对角等待使用LOAD力 否则着地直接使用力分配
#define EN_TORT_LOAD_FORCE5     1   //摆动故障等待使用LOAD力

#define EN_Q_I_MIT_MODE       1   //角度积分
#define EN_END_SPD_MODE       1   //站立等末端采用速度控制

#define USB_SW_CAPTURE			  0	//使用MIT Capture Point的落足 前向会发散
#define ODOM_USE_1     				0	//使用丁博士的加速度里程计

#define TEST_TROT_SW 					0 //摆动测试<<------------------------
#define EN_SW 								1 //TROT能摆动
#define SW_WITH_REAL_FB       0 //使用反馈容易发散
#define SWING_USE_SPD_MODE    1    //摆动使用位置微分速度  仿真中不使用比较好 低摆动线程  低摆动线程只能使用SWING_USE_SPD_MODE==0用雅克比直接映射

#define G_EST_TIME            0.05 //5Hz
#define DEAD_G_ATT            0.5 //degree
#define FLT_GROUND_ATT_EST    3.5   //Hz

#define TEST_FF_MODE  0 //测试模块前提
#define TSET_F_IMP  0		//力导纳输出测试
#define TSET_F_FF   1		//力前馈输出
#define TSET_F_IF   0		//位力输出
#define TEST_F_FB_OUT 0 //力导纳位置反馈测试

#define STAND_GROUND_CHECK_TEST 1   //使能力控站立下的着地测试
#define SINGLE_LEG_TEST 0						//单腿测试仅支持站立
#define SINGLE_LEG_ID   2					//单腿测试ID

#define EN_GROUND_CHECK 1  			//使能 步态使用 着地判断
#define EN_ATT_GROUND_CONTROL 1 //使能 步态使用 地形估计
#define EN_TORQUE_CONTROL     1 //使能步态力矩输出<<------------------修改这使能机器人

#define GROUND_USE_EST 1		 //使用足底力估计Touch状态 或者使用着地传感器
#define USE_FPOS_CONTROL 1   //使用力控  <<------------------修改这改变空中位置模式

#define Q_NOW_USE_SET 0     //使用当前角度作为反馈

#define MIN_SPD_ST 0.003
#define MIN_SPD_ST_RAD 1
#define T_RST 1.5
#if USE_MPC
    #define MAX_FSPD 0.42   //??????????m/s
#else
    #define MAX_FSPD 0.58   //??????????m/s
#endif
#define POS_DEAD 0.05  //????????m
#define YAW_POS_MAX 25

#define USE_ESO_OBSEVER   		0//			???ESO????????????
#define ODOM_SPD_MODE         1//			1????????????
#define END_SPD_USE_POS       1//			1????????λ?????  1WS
#define ODOM_SPD_USE_LEG_AV   1//     ???????????????????
//-------------------------------机器人物理参数----------------------
//#define MOCO_ML_LST
#define MOCO_ML

#if RUN_WEBOTS&&!RUN_PI
#define Www          0.26
#define Hw           0.34
#define L1w          0.1
#define L2w          0.2
#define Mw           5.0
#define I_2_Nm       0.0326 //Nm/A 力矩系数
#else
#if defined(MOCO_ML_LST)
    #define Www          0.15	//宽
    #define Hw           0.264		//长
    #define L1w          0.055  //1连杆
    #define L2w          0.11		//2连杆
    #define L3w          0.02	//3连杆
    #define Mw           2.85  //kg     总重量
    #define I_2_Nm       0.035//Nm/A 力矩系数
#else
//	#define Www          0.19	//宽
//	#define Hw           0.365		//长
//	#define L1w          0.075  //1连杆
//	#define L2w          0.144		//2连杆
//	#define L3w          0.027	//3连杆
//	#define Mw           4.8  //kg     总重量
//	#define I_2_Nm       0.055//Nm/A 力矩系数
//#define MESS_KIN1 0.46 //kg
//#define MESS_KIN14 0.379 //kg
//#define MESS_KIN2 0.7 //0.46+0.335 //kg
//#define L_MESS_KIN1 0.035
//#define L_MESS_KIN14 0.4
//#define L_MESS_KIN2 0.075
#endif
#endif

#define SAFE_PITCH 35
#define SAFE_ROLL  25
#define SAFE_T     0.02//s
#define SAFE_Q     3
//-----------------------------------------数学参数--------------------
#define gw           9.8
#define piw          3.1415926
#define rad(x)      (x*piw/180.0)    //将角度化为弧度
#define deg(x)      (x*180.0/piw)    //将弧度化为角度
#define RAD_TO_DEGw  57.3
#define DEG_TO_RADw  0.0173

#define PITrw 0
#define ROLrw 1
#define YAWrw 2
#define Xrw 0
#define Yrw 1
#define Zrw 2
#define FRw 0
#define HRw 1
#define FLw 2
#define HLw 3
#define Fw 0
#define Bw 1

//-------------------------------------------------------------
#define D_LEG 0
#define X_LEG 1
#define T_LEG 2
#define Ls 0
#define Rs 1
#define Fs 0
#define Bs 1
#define FL 0
#define BL 1
#define FL1 0
#define BL1 1
#define FL2 2
#define BL2 3
#define Xr 0
#define Yr 1
#define Zr 2
#define P 0
#define I 2
#define DE 1
#define FB 3
#define IPv 4
#define IIv 5
#define IDv 6
#define PITr 0
#define ROLr 1
#define YAWr 2
#define YAWrr 3

#define MODE_SPD 1
#define MODE_POS 2
#define MODE_RAD 3
#define MODE_ATT 4
#define MODE_BODY 5
#define MODE_GLOBAL 6
#define MODE_ATT_YAW_ONLY 7
#define MODE_ATT_PR_ONLY 8
#define MODE_FAST_WAY 1
#define NMODE_FAST_WAY 0

#define PI 3.14159267
#define RAD_TO_DEG 180/PI
#define DEG_TO_RAD PI/180


//---------------------------------基头文件--------------------------
extern float MAX_SPD,MAX_SPD_X,MAX_SPD_Y,MAX_SPD_RAD,MIN_Z,MAX_Z,MIN_X,MAX_X,MIN_Y,MAX_Y;
extern float  MESS_KIN1;//=0;//   0.63 //kg
extern float  MESS_KIN14;//=0;//   0.35 //kg
extern float  MESS_KIN2;//=0;//     0.63 //0.46+0.335 //kg
extern float  L_MESS_KIN1;//=0;// 1  0.045
extern float  L_MESS_KIN14;//=0;//  0.08
extern float  L_MESS_KIN2;//=0;//   0.06

typedef struct
{
  float x;
  float y;
  float z;
}Vect3;

typedef struct
{
    float x;
    float y;
    float z;
    float zz;
}END_POS;


typedef struct
{
  float pos_now[3];
    float spd_now[3];
    float acc_now[3];
    float Tsw;//=0.25;
    float leg_dis;//=0.5;
    float leg_h;//=0.25;
    float spd_lift;//=0.8;
    float max_spd;//=leg_dis/Tsw;
    float limit_spd;//= max_spd*4;
    char flag[3];
    float T12;//=Tsw*0.2;%???????
    float T45;//=T14;
    float T23;//=(Tsw-T14*2)/2;
    float T34;//=T23;

    float p1[3],p2[3],p3[3],p4[3],p5[3];
    float v1[3],v2[3],v3[3],v4[3],v5[3];
    float a1[3],a2[3],a3[3],a4[3],a5[3];
    float a12[3],b12[3],g12[3];
    float a23[3],b23[3],g23[3];
    float a34[3],b34[3],g34[3];
    float a45[3],b45[3],g45[3];
    float param_x[6],param_y[6],param_z1[6],param_z2[6];
    float param_traj_x[10][6],param_traj_y[10][6], param_traj_z[10][6];
}END_PLANNER;

typedef struct
{
    char id,trig_state,ground_state,invert_knee;
    char q_now_use_tar;
    char vision_sw_enable;
    int invert_knee_epos[2];
    float sita1_off,sita2_off,sita3_off;
    float spd_dj[3];
    float acc_dj[3];
    float lift_spd,td_spd;
    float delta_h,delta_h_att_off;
    float kp_trig;
    float time_trig;
    float spd_est_cnt;
    END_POS tar_epos,tar_epos_b,tar_epos_n,tar_epos_n_reg,tar_epos_h,tar_epos_h_reg;
    END_POS tar_acc_n,tar_acc_b;
    int sita_flag[4],sita_flag_ocu[4];
    float st_time_used;
    float sw_time_used;
    float y_side;
    END_PLANNER end_planner;
    //-------------------
    float trap_cnt;
    float trap_mov_dis[4];
    float trap_mov_dis_norm[4];
    char is_trap;
    //-------------------
    float up_stair_cnt;
    int upstair_cnt;
    int duty_cnt;
    int upstarir_gait_keep_duty;
    char is_up_stair;
    Vect3 pos_stair;
    END_POS stair_pos;
    float trap_sw_t_fix;
    int force_foot_sw_end;
    float foot_sw_duty_set;
    float swingTimeRemaining;
    float posTimeRemaining[3];
}PARAM;

typedef struct
{
    float l1,l2,l3,l4,r;
    float tar_sita1,tar_sita2,tar_sita3;
    float sita1,sita2,sita3,sita;
    float dsita1,dsita2,dsita3,dsita;
    float sita_reg[3];
    float alfa,beta;
    END_POS epos,epos_b,epos_n,epos_nnn,epos_reg,epos_regn,epos_vm,epos_n_b;
    END_POS tar_epos_b,tar_epos_b_reg,tar_epos_n_sw;
    END_POS spd,spd_n,spd_b,spd_o;
    END_POS tar_epos_h,tar_epos_h_reg,tar_epos,tar_epos_n,tar_spd,tar_pos;
    END_POS st_pos,st_spd,epos_td_n,epos_lf_n,epos_sw_end_n;
    END_POS epos_td_hn,epos_lf_hn;
    END_POS epos_spdd_n,epos_spdd_b;
    END_POS epos_sw_st_n,epos_sw_st_b;
    END_POS epos_nn;
    END_POS odom_st;
    END_POS spd_hip_n;
    END_POS opt_epos_n,slip_epos_n;
    int slip_epos_n_flt_init;
    float tar_h,h,delta_ht;
    float jacobi33[9],ijacobi33[9];
    float jacobi33_reg[9],d_jacobi33[9];

    float jacobi33_kin1[9],ijacobi33_kin1[9];
    float jacobi33_kin12[9],ijacobi33_kin12[9];
    float jacobi33_kin2[9],ijacobi33_kin2[9];
    float jacobi22[4],ijacobi22[4];
    char ground,ground_noshock,ground_s,is_touch;
    float ground_force[3];
    float force[3],force_b[3],force_n[3],force_cmd[3];
    int flag_fb,flag_rl;
    float cnt_ss;
    float td_time_last;

    float st_phase, st_phase_timer;
    float sw_phase, sw_phase_timer;
    PARAM param;
}VMC;
//-----------------------------------------------------------
typedef struct{
    float fx,fy,fz;
    float slip_timer;
    float slip_sin;
    float slip_F[3];
}_SLIP;

extern _SLIP slip_mode;


enum Robot_type
{
  CUSTOM=0, Tinyml, Tinker, Taita
};

enum Gait_mode
{
  IDLE=0, TROT, STAND_RC, RECOVER, G_RL,G_KICK, SOFT
};

enum Robot_mode
{
  M_SAFE=0, M_STAND_RC, M_TROT, M_RECOVER, M_SOFT, M_RL,M_KICK
};

enum Leg_type
{
  PARALE=0, LOOP, PARALE_LOOP
};

enum Rc_type
{
  RC=0, UART, SBUS, SMART
};

enum DJ_type
{
  CUSTIOM_DJ=0,  DDJ_MG995, DDJ_MG355, DDJ_6221MG, DDJ_DSERVO, DDJ_9G, DDJ_DSB, DDJ_KPOWER,
    DDJ_BLS892MG, DDJ_EMAX_ES08MA, DDJ_MG90S,  DDJ_SG92R, DDJ_DM0090, DJ_S892,   DDJ_GDW_RS0708
};

typedef struct
{
    float pid_pit[3];
    float pid_rol[3];
    float pid_yaw[3];
    float pid_vx[3];
    float pid_vy[3];
    float pid_posxy[3];
    float pid_posz[3];
    float move_com_off[2];
    float move_att_off[2];
    float leg_off[2];
    float side_off[2],side_off_stand[2],side_off_walk[2],side_off_FH[2];
    float stance_xy_kp[2];
    float stance_zoff_kp;
    float stance_time[2];
    float swing_hight;
    float swing_spdkp[2];
    float posz_idle[3];
    float slip_p[3];
    float ground_seek_spd;
}PARAM_VMC;

typedef struct
{
    PARAM_VMC param_vmc,param_vmc_default;
    float Ix_Body,Iy_Body,Iz_Body;
    float Cogx_Body,Cogy_Body,Cogz_Body;
    float mom_lip_gain;

    float test_pos_flt[3],test_att_flt[3];
    int leg_dof;
    enum Leg_type leg_type;
    enum Robot_type robot_type;
    enum DJ_type dj_type;
    enum Rc_type rc_type;
    enum Robot_mode robot_mode;
    float soft_weight;
    float sw_com_off[3];
    float cof_off_all[4];
    float end_sample_dt;
    float gain_control_x,move_check_x;
    float safe_sita[3];
    END_POS gait_sw_end_reg_n[4];
    char trot_sw_end_flag[4];
    char param_save,cmd_use_ocu,send_mask;
    char stand_trot_switch_flag;
    char stand_switch_flag[2];
    char stand_switch_cnt[2];
    float ground_force[4][3];//足底传感器
    float encoder_spd[2];
    float cog_off_use[4];
    END_POS tar_spd_use[2],tar_spd_use_rc;
    END_POS climb_off;
    float MAX_Z,MIN_Z,MAX_X,MIN_X,MAX_Y,MIN_Y,MAX_PIT,MAX_ROL,MAX_YAW;
    char en_gait_switch,en_fall_protect;
    char have_cmd,have_cmd_rc[2],have_cmd_sdk[2][4],en_sdk;
    char control_mode,rc_mode[2];
    char cal_flag[5];
    char smart_control_mode[3];//pos/spd   high  att/rad
    int key[10];
    int rc_input[10];
    int sw_ot_compass[5];
    float trot_fb_off_y[3];
    float ctrot_fb_off_y[3];
}PARAM_ALL;


typedef struct
{
    u8 key_right;
    float version[2];
    char your_key[3];
    int lisence_test[3];
    int your_key_all;
    int board_id[4];
    char rl_mode_used;
    char side_flip,side_flip_done;
    int slope_face;
    char gait_force_fast;
#if !RUN_WEBOTS||RUN_PI
    enum Leg_type leg_type;
    enum Robot_type robot_type;
    enum DJ_type dj_type;
    enum Rc_type rc_type;
    enum Robot_mode robot_mode;
    int gait_mode, gait_mode_reg,gait_switching;
    int reset_trot_flag;
    int falling[2];
    float t_reset_trot_flag;
#else
    int gait_mode, gait_mode_reg;
#endif
    float sita_test[5];
    //---------------------------------------
    char trot_state, trot_phase, trot_air_test;
    u8 ground[2][4];
    END_POS tar_pos, tar_spd, tar_spd_rc;
    float tar_att[3],tar_att_mpc[3], tar_att_rate[3], att_rate_swing[3],tar_att_off[3], tar_att_bias[3], ground_off[2], tar_pos_off[3];
    float att_measure_bias[3],att_measure_bias_flip[3];
    //------------------------------------------
    float kp_trig[2];
    float cog_off[6], off_leg_dis[3];
    //
    float l1, l2, l3, l4, W, H, mess;
    float gait_time[4];
    float gait_alfa;//0~1
    float delay_time[3], gait_delay_time, stance_time, stance_time_auto;
    //
    float att_trig[4], att_ctrl[4], att_rate_trig[3], att_rate_ctrl[3], att[3],att_rate_imp[3];
    float att_rate[3], att_vm_b[3], att_vm[3], att_rate_vm[3], att_vmo[3], acc[4];
    char ground_per_trot;
    float ground_per_trot_timer=0;
    float ground_att[3], ground_att_est[3],ground_att_est_ng[3],ground_height,ground_height_spd, ground_att_cmd[3];
    float body_spd[4];
    float body_spd_b[2][4];
    END_POS pos, pos_n, pos_n_avg,pos_n_fp, pos_vm_leg, pos_vm_leg_n;
    END_POS cog_pos_n, zmp_pos_n, cog_spd_n, zmp_spd_n,spd_n_kf,pos_n_kf,now_zmp_nn,spd_nn_kf,pos_nn_kf;
    END_POS pos_n_b[2], spd_n_b[2];

    END_POS spd, spd_n,spd_ng;
    END_POS acc_n, acc_nn,acc_b;

    END_POS odom_st, ankle_pos_b[4],ankle_pos_n[4],ankle_pos_g[4];

    END_POS climb_off;
    float tar_spd_walk[2];
    float yaw_force, exp_yaw_rate;
    float Rb_n[3][3];
    float Rn_b[3][3];
    float Rb_nn[3][3];
    float Rnn_b[3][3];
    float Rn_b_noroll[3][3];
    float Rb_n_noroll[3][3];
    float Rn_g[3][3],Rg_n[3][3];
    float acc_norm;
    char ground_num, ground_num_touch, leg_power, power_state;
    char use_ground_sensor_new;

    float ground_plane_p[3],ground_norm[3];
    u8 err, unmove, hand_hold, fall, fly, fall_self;
    char mpc_init;
    //-------------rl
    float rl_commond_off[5];
    float rl_commond_rl_rst[3];
    float action_scale;
    float net_run_dt;
    float aciton_rl[14];
    float default_action[14];
    int aciton_rl_flag=0;
    float rl_gain;
    float sita3_off;
    int rl_connect;
    int loss_rl;
    PARAM_ALL param;
}VMC_ALL;

extern VMC vmc[4];
extern VMC_ALL vmc_all;

//---------------------------------------------------------------------------------------------------
typedef struct
{
    long mcuID[4];
    int board_id_test[3];
    int board_license_test[3];
    int board_license_check[3],board_ido[3];
    char key_right;
}_LISENCE;
extern _LISENCE lisens_vmc;
void get_license(void);

typedef struct{
 char en_record;
 char is_touch[4],is_ground[4];
 char force_en_flag[4];
 char leg_state[4];
 END_POS epos_n_tar[4];
 END_POS epos_n_now[4];
 END_POS depos_n_tar[4];
 END_POS depos_n_now[4];
 float sita_tar[4][3];
 float sita_now[4][3];
 END_POS GRF_n_tar[4];
 END_POS GRF_n_now[4];
 float com_n_tar[3];
 float com_n_now[3];
 float dcom_n_tar[3];
 float dcom_n_now[3];
 float ground_att_now[3];
 float att_now[3];
 float datt_now[3];
 float att_tar[3];
 float datt_tar[3];
 float temp_record[10];
}_RECORD;

typedef struct{
    char connect;
    int loss_cnt;
    char mode;
    char up_mode;
    char auto_mode;
    float rc_spd_b[3],rc_rate_b[3];

    char cmd_robot_state;
    float rc_spd_w[2],rc_att_w[2],rate_yaw_w;
    int key_ud,key_lr,key_x,key_y,key_a,key_b,key_ll,key_rr,key_st,key_back,key_ud_reg,key_lr_reg,key_x_reg,key_y_reg,key_a_reg,key_b_reg,key_ll_reg,key_rr_reg,key_st_reg,key_back_reg;
    float curve[20];

    char sbus_conncect;
    char sbus_power_sw,sbus_power_sw_reg;
    int sbus_rc_main[4];
    int sbus_rc_main_reg[4];
    int sbus_mode,sbus_mode_reg;
    int sbus_mode_e,sbus_mode_e_reg;
    float sbus_height,sbus_height_reg;
    float sbus_ch[6];
    int sbus_aux[6];
    int sbus_aux_reg[6];
    int sbus_cal_motor_zero;
    int sbus_motor_init,sbus_motor_init_reg;
    int sbus_cal_mems;
    int sbus_control_mode,sbus_control_mode_reg;
    int sbus_height_switch,sbus_height_switch_reg;
    int sbus_power_off,sbus_power_off_reg;
    int sbus_power_switch;
    int sbus_smart_en;
    int sbus_gait_sel;
    int sbus_auto_forward;
    int sbus_trig_aux_left;
    int sbus_trig_aux_right;
    float sbus_height_rate;
    float sbus_spd_rate[3];
    int cnt_zero_cal;

    //yunzhuo
    int key_a_yun,key_b_yun,key_c_yun,key_d_yun;
    int sel_e_yun,sel_f_yun;
    float rc_g_yun,rc_h_yun;

    //esp32
    char esp32_connect;
  _RECORD record;
}_OCU;
extern _OCU ocu;

typedef struct{
    float high_leg_end;
    float gait_duty;
    float max_spd;
    float max_rad;
    float cog_off[2];
    float kp_trig[3];
}_GAIT_PARM;
extern _GAIT_PARM tort_p,walk_p,ftort_p,init_p,bound_p,pace_p,climb_p,custom_gait,crawl_p,pronk_p,bound_p;

//-----------------------------------------机器结构体--------------------------------------------
typedef struct
{
  float roll;       //横滚，x轴
  float pitch;      //俯仰，y轴
  float yaw;        //偏航，z轴
  float yaw_off;
}eulerAngleTypeDef;

typedef struct
{
  double x;
  double y;
  double z;
}robPosTypeDef;

typedef struct
{
  float exp,now,now_reg;
    float err,err_dead,err_reg;
  float p_out;
  float i_out;
  float d_out;
  float max_out;
  float kp_o,kp,ki,kd,vff;
  float kp_o_d[3],kp_d[3],ki_d[3],kd_d[3],vff_d[3];

  float kp_sw,ki_sw,kd_sw;
  float kp_st,ki_st,kd_st;

  float kp_sw_d[3],ki_sw_d[3],kd_sw_d[3];
  float kp_st_d[3],ki_st_d[3],kd_st_d[3];
  char param_sel;
}PIDs;

typedef struct
{
  float kp_pos,ki_pos,kd_pos;
  float kp_force,ki_force,kd_force;
}POS_FORCE_P;

typedef struct
{
  float st_lf,st_td;
  float trot_lf,trot_sw,trot_td;
  float td_short;
  int check_td,check_lf;
  float check_spd;
}_TD_CHECK_PARAM;

//单腿信息结构体
typedef struct
{
    int id;
    Vect3 epos_h,epos_h_reg;
    Vect3 tar_epos_h,tar_epos_n;
    Vect3 force_err_n,force_imp_spd_h,force_imp_spd_n;
    float pos_taod[3],fb_force_taod[3],ff_force_taod[3];

    Vect3 epos_b;
    Vect3 epos_n;
    Vect3 epos_n_kf;
    Vect3 espd_h,espd_n;

    Vect3 tar_espd_h;
    Vect3 tar_espd_n;

    Vect3 tar_force_h;
    Vect3 tar_force_n;
    Vect3 tar_force_dis_n;
    Vect3 tar_force_dis_n_reg;

    Vect3 force_est_h;
    Vect3 force_est_n;
    Vect3 tar_force_dis_n_qp;
    Vect3 tar_force_dis_n_mpc;
    float est_u_fz,est_u_fxy;

    Vect3 force_est_h_output;
    Vect3 force_est_n_output;
    float force_est_n_length,force_est_h_length;

    Vect3 dforce_est_h_output;
    Vect3 dforce_est_n_output;

    Vect3 force_est_h_output_reg;
    Vect3 force_est_n_output_reg;

    Vect3 F_n_ff;
    float w_force_taod[3];
    Vect3 exp_pos_b[3],exp_pos_n_b[3];
    PIDs q_pid_sw,q_pid_st;
    PIDs f_pos_pid_st[3],f_pos_pid_sw[3];
    PIDs f_pid_st[3],f_pid_sw[3];
    PIDs q_pid,f_pid[3],f_pos_pid[3];

    float sita[3],sita_reg[3],sita_r,r,alfa,beta;
    float limit_sita[3];
    float sita_d[3],sita_dd[3];
    float sita_d_flt[3],sita_dd_flt[3];
    float err_sita[3];
    float tar_sita[3];
    float tar_sita_d[3];

    float jacobi[9],jacobi_inv[9];
    float jacobi_kin1[9],jacobi_inv_kin1[9];
    float jacobi_kin12[9],jacobi_inv_kin12[9];
    float jacobi_kin2[9],jacobi_inv_kin2[9];
    float jacobi33[9],jacobi_inv33[9];
    int flag_fb,flag_rl;

    float pos[3];         //测量得到的当前关节角度            弧度制
    float taom[3];        //实时测量的扭矩
    float taom_output[3];        //实时测量的扭矩
    float taod[3],tao_bias[3],taod_mess[3],taod_mess1[3];        //期望扭矩
    float taod_D[3];
    float taod_C[3];
    float taom_C[3];
    float taom_D[3];

    float taod_kin1[3];
    float taod_kin12[3];
    float taod_kin2[3];
    float taod_ff[3];        //期望扭矩
    float tao_q_i[3];
    float limit_tao[3];
    char  is_ground;
    char  is_touch,is_touch_est;       //是否触地 true : 触地 false : 离地
    _TD_CHECK_PARAM touch_z_param;

    int touch_cnt[5][3];
    int touch_cnto[5];
    int trig_state,ground_state,st_torque_need_init;
    Vect3 st_pos;
    Vect3 epos_td_n,epos_lf_n;
    Vect3 epos_sw_end_n;
    Vect3 epos_td_hn,epos_lf_hn;
    Vect3 epos_spdd_n,epos_spdd_b;
    Vect3 epos_sw_st_n,epos_sw_st_b;
    Vect3 epos_nn;
    Vect3 odom_st;
    float delta_ht;
    float time_trig;
    float cnt_ss;
    float espd_norm_b;
    float contactStates;
    float swingStates;
    int mpc_table[20];
    int mpc_table_o[20];
    int mpc_table_s[20];
}LegTypeDef;

typedef struct {
    char en_force_control_cal,en_force_control_out;
    PIDs q_pid_sw,q_pid_st_trot,q_pid_st_stance,q_pid_init;
    PIDs f_pos_pid_st[3];
    PIDs f_pid_st[3];
    PIDs zeros;
    float load_fz,td_before_fz;
    //---
    _TD_CHECK_PARAM touch_z_param_st;
    _TD_CHECK_PARAM touch_z_param_sw;
    _TD_CHECK_PARAM touch_z_param_td;
    _TD_CHECK_PARAM touch_z_param_trot_st;
    _TD_CHECK_PARAM touch_z_param_pronk;
    _TD_CHECK_PARAM touch_z_param_climb;
    _TD_CHECK_PARAM touch_z_param_bound;
    //--
    float t_to_i;
    float max_t;
    float max_i;
    float max_t_d[3];
    float max_i_i[3];
}POS_FORCE_PARM;

extern POS_FORCE_PARM pos_force_p;

typedef struct {
    //Stand
    PIDs pos_x,pos_y,pos_z;
    PIDs spd_x,spd_y,spd_yaw;
    PIDs att_pit,att_rol,att_yaw;
    //Trot
    PIDs att_pit_trot,att_rol_trot,att_yaw_trot,pos_z_trot;
    //walk
    PIDs pos_x_walk,pos_z_walk;
    PIDs spd_x_walk,spd_yaw_walk;
    PIDs att_pit_walk,att_rol_walk,att_yaw_walk;
    //Bound
    PIDs att_rol_bound,att_yaw_bound;
    Vect3 stand_off,spd_off;
    float mess_scale;
    float sw_deltah;
    float ground_mu;
}VMC_ROBOT_PARM;

extern VMC_ROBOT_PARM vmc_robot_p;

//机器人结构体 包含整个机器人的所有信息
typedef struct
{
    char gait_level;
    char beep_state;
    LegTypeDef  Leg[4];
    Vect3 vect3_zero;
    eulerAngleTypeDef   IMU_now,IMU_now_o;        //机器人当前欧拉角
    eulerAngleTypeDef   IMU_last;       //机器人上次欧拉角
    eulerAngleTypeDef   IMU_dot,IMU_dot_o;        //机器人欧拉角速度
    eulerAngleTypeDef   ground_att;

    eulerAngleTypeDef   exp_att;
    eulerAngleTypeDef   now_att;
    eulerAngleTypeDef   now_rate;
    eulerAngleTypeDef   now_rate_reg;
    eulerAngleTypeDef   exp_rate;
    Vect3               exp_spd_b;
    Vect3               exp_pos_b;
    Vect3               exp_spd_n,exp_spd_ng,exp_pos_ng;
    Vect3               exp_pos_n;
    Vect3               cog_pos_b;
    Vect3               cog_spd_n;
    Vect3               cog_spd_b;
    Vect3               cog_acc_n;
    Vect3               cog_acc_b;
    Vect3               cog_pos_n;
    Vect3				cog_pos_nn;

    Vect3				cog_F_ff;
    Vect3				cog_T_ff;

    int ground_num;
    int ground_num_rl[2];
    int ground_num_fb[2];

    float mess_payload;
    float mess_off[3];
    float I_off[3];

    Vect3 exp_force;
    Vect3 exp_torque;
    Vect3 exp_torque_inv;
    Vect3 exp_force_i;
    Vect3 exp_torque_i;
    //
    Vect3 exp_pos_n_b[2];
    Vect3 exp_force_b[2];
    Vect3 exp_torque_b[2];
    Vect3 exp_force_i_b[2];
    Vect3 exp_torque_i_b[2];

    Vect3 max_force;
    Vect3 max_err_force;
    Vect3 max_torque;
    Vect3 max_leg_force,min_leg_force;

    float mess_est;
    float Rb_n[3][3];
    float Rn_b[3][3];
    float Rn_b_noroll[3][3];
    float Rb_n_noroll[3][3];
    float Rn_g[3][3],Rg_n[3][3];
    float MIN_X;
    float MAX_X;
    float MIN_Y;
    float MAX_Y;
    float MIN_Z;
    float MAX_Z;
    //param
    int use_ground_sensor;
    float gait_time;
    float stance_time;
    float kp_trig[2];
    float leg_off[2];
    Vect3 ankle_pos_n[4], cog_spd_nn;
    _OCU ocu;

    int _nIterations;
    float dtMPC;
}robotTypeDef;
extern robotTypeDef robotwb;



typedef struct {
    char state_m;
    char need_trig, en_fast_cog_move;
    char area_too_small;
    char cog_stable;
    float cog_stable_timer;
    char cog_tar_reach;
    char ground_num;
    char pre_trig_id, trig_id, trig_id_last;
    char trig_record_cnt;
    char trig_id_his[4];
    char xy_control_mode;
    float stable_value;
    float min_st_value;//质心稳定阈值死区
    float stable_bind_width;//支撑三角形稳定边界
    float cog_reach_dead, cog_reach_deadnn;//质心控制死区
    float work_space_band;//工作空间边界
    float stable_band;
    float work_space[4];
    char out_ws_flag[4];
    float att_com_off[2];
    float support_area;
    float min_support_area;
    float support_area_check;
    float tar_spd_length;
    float tar_spd_yaw;
    float sw_h_off_walke;
    END_POS cord_n;//全局运动坐标系复位位置
    END_POS now_cog_nn;//目前支撑平面全局质心
    END_POS tar_zmp_nn,now_zmp_nn;
    END_POS tar_zmp_n, tar_zmp_b;//质心参数(坐标系在质心)
    END_POS now_zmp_n, now_zmp_b;
    END_POS now_cog_n, now_cog_b;
    END_POS leg_move_per_trig;
    END_POS leg_off_move;
    END_POS now_st_cog_n, now_st_cog_b;//支撑区域参数
    END_POS now_st_cog_n_reg, now_st_cog_b_reg;//支撑区域参数
    END_POS now_st_cogspd_n, now_st_cogspd_b;//支撑区域参数
    END_POS now_st_cog_n1, now_st_cog_b1;//支撑区域参数
    END_POS walk_cmd_trig;

    float imp_force_time_rate;
    float imp_alfa[3];
}_GAIT_WALK;
extern _GAIT_WALK walk_gait;

typedef struct
{ //phase
    char ref_leg;
    float timer[2];
    float t_ref;
    float t_d;
    float ti[4];
    float T_all;
    float T_sw;
    float T_st;

    float dS[4];
    float S_target[4];
    float S_now[4];
    float S_st[4];
    float S_sw[4];
    float S_sw_ref_reg, S_st_ref_reg;
    float sw_start[4], sw_end[4];//NEW
    int ground_flag[4][2];
    float tg;
    float tg_st;
    float w_gait_change;
    float t_gait_change;
    float min_st_rate;
    int gait_mode[2];
    //event
    char state[4];
    float imp_force[4][3];
}GAIT_SHEC;

extern GAIT_SHEC gait;
#endif
