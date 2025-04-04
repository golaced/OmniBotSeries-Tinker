#ifndef __BASE_STRUCT_H__
#define __BASE_STRUCT_H__
#include "math.h"
#define RUN_WEBOTS 1

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
//--------------------------------VMC��궨��----------------------------------
#define EN_PLAN_USE_JERK      0   //ʹ��Jerk�켣�ٶ���
#define USB_SW_CAPTURE		    0	//ʹ��MIT Capture Point������ ǰ��ᷢɢ
#define USE_FORCE_REAL_GROUND 0   //TORT���ᶶ  0��߶ȱ�֤������̬�������Դ� Ϊ0�������Ʋ�ס
#define EN_SW_NEW             (USE_FORCE_REAL_GROUND&&1)||0   //USE_FORCE_REAL_GROUNDֻ��ʹ������¹滮 ���ⲻ��ʹ��USB_SW_CAPTURE

#define FIX_ST_TIME           1   //�̶�֧��ʱ��
#define MIN_ST_TIME_RATE      0.6 //%
#define SW_LOW_RATE           0.56 //% ����Ч������˶�������
#define FORCE_FB_USE_REAL     1   //�����ʹ����ʵIQ�����

#define EN_ONLINE_SW_PLAN     1   //��������X���������
#define RE_PLAN_DT            0.05//sԽ��Խ��

#define F_CONTROL_WITH_ROLL   1   //��������������  ������û����
#define F_EST_WITH_ROLL       1   //������ʹ�ú������

#define KIN_5LINE_FK          1
#define KIN_5LINE_IK          0  //Bug
#define KIN_5LINE_J           1  //����ϵĲ�̫��
//��״̬�����¹滮 ʹ�÷������ڲ�ʹ�� ������̨��ʱ��̫��    GROUND_AFTER_TRIG=0|| �Ϲ滮  ���к�ʵ������
//��״̬�������¹滮 ʹ�÷����� GROUND_AFTER_TRIG=0  l||�Ϲ滮 ���� GROUND_AFTER_TRIG=0
//->ʹ���Ϲ滮����״̬�� ��ʹ�÷���   ->��״̬��+�Ϲ滮
#define GROUND_AFTER_TRIG       1   //TROT �ŵؾ������� ���ȴ���һ�������   �滮����״̬��--ʹ�ú���Ƹ߶Ȳ�����
#define EN_TORT_LOAD_FORCE4 	  1   //TROT�Խǵȴ�ʹ��LOAD�� �����ŵ�ֱ��ʹ��������  ���²෭�޷���ԭ������ ������GROUND_AFTER_TRIGһ��ʹ��
#define EN_TORT_LOAD_FORCE5     1   //�ڶ����ϵȴ�ʹ��LOAD��

#define EN_Q_I_MIT_MODE       1   //�ǶȻ���
#define EN_END_SPD_MODE       1   //վ����ĩ�˲����ٶȿ���  

#define ODOM_USE_1     				0	//ʹ�ö���ʿ�ļ��ٶ���̼�

#define TEST_TROT_SW 					0 //�ڶ�����<<------------------------
#define EN_SW 								1 //TROT�ܰڶ�
#define SW_WITH_REAL_FB       0 //ʹ�÷������׷�ɢ
#define SWING_USE_SPD_MODE    1    //�ڶ�ʹ��λ��΢���ٶ�  �����в�ʹ�ñȽϺ� �Ͱڶ��߳�  �Ͱڶ��߳�ֻ��ʹ��SWING_USE_SPD_MODE==0���ſ˱�ֱ��ӳ��

#define G_EST_TIME            0.25 //4Hz
#define DEAD_G_ATT            0.5 //degree
#define FLT_GROUND_ATT_EST    3   //Hz

#define TEST_FF_MODE  0 //����ģ��ǰ��
#define TSET_F_IMP  0		//�������������
#define TSET_F_FF   1		//��ǰ�����
#define TSET_F_IF   0		//λ�����
#define TEST_F_FB_OUT 0 //������λ�÷�������

#define STAND_GROUND_CHECK_TEST 1   //ʹ������վ���µ��ŵز���
#define SINGLE_LEG_TEST 0						//���Ȳ��Խ�֧��վ��	
#define SINGLE_LEG_ID   2					//���Ȳ���ID

#define EN_GROUND_CHECK 1  			//ʹ�� ��̬ʹ�� �ŵ��ж�
#define EN_ATT_GROUND_CONTROL 1 //ʹ�� ��̬ʹ�� ���ι���
#define EN_TORQUE_CONTROL     1 //ʹ�ܲ�̬�������<<------------------�޸���ʹ�ܻ�����

#define GROUND_USE_EST 1		 //ʹ�����������Touch״̬ ����ʹ���ŵش�����
#define USE_FPOS_CONTROL 1   //ʹ������  <<------------------�޸���ı����λ��ģʽ

#define Q_NOW_USE_SET 0     //ʹ�õ�ǰ�Ƕ���Ϊ����

#define MIN_SPD_ST 0.003
#define MIN_SPD_ST_RAD 1
#define T_RST 1.5
#define MAX_FSPD 0.5   //??????????m/s
#define POS_DEAD 0.05  //????????m
#define YAW_POS_MAX 25

#define USE_ESO_OBSEVER   		0//			???ESO????????????
#define ODOM_SPD_MODE         1//			1????????????
#define END_SPD_USE_POS       1//			1????????��?????  1WS
#define ODOM_SPD_USE_LEG_AV   1//     ???????????????????
//-------------------------------��������������----------------------
#define MOCO_ML_LST
//#define MOCO_ML

#if RUN_WEBOTS&&!RUN_PI
#define Www          0.26 
#define Hw           0.34
#define L1w          0.1
#define L2w          0.2
#define Mw           5.0
#define I_2_Nm       0.0326 //Nm/A ����ϵ��
#else
#if defined(MOCO_ML_LST)
	#define Www          0.15	//��
	#define Hw           0.264		//��
	#define L1w          0.055  //1����
	#define L2w          0.11		//2����
	#define L3w          0.015	//3����
	#define Mw           2.85  //kg     ������
	#define I_2_Nm       0.035//Nm/A ����ϵ��
#else
	#define Www          0.19	//��
	#define Hw           0.35		//��
	#define L1w          0.073  //1����
	#define L2w          0.145		//2����
	#define Mw           3.7  //kg     ������
	#define I_2_Nm       0.0326 //Nm/A ����ϵ��
#endif
#endif

#define SAFE_PITCH 25
#define SAFE_ROLL  25
#define SAFE_Q     3
//-----------------------------------------��ѧ����--------------------
#define gw           9.8
#define piw          3.1415926
#define rad(x)      (x*piw/180.0)    //���ǶȻ�Ϊ����
#define deg(x)      (x*180.0/piw)    //�����Ȼ�Ϊ�Ƕ�
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
#define R 1
#define F 0
#define B 1
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
#define D 1
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


//---------------------------------��ͷ�ļ�--------------------------
extern float MAX_SPD,MAX_SPD_RAD,MIN_Z,MAX_Z,MIN_X,MAX_X,MIN_Y,MAX_Y;

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
	float T45;//=T12;
	float T23;//=(Tsw-T12*2)/2;
	float T34;//=T23;

	float p1[3],p2[3],p3[3],p4[3],p5[3];
	float v1[3],v2[3],v3[3],v4[3],v5[3];
	float a1[3],a2[3],a3[3],a4[3],a5[3];
	float a12[3],b12[3],g12[3];
	float a23[3],b23[3],g23[3];
	float a34[3],b34[3],g34[3];
	float a45[3],b45[3],g45[3];
}END_PLANNER;

typedef struct 
{
  char id,trig_state,ground_state,invert_knee;
	char q_now_use_tar;
	int invert_knee_epos[2];
	float sita1_off,sita2_off,sita3_off;
	float spd_dj[3];
	float lift_spd,td_spd;
	float delta_h,delta_h_att_off;
	float kp_trig;
	float time_trig;
	float spd_est_cnt;
	END_POS tar_epos,tar_epos_b,tar_epos_n,tar_epos_n_reg,tar_epos_h,tar_epos_h_reg;
	int sita_flag[4],sita_flag_ocu[4];				
	float st_time_used;
	float sw_time_used;
	END_PLANNER end_planner;
}PARAM;

typedef struct 
{
	float l1,l2,l3,l4,r;
	float tar_sita1,tar_sita2,tar_sita3;
	float sita1,sita2,sita3,sita;
	float dsita1,dsita2,dsita3,dsita;
	float sita_reg[3];
	END_POS epos,epos_b,epos_n,epos_reg,epos_regn,epos_vm;
	END_POS spd,spd_n,spd_b,spd_o;
	END_POS tar_epos_h,tar_epos_h_reg,tar_epos,tar_epos_n,tar_spd,tar_pos;
	END_POS st_pos,epos_td_n,epos_lf_n,epos_sw_end_n;
	END_POS epos_td_hn,epos_lf_hn;
	END_POS epos_spdd_n,epos_spdd_b;
	END_POS epos_sw_st_n,epos_sw_st_b;
	END_POS epos_nn;
	END_POS odom_st;
	float tar_h,h,delta_ht;
	float jacobi33[9],ijacobi33[9];
	float jacobi22[4],ijacobi22[4];
	char ground,ground_s,is_touch;
	float ground_force[3];
	float force[3],force_b[3],force_n[3],force_cmd[3];
	int flag_fb,flag_rl;
	float cnt_ss;
	PARAM param;
}VMC;
//-----------------------------------------------------------
#if !RUN_WEBOTS
enum Robot_type
{
  CUSTOM=0, MOCO8_PRO, MOCO8_MID, MOCO8_CHEETHA, MOCO8_LS3, MOCO12_PRO, MOCO12_LS3, MOCO12_SPOT, MOCO12_SPOTMINI, MOCO12_HYQ, MOCO12_ANYMAL 
};

enum Gait_mode
{
  IDLE=0, TROT, F_TROT, WALK, STAND_RC,  STAND_IMU,  STAND_PUSH , RECOVER, FALLING, CLIMB,  WALK_E, CRAWL, PRONK, BOUND
};

enum Robot_mode
{
  M_SAFE=0, M_STAND_RC,  M_STAND_IMU,  M_STAND_PUSH, M_TROT, M_F_TROT, M_WALK, M_RECOVER,M_FALLING,M_CLIMB,M_WALKE,M_CRAWL,M_PRONK,M_BOUND
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
#else
#define CUSTOM  0
#define MOCO8_PRO 1
#define MOCO8_MID 2
#define MOCO8_CHEETHA 3
#define MOCO8_LS3 4
#define MOCO12_PRO 5 
#define MOCO12_LS3 6
#define MOCO12_SPOT 7
#define MOCO12_SPOTMINI 8
#define MOCO12_HYQ 9
#define MOCO12_ANYMAL 10


#define IDLE  0
#define TROT 1
#define F_TROT 2
#define WALK  3
#define STAND_RC 4
#define STAND_IMU  5
#define STAND_PUSH 6
#define RECOVER 7
#define FALLING 8
#define CLIMB  9
#define WALK_E 10
#define CRAWL 11
#define PRONK  12
#define BOUND 13

#define M_SAFE 0
#define M_STAND_RC  1
#define M_STAND_IMU 2
#define M_STAND_PUSH 3
#define M_TROT 4
#define M_F_TROT 5
#define M_WALK 6
#define M_RECOVER 7
#define M_FALLING 8
#define M_CLIMB 9
#define M_WALKE 10
#define M_CRAWL 11
#define M_PRONK 12
#define M_BOUND 13

#define PARALE 0
#define LOOP 1
#define PARALE_LOOP 2
#endif
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
	float side_off[2],side_off_stand[2],side_off_walk[2];
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
	float test_pos_flt[3],test_att_flt[3];
	int leg_dof;
#if !RUN_WEBOTS
	enum Leg_type leg_type;
	enum Robot_type robot_type;
	enum DJ_type dj_type;
	enum Rc_type rc_type;
	enum Robot_mode robot_mode;
#else
	int leg_type;
	int robot_type;
	int dj_type;
	int rc_type;
	int robot_mode;
#endif
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
	float ground_force[4][3];//��״�����
	float encoder_spd[2];
	float cog_off_use[4];
	END_POS tar_spd_use[2],tar_spd_use_rc;
	float MAX_Z,MIN_Z,MAX_X,MIN_X,MAX_Y,MIN_Y,MAX_PIT,MAX_ROL,MAX_YAW;
	char en_gait_switch,en_fall_protect;
	char have_cmd,have_cmd_rc[2],have_cmd_sdk[2][4],en_sdk;
	char control_mode,rc_mode[2];
	char cal_flag[5];
	char smart_control_mode[3];//pos/spd   high  att/rad
	int key[10];
	int rc_input[10];
}PARAM_ALL;

typedef struct 
{
	u8 key_right;		 
	float version[2];
	char your_key[3];
	int lisence_test[3];
	int your_key_all;
	int board_id[4]; 
#if !RUN_WEBOTS
	enum Gait_mode gait_mode,gait_mode_reg;
#else
	int gait_mode, gait_mode_reg;
#endif
	float sita_test[5];
	//---------------------------------------
	char trot_state,trot_phase;
	u8 ground[2][4];
	END_POS tar_pos,tar_spd,tar_spd_rc;
	float tar_att[3],tar_att_rate[3],tar_att_off[3],tar_att_bias[3],ground_off[2];
	//------------------------------------------
	float kp_trig[2];
	float cog_off[6],off_leg_dis[3];
	//
  float l1,l2,l3,l4,W,H,mess;
	float gait_time[4];
	float gait_alfa;//0~1
	float delay_time[3],gait_delay_time,stance_time,stance_time_auto;
	//
	float att_trig[4],att_ctrl[4],att_rate_trig[3],att_rate_ctrl[3],att[3],att_rate[3],att_vm_b[3],att_vm[3],att_rate_vm[3],att_vmo[3],acc[4];
	float ground_att[3],ground_att_est[3],ground_att_cmd[3];
	float body_spd[4];
	END_POS pos,pos_n,pos_n_fp,pos_vm_leg,pos_vm_leg_n;
	END_POS cog_pos_n,zmp_pos_n,cog_spd_n,zmp_spd_n;
	
	END_POS spd,spd_n;
	END_POS acc_n,acc_b;
	
	END_POS odom_st,ankle_pos_n[4];
	float yaw_force,exp_yaw_rate;
	float Rb_n[3][3];
	float Rn_b[3][3];
	float acc_norm;
	char ground_num,ground_num_touch,leg_power,power_state;
	char use_ground_sensor_new;
	u8 err,unmove,hand_hold,fall,fly,fall_self;
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
	int sbus_aux[4];
	int sbus_aux_reg[4];

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

//-----------------------------------------�����ṹ��--------------------------------------------
typedef struct
{
  double roll;       //�����x��
  double pitch;      //������y��
  double yaw;        //ƫ����z��
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
  float kp,ki,kd,vff;   
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
	int check_td,check_lf;
}_TD_CHECK_PARAM;

//������Ϣ�ṹ��
typedef struct 
{
  int id;
  Vect3 epos_h,epos_h_reg;
  Vect3 tar_epos_h,tar_epos_n;
	Vect3 force_err_n,force_imp_spd_h,force_imp_spd_n;
	float pos_taod[3],fb_force_taod[3],ff_force_taod[3];

  Vect3 epos_b;
  Vect3 epos_n;

  Vect3 espd_h,espd_n;

  Vect3 tar_espd_h;
  Vect3 tar_espd_n;

  Vect3 tar_force_h;
  Vect3 tar_force_n;
	Vect3 tar_force_dis_n;
	Vect3 tar_force_dis_n_reg;
  
  Vect3 force_est_h;
  Vect3 force_est_n;
  
  Vect3 force_est_h_output;
  Vect3 force_est_n_output;
  float force_est_n_length;
	
  Vect3 dforce_est_h_output;
  Vect3 dforce_est_n_output;
  Vect3 tar_force_dis_n_qp;
  Vect3 force_est_h_output_reg;
  Vect3 force_est_n_output_reg;

  PIDs q_pid_sw,q_pid_st;
  PIDs f_pos_pid_st[3],f_pos_pid_sw[3];
  PIDs f_pid_st[3],f_pid_sw[3];
  PIDs q_pid,f_pid[3],f_pos_pid[3];

  float sita[2],sita_reg[2],sita_r,r,alfa,beta;
  float limit_sita[2];
  float sita_d[2],sita_dd[2];
	float err_sita[2];
  float tar_sita[2];
  float tar_sita_d[2];

  float jacobi[4],jacobi_inv[4];
  
  int flag_fb,flag_rl;

  float pos[2];         //�����õ��ĵ�ǰ�ؽڽǶ�            ������
  float taom[2];        //ʵʱ������Ť��
  float taom_output[2];        //ʵʱ������Ť��
  float taod[2];        //����Ť��
  float taod_ff[2];        //����Ť��
	float tao_q_i[2];
  float limit_tao[2];
  char  is_ground;
  char  is_touch,is_touch_est;       //�Ƿ񴥵� true : ���� false : ���
	_TD_CHECK_PARAM touch_z_param;
	
  int touch_cnt[5][2];
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
}LegTypeDef;

typedef struct {
	char en_force_control_cal,en_force_control_out;
  PIDs q_pid_sw,q_pid_st_trot,q_pid_st_stance;
  PIDs f_pos_pid_st[3];
  PIDs f_pid_st[3];
	PIDs zeros;
	float load_fz;
	float td_before_fz;
	//---
	_TD_CHECK_PARAM touch_z_param_st;
	_TD_CHECK_PARAM touch_z_param_sw;
	_TD_CHECK_PARAM touch_z_param_td;
	_TD_CHECK_PARAM touch_z_param_trot_st;
	//--
	float t_to_i;
	float max_t;
	float max_i;
	
}POS_FORCE_PARM;

extern POS_FORCE_PARM pos_force_p;

typedef struct {
  PIDs pos_x,pos_z;
  PIDs spd_x,spd_yaw;
  PIDs att_pit,att_rol,att_yaw;
	
	PIDs att_pit_trot,att_rol_trot,att_yaw_trot;
	Vect3 stand_off;
	float mess_scale;
	float sw_deltah;
	float ground_mu;
}VMC_ROBOT_PARM;

extern VMC_ROBOT_PARM vmc_robot_p;

//�����˽ṹ�� �������������˵�������Ϣ
typedef struct 
{
  char gait_level;
	char beep_state;
	LegTypeDef  Leg[4]; 
	Vect3 vect3_zero;
  eulerAngleTypeDef   IMU_now;        //�����˵�ǰŷ����
  eulerAngleTypeDef   IMU_last;       //�������ϴ�ŷ����
  eulerAngleTypeDef   IMU_dot;        //������ŷ�����ٶ�
  eulerAngleTypeDef   ground_att;
	
  eulerAngleTypeDef   exp_att;        
  eulerAngleTypeDef   now_att;        
  eulerAngleTypeDef   now_rate; 
  eulerAngleTypeDef   now_rate_reg; 
  eulerAngleTypeDef   exp_rate; 
  Vect3               cog_acc_b;	
  Vect3               cog_acc_n;
  Vect3               exp_spd_b;
  Vect3               exp_pos_b;
  Vect3               exp_spd_n;
  Vect3               exp_pos_n;
  Vect3               cog_spd_b;
  Vect3               cog_pos_b;
  Vect3               cog_spd_n;
  Vect3               cog_pos_n;
	Vect3								cog_pos_nn;
	
  int ground_num;
  int ground_num_rl[2];
  int ground_num_fb[2];
   
  Vect3 exp_force;
  Vect3 exp_torque;
  Vect3 exp_force_i;
  Vect3 exp_torque_i;
  Vect3 max_force;
	Vect3 max_err_force;
  Vect3 max_torque;
	
	float mess_est;
  float Rb_n[3][3];
  float Rn_b[3][3]; 
  
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
  Vect3 ankle_pos_n[4],cog_spd_nn;
  _OCU ocu;
}robotTypeDef;
extern robotTypeDef robotwb;


typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

struct xyz_f_tr
	{
  float x;
	float y;
	float z;
};
typedef struct 
{
  s16 x;
	s16 y;
	s16 z;

}xyz_s16_t;

typedef struct 
{ 
	char Acc_CALIBRATE;
	char Gyro_CALIBRATE;
	char Cali_3d;
    xyz_s16_t Acc_I16;
	xyz_s16_t Gyro_I16;
	xyz_f_t imu_pos;
	xyz_f_t imu_att;
	xyz_f_t gps_pos;
	xyz_f_t Acc;
	xyz_f_t Acc_rt;
	xyz_f_t Gyro;
	xyz_f_t Gyro_deg;
	xyz_f_t Gyro_deg_rt;
	xyz_f_t Mag,Mag_rt,Mago,Mago_rt;
	xyz_f_t Acc_Offset;
	xyz_f_t Acc_Scale;
	xyz_f_t Gyro_Offset;
	xyz_f_t Gyro_Scale;
	xyz_f_t Gyro_Auto_Offset;
	xyz_f_t Gain_3d;
	xyz_f_t Off_3d;
 	char Mag_CALIBRATE,Mag_Have_Param,Mag_ERR,Mag_update;
	xyz_s16_t Mag_Adc,Mag_Adc_o;			//采样值
	xyz_f_t   Mag_Offset,Mag_Offseto;		//偏移值
	xyz_f_t   Mag_Offset_c,Mag_Offset_co;		//偏移值
	xyz_f_t   Mag_Gain,Mag_Gaino;		//偏移值
	xyz_f_t 	Mag_Gain_c,Mag_Gain_co;			//比例缩放	
	xyz_f_t 	Mag_Val,Mag_Val_t,Mag_Valo,Mag_Val_to;			//纠正后的值
	float hmlOneMAG,hmlOneACC;
  float Yaw_Mag;
	float Ftempreature;
}_MEMS;

extern _MEMS mems;


//-----------------------------------------------------------------
typedef struct {
char connect;
int loss_cnt;
char ros_connect;
char mode;
char mode_ros;
char power_off;
char gait_mode;
float rc_spd_cmd[3];
float rc_pos_cmd[3];
float rc_att_cmd[3];
float rc_att_rate_cmd[3];
}_SDK;

extern _SDK moco_sdk;

void simple_test(float dt);
#endif 
