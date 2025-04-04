#ifndef _OCU_H_
#define _OCU_H_
#include <TFT_eSPI.h>

#define USE_XBOX_RC 0

#define GET_TIME_NUM 	(10)		 
#define M_SAFE 0
#define M_STAND_RC 1  
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
#define M_SOFT 14
#define M_C_TROT 15
#define M_C_JOG 16
#define M_ROLLING 17
#define M_ETL 18
 
#define CAL_Q 1
#define CAL_IMU 2
#define B_MODE  3

#define BUTTON_WIN 0
#define BUTTON_RC  1


#define RAD_TO_DEGw  57.3
#define DEG_TO_RADw  0.0173

#define PITr 0
#define ROLr 1
#define YAWr 2
#define Xr 0
#define Yr 1
#define Zr 2
#define Ls 0
#define Rs 1
#define FL 0
#define BL 1
#define FL1 0
#define BL1 1
#define FL2 2
#define BL2 3

typedef struct
{
  float x;
  float y;
  float z;
} END_POS;

typedef struct
{
  float x;   
  float y;   
  float z;   
}Vect3;

typedef struct
{
  char en_record, gait_state;
  char is_touch[4], is_ground[4];
  char force_en_flag[4];
  char leg_state[4];
  char leg_connect[4], motor_connect[4][3], motor_ready[4][3];
  END_POS epos_n_tar[4];
  END_POS epos_n_now[4];
  END_POS depos_n_tar[4];
  END_POS depos_n_now[4];
  float sita_tar[4][3];
  float sita_now[4][3];
  float tau_tar[4][3];
  float tau_now[4][3];
  END_POS GRF_n_tar[4];
  END_POS GRF_n_now[4];
  float com_n_tar[3];
  float com_n_now[3];
  float acc_n_now[3];
  float dcom_n_tar[3];
  float dcom_n_now[3];
  float ground_att_now[3];
  float att_now[3];
  float datt_now[3];
  float att_tar[3];
  float datt_tar[3];
  float temp_record[10];
  float sita1_off[4];
  float sita2_off[4];
  float sita3_off[4];
  END_POS cog_pos_n, cog_spd_n;
  int  robot_mode;
  //--------ocu set
  int gait_mode;
  int sys_mode;
  float bat_robot;
  float bat_rc;
  int cal_imu;
  int cal_q;
  int shut_down;
} _NAV_RX;

typedef struct {
  char en_record;
  char is_touch[4], is_ground[4];
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
} _RECORD;

typedef struct {
  char connect;
  int loss_cnt;
  char mode;
  char up_mode;
  float rc_spd_b[3], rc_rate_b[3];

  char cmd_robot_state;
  float rc_spd_w[2], rc_att_w[2], rate_yaw_w;
  int key_ud, key_lr, key_x, key_y, key_a, key_b, key_ll, key_rr, key_st, key_back, key_ud_reg, key_lr_reg, key_x_reg, key_y_reg, key_a_reg, key_b_reg, key_ll_reg, key_rr_reg, key_st_reg, key_back_reg;
  float curve[20];

  char sbus_conncect;
  char sbus_power_sw, sbus_power_sw_reg;
  int sbus_rc_main[4];
  int sbus_rc_main_reg[4];
  int sbus_mode, sbus_mode_reg;
  int sbus_mode_e, sbus_mode_e_reg;
  float sbus_height, sbus_height_reg;
  int sbus_aux[4];
  int sbus_aux_reg[4];

  _RECORD record;
} _OCU;

//------------------------------------------------------------------------------
typedef struct 
{
  char id,trig_state,ground_state,invert_knee;
  int invert_knee_epos[2];
}PARAM;

typedef struct 
{
  float l1,l2,l3,l4,r;
  float tar_sita1,tar_sita2,tar_sita3;
  float sita1,sita2,sita3,sita;
  END_POS epos,epos_b,epos_n,epos_reg,epos_regn,epos_vm,epos_n_b;
  END_POS tar_epos_b,tar_epos_b_reg,tar_epos_n_sw;
  END_POS spd,spd_n,spd_b,spd_o;
  END_POS hip_pos_b,kin1_pos_b,kin12_pos_b,kin2_pos_b;
  END_POS hip_pos_n,kin1_pos_n,kin12_pos_n,kin2_pos_n;
  float jacobi33[9],ijacobi33[9];
  float jacobi33_reg[9],d_jacobi33[9];
  float jacobi33_kin1[9],ijacobi33_kin1[9];
  float jacobi33_kin12[9],ijacobi33_kin12[9];
  float jacobi33_kin2[9],ijacobi33_kin2[9];
  float jacobi22[4],ijacobi22[4];
  char ground,ground_noshock,ground_s,is_touch;
  int flag_fb,flag_rl;
	PARAM param;
}VMC;


typedef struct
{
  int gait_mode, gait_mode_reg;
  char trot_state, trot_phase, trot_air_test;
  char ground[2][4];
  float l1, l2, l3, l4, W, H, mess;
  float Rb_n[3][3];
  float Rb_n_noatt[3][3];
  float Rn_b[3][3];
  float Rn_b_noatt[3][3];
  END_POS tar_spd_v,tar_spd_vn;
  END_POS spd_n,spd_b;
  END_POS odom_n,odom_b;
  END_POS terrain_map[50][2];
  float test_att_flt[3];
  float test_pos_flt[3];
  float MAX_Z,MIN_Z,MAX_X,MIN_X,MAX_Y,MIN_Y,MAX_PIT,MAX_ROL,MAX_YAW;
}VMC_ALL;

extern VMC vmc[4];
extern VMC_ALL vmc_all;



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

typedef struct
{
  float roll;       //横滚，x轴
  float pitch;      //俯仰，y轴
  float yaw;        //偏航，z轴
  float yaw_off;
}eulerAngleTypeDef;

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
  Vect3               cog_pos_nn;

  Vect3               cog_F_ff;
  Vect3               cog_T_ff;

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
  float Rn_gg[3][3],Rgg_n[3][3];
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

//----------------------------------------------------------------------------
enum {
  NOW = 0,
  OLD,
  NEW,
};

#define F_PI 3.1415926
#define LPF_COF_05Hz  1.0f/(2*F_PI*0.5)
#define LPF_COF_1t5Hz  1.0f/(2*F_PI*3)
#define LPF_COF_5t10Hz  1.0f/(2*F_PI*7)
#define LPF_COF_10t15Hz  1.0f/(2*F_PI*12)
#define LPF_COF_15t20Hz  1.0f/(2*F_PI*17)
#define LPF_COF_20t25Hz  1.0f/(2*F_PI*22)
#define LPF_COF_25t30Hz  1.0f/(2*F_PI*27)
#define LPF_COF_30t50Hz  1.0f/(2*F_PI*40)
#define LPF_COF_50t70Hz  1.0f/(2*F_PI*60)
#define LPF_COF_70t100Hz  1.0f/(2*F_PI*80)
#define LPF_COF_100tHz  1.0f/(2*F_PI*100)

void robot_setup(int sel);
void robot_loop(int pit,int rol,int yaw,float dt);
void beep_setup();
void playTone(int tone, int duration) ;
void playNote(char note, int duration);


void setup_xbox();
void loop_xbox();
void start_loop();
void start_setup();
extern int rc_connect;
extern int usb_send_cnt;
extern char SendBuff_USB[500];
extern char RxBuffer_USB[255];
void send_udp1(void);
void send_udp2(void);
void UDP_RX_PROCESS(char *Buf, int Len);
//
float Get_Cycle_T(int item);
void Cycle_Time_Init();
//
float LIMIT(float in, float min, float max);
int generateJudgment(String str);
void DigitalLPF(float in, float* out, float cutoff_freq, float dt);
void converV_n_to_b_w_yaw(float yaw, float xn, float yn, float zn, float *xb, float *yb, float *zb);
void converV_b_to_leg(char leg, float xb, float yb, float zb, float *xl, float *yl, float *zl);
void converV_b_to_n_RT(float RT[3][3], float yaw, float xb, float yb, float zb, float *xn, float *yn, float *zn);
//
void draw_data(TFT_eSprite tft_in, float data, int X, int Y) ;
void draw_content(TFT_eSprite tft_in, char *data, int X, int Y) ;
void draw_float(TFT_eSprite tft_in, float data, int X, int Y);

extern int wifi_search;
#endif
