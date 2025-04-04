#ifndef __LOCOMOITON_H__
#define __LOCOMOITON_H__
#include "base_struct.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include <pthread.h>
extern pthread_mutex_t lock;
extern pthread_mutex_t lock_mpc;
extern YAML::Node config_robot;
extern YAML::Node config_gait;
extern float Www;
extern float Hw;
extern float L1w;
extern float L2w  ;
extern float L3w ;
extern float Mw  ;
extern float I_2_Nm ;
extern float I_VMC[3]  ;
extern float Q_SIDE_BIAS;

extern float min_q1[4];//
extern float min_q2[4];//
extern float min_q3[4];//

extern float max_q1[4];//
extern float max_q2[4];//
extern float max_q3[4];//

extern float ctort_kp;
extern float ctort_ki;
extern float ctort_kd;
extern float tort_kp;
extern float tort_ki;
extern float tort_kd;
extern float init_end_z;

extern float  MESS_KIN1;//=0;//   0.63 //kg
extern float  MESS_KIN12;//=0;//   0.35 //kg
extern float  MESS_KIN2;//=0;//     0.63 //0.46+0.335 //kg
extern float  L_MESS_KIN1;//=0;// 1  0.045
extern float  L_MESS_KIN12;//=0;//  0.08
extern float  L_MESS_KIN2;//=0;//   0.06
//----------------------------------------------VMC controller---------------------
void body_traj_planner(float dt);
void body_servo_control(float dt);
void force_dis_n(void);
void force_dis_n_side_FH(void);
void force_dis_n_side_LR(void);
void reset_servo_interge(void);

//---------------------------------------------Force IMP controller----------------------
void force_control_and_dis_stand(float dt);
void force_control_and_dis_rl(float dt);
void force_control_and_dis_rl_eso(float dt);
void force_control_and_dis_climb(float dt);
void force_control_and_dis_trot(float dt);
void force_control_and_dis_trot_air(float dt);
void force_control_and_dis_trot_local(float dt);
void force_control_and_dis_bound(float dt);
void force_control_and_dis_rolling(float dt);
//---------------------------------------------state estimator-------------------------
typedef struct{
	END_POS acc_n_m;
	END_POS acc_n_f;
	END_POS acc_nn_f;
	
	END_POS odom_spd_n;
	END_POS odom_pos_n;

	END_POS odom_spd_nn;
	END_POS odom_pos_nn;
	
	END_POS odom_dspd_n;
	END_POS odom_dpos_n;
	
	END_POS odom_spd_n_r;
	END_POS odom_pos_n_r;
	float Kpv;
}_ODOM_1;
extern _ODOM_1 odom1;

typedef struct 
{
  int state_gait;
  int auto_switch;
  int auto_gait_time;
  int auto_zmp_st_check;
  int auto_mess_est;
  int auto_mode;
  float switch_timer[10];
}Gait_Mode;

extern Gait_Mode gait_ww;
extern char sit_down_flag;


void reset_robot_statement(void);
void touchdown_check(robotTypeDef* rob, float dt);
void estimate_GRF(float dt);
void state_estimator(float dt);
void estimate_ground_att(float dt);
void Mess_compass(int leg,float dt);
void pay_load_estimation(float dt);
//---------------------------------------------hardware interface---------------------
void subscribe_imu_to_webot(robotTypeDef* rob,float dt);
void subscribe_webot_to_vmc(float dt);
void publish_vmc_to_webot(float dt);
void readAllMotorPos(robotTypeDef* rob,float dt);
void set_motor_q(int id);
void set_motor_t(int id);

extern float sw_t[4] ;//= { 0.4,0.4,0.4,0.4 };
extern float st_t[4];// = { 0.2,0.2,0.2,0.2 };
//---------------------------------------------leg planner--------------------------------
void cal_vir_leg_pos(VMC* in, END_POS *td_vir_n, END_POS *lf_vir_n);
float end_pos_y_correction(VMC* in, float ep_slip_n, float dt);
void trig_curve_global(VMC *in, float *x, float *y, float *z,float *vx, float *vy, float *vz, float dt);
void espd_to_qspd(VMC *in, END_POS spd_end, float* spd_q, float dt);
void reset_sw_torque_as_now(VMC *in, float torque_now[3]);
void trig_plan(char leg_sel_trig, float dt);
char trig_lift(char leg_sel_trig, float dt);
char trig_swing(char leg_sel_trig, float dt);
char trig_td(char leg_sel_trig, float dt);
void swing_spd_control(char leg_sel_trig, float dt);
void reset_tar_pos(char leg_sel_trig);
void trig_plan_online(char leg_sel_trig, float dt);

//---------------------------------------------locomotion SFM--------------------------
typedef struct{
	char sdk_connect;
	int sdk_loss_cnt;
	char sdk_mode;
	char gait_mode;
	char cmd_power;
	char trig_mode;
	float cmd_vx;
	float cmd_vy;
	float cmd_vz;
	float cmd_vyaw;
    float cmd_x,cmd_y,cmd_z;
	float cmd_pit;
	float cmd_rol;
	float cmd_yaw;
	int cmd_pwm[12];
	float cmd_angle[4][3];
	END_POS trig_end_cmd[4];
}_SDK;
extern _SDK sdk;

extern char stand_force_enable_flag[5];
void vmc_param_init(void);
char safe_check(float dt);
void locomotion_sfm(float dt);
void servo_controller(float dt);
char gait_switch(float dt);
char gait_switch_sbus(float dt);
//-------------------------------------Gati  APP-----------------------------
typedef struct
{
	int state;
	int jump_trig;
	float timer[3];
	float jump_x_dis,jump_x_side_off;
	float jump_yaw;
	float jump_w;
	float jump_height;
	float jump_height_off;
	float jump_height_off_forward;
	float jump_power;
	float jump_avg_F[3];
	float jump_avg_T[3];
	float st_height;
	float ch_height;
	float couch_height;
	float damp_height;
	float sw_z;
	float stand_off;
	float lf_time;
	float couch_time;
	float td_time;
	float fly_time, fly_time_real;
	float lf_spd[3];
	float td_spd[3];

	float kp[3];
	float kp_T[3];
}Gait_Pronk;

extern Gait_Pronk gait_pronk;


typedef struct
{
	int state;
	int jump_trig;
	int jump_phase;
	int jump_phase_other;
	int force_control_mode;
	int up_stair;
	float timer[3];
	float jump_spdx;
	float jump_yaw;
	float jump_w;
	float jump_height;
	float jump_height_off;
	float jump_power;
	float jump_avg_F[3];
	float jump_avg_T[3];
	float st_height;
	float ch_height;
	float couch_height;
	float damp_height;
	float lf_time;
	float couch_time;
	float td_time;
	float fly_time, fly_time_real;
	float lf_spd[3];
	float td_spd[3];

	float kp[3];
	float fp[3];
	float kp_T[3];
}Gait_Climb;

extern Gait_Climb gait_climb;



typedef struct
{
    int state_all;
    int first_bound;
    float sw_dh;
    float T_sw;
    float T_st_norm;
    float T_air;
    float k_time;
    float L;
    float vd;
    float exp_z;
    float c;
    float mess;
    float s_peak_Fz;
    float tauP;
}Gait_Bound_IMP_Param;




typedef struct
{
    int state;
    int phase;
    int phase_rl;
    int other_phase;
    int wait_trig;
    float wait_trig_time;
    char touch, ground;
    char id_sw[2];
    END_POS hip_n_exp;
    END_POS hip_n_now;
    END_POS hip_nb_now;
    END_POS dhip_n_exp;
    END_POS dhip_n_now;
    END_POS sw_off;
    float sw_time;
    float att_exp[3];
    float att_now_rate[3];
    float att_now[3];
    float pid_hip[3];
    float pid_hip_x[3];
    float pid_v[3];
    float pid_P[3];
    float pid_R[3];
    float k_jump;
    float T, T_tirg;
    float t;
    float t_start;
    float s, s_sw;
    float T_st_norm;
    float T_st_now, T_st_now_trig;
    float T_st_last;
    float T_sw;
    float T_air;
    float alfa_Fz;
    float alfa_tauP;
    float Fz_imp;
    float TauP_imp;
    float g_st;
    float Fx;
    float Fz;
    float v_inter;
    float time_fly_near;
    float time_fly_near_last;
    float time_phase_off;
}Gait_Bound_IMP;


typedef struct
{
	Gait_Bound_IMP_Param slip_param;
	Gait_Bound_IMP slip[2];
	Gait_Bound_IMP slip_leg[4];
	Gait_Bound_IMP slip_all;
	int state;
	int jump_trig;
	int jump_phase;
	int jump_phase_other;
	int force_control_mode;
	int up_stair;
	float timer[3];
	float jump_spdx;
	float jump_yaw;
	float jump_w;
	float jump_height;
	float jump_height_off;
	float jump_power;
	float jump_avg_F[3];
	float jump_avg_T[3];
	float st_height;
	float ch_height;
	float couch_height;
	float damp_height;
	float lf_time;
	float couch_time;
	float td_time;
	float fly_time, fly_time_real;
	float lf_spd[3];
	float td_spd[3];
	float side_off[2];
	float kp[3];
	float fp[3];
	float kp_T[3];
	float kp_Y[4];
	float yaw_T;
	float yaw_i_T;
	float yaw_control_step;
	float yaw_rate_reg;
}Gait_Double_SLIP;
extern Gait_Double_SLIP gait_bound;

void  Gait_ETL_Active(void);
void  Gait_ETL_Update_v1(float dt);

void  Gait_Stand_Active(void);
void  Gait_Stand_Update(float dt);

void  Gait_RL_Active(char rst);
void  Gait_RL_Update(float dt);

void  Gait_Trot_Active(void);
void  Gait_Trot_Update_v1(float dt);

void  Gait_Recovery_Active(void);
void  Gait_Recovery_Falling(float dt);
void  Gait_Recovery_Update(float dt);

void  Gait_Pronk_Active(void);
void  Gait_Pronk_Update(float dt);

void  Gait_Climb_Active(void);
void  Gait_Climb_Update(float dt);

void  Gait_Bound_Active(void);
void  Gait_Bound_Update(float dt);

void Gait_Walk_Active(void);
void Gait_Walk_Update_v1(float dt);

void Gait_Rolling_Active(char way_sel);
void Gait_Rolling_Update_v1(float dt);

void  Gait_Trot_Active(void);
void  Gait_Trot_Update_v1(float dt);

void Gait_FTrot_Active(void);
void Gait_FTrot_Update_v1(float dt);

void  Gait_Recovery_Active(void);
void  Gait_Recovery_Falling(float dt);
void  Gait_Recovery_Update(float dt);

char move_joint_to_pos(VMC * in, int joint, float tar_angle, float max_spd, float dt);
char move_joint_to_pos1(VMC * in, int joint, float tar_angle, float max_spd, float err_check, float dt);
char move_joint_to_pos_sel(VMC * in, int joint, float tar_angle, float max_spd, float err_check, char sel,float dt);
void move_joint_with_spd(VMC * in, int joint, float tar_spd, float dt);
float move_joint_to_pos_all(float now, float tar_angle, float max_spd,float dt);
char check_lisence(void);
void robot_param_read(void);


//=============================SDK API
typedef struct
{
    char check;
    END_POS pos_tar;
    END_POS pos;
    float att[3];
    float att_tar[3];
}_TAR_WAY_POINT;
extern _TAR_WAY_POINT sdk_tar_way_point_uwb;

typedef struct
{
    char enable;
    END_POS tar_pos;
    END_POS tar_spd;
    float tar_att[3];
}_SDK_CONTROL;
extern _SDK_CONTROL sdk_control;

int track_point_mode_uwb(_TAR_WAY_POINT* tar_point, float dt);
int human_guide_mode_uwb(_TAR_WAY_POINT* tar_point, _SDK* sdk,float dt);

//----------------------Vision mapper------------------
#define FMAP_NUM 3
#define MAP_W 80//高程地图
#define MAP_H 80

#define FMAP_W 40// 人工地图 离线设置
#define FMAP_H 40

#define GMAP_W 100//猜测地图
#define GMAP_H 100
#define MAP_SIZE 0.02
#if 0//traj 同步修改
#define CUBE_NUM 28//28
#define CUBE_W 0.25

#define WAY_POINT_NUM 5//5
#else
#define CUBE_NUM 50+1//28
#define CUBE_W 0.25

#define WAY_POINT_NUM 14//5
#endif
typedef struct
{
    END_POS map_origin_pos_b;
    float map[GMAP_H][GMAP_W];
    END_POS grid_pos[GMAP_H][GMAP_W];
}_HEIGHT_MAP;
extern _HEIGHT_MAP fake_map_stair_12cm, fake_map_trench_5cm, guass_map;

#define MAP_W_N 5600
#define MAP_H_N 5600
typedef struct
{
    char init_odom;

    END_POS robot_cog;
    END_POS robot_cog_off_real;
    END_POS robot_vel;
    END_POS leg_pos[4];
    END_POS leg_tar[4][20];
    float robot_att[3];
    float h_grid[MAP_H_N][MAP_W_N];
    END_POS h_grid_pos[MAP_H_N][MAP_W_N];
    //相对质心的局部地图
    float h_grid_local[MAP_H][MAP_W];
    float h_grid_local_good[MAP_H][MAP_W];
    float h_grid_local_edge[MAP_H][MAP_W];
    END_POS grid_pos_local[MAP_H][MAP_W];
    float stand_map_z_off;
    END_POS way_point[20];
    END_POS flag_n[20];//先验地形标志
}_GLOBAL_MAP;
extern _HEIGHT_MAP fake_map_stair_12cm, fake_map_trench_5cm,guass_map;
extern _GLOBAL_MAP global_map;
int fake_map_init(_HEIGHT_MAP* map_fake, float x_off_n, float y_off_n, float rotate_yaw, char map_sel);
int fake_map_link_init(void);
typedef struct
{
    END_POS leg_sw_tar[4];
    END_POS leg_sw_slip[4];
    int ground[4], touch[4];
    END_POS tar_grf[4];
    END_POS mu_limit[4];
    double force_rotate[4][4];
    double force_size[4];
    double mu_rotate[4][4];
    double mu_size[4];
    double mu_r[4];
    END_POS sw_tar_traj[4][1024];
    END_POS sw_tar_traj_webots[4][1024];
    int sw_traj_cnt[4];
    END_POS cog_pos;
    float cog_att[3];

    float grid_m[328][328];
    float grid_m_good[328][328];
    float grid_m_edge[328][328];
    float fgrid_m[FMAP_NUM][328][328];
    float grid_m_base_z;
    int en_draw_map;

    END_POS zmp_pos;
    END_POS tar_zmp_pos;
    END_POS support_center;
    //human
    float grid_wide;
}WEBTOS_DRAWING;


extern WEBTOS_DRAWING webots_draw;

typedef struct
{

    END_POS cog_real;
    END_POS leg_end_real[4];
    END_POS fake_map_cog[10];
    double fake_map_ori[10][4];

    END_POS cube_cog[CUBE_NUM];
    double cube_ori[CUBE_NUM][4];

    END_POS flag_pos[20];
    END_POS way_point_pos[20];
}WEBTOS_DRAWING_RX;
extern WEBTOS_DRAWING_RX webots_draw_rx;

void share_memory_drawing(void);
int move_robot_on_map_n(float dt);
int location_select_on_map(char sel, END_POS slip_n, char mode, END_POS *location);

#define FMAP_USE_SIMU 1
#define POS_USE_SIMU 0
#define HU_TASK1 0 //地形切换
#define HU_TASK2 1 //视觉落足
#define HU_TASK3 2 //台阶斜坡
#define HU_TASK4 3 //梅花桩

#define HU_TASK  HU_TASK2

typedef struct
{
    int env_type;//地形种类
    float complus;//复杂度
    float slape;//地形崎岖度
    float cute;//地形割裂度
    float slope[2]; //坡度
    float mu;//滑移率
    float deep;//沉陷度
}_HU_ENV;
extern  _HU_ENV _hu_env;

typedef struct
{
    int mode_en;
    int gait_mode;
    int gait_switching;
    int way_point_id;
    int en_traj_follow;
    float cmd_att[3];
    float cmd_spd[3];
    float cmd_spd_limit[3];
    float cmd_height;
    float decide_range;//注意力 m
    float local_map_grid;//局部地图精度
    float gait_duty;
    float gait_delay;
    float sw_z;

    int phase_mode;//0 lsm 1elt
    int balance_mode;//0 vmc 1 mpc
    int perdict_win;//预测窗口

    int stair_slope_mode_flag;//0 normal   1 斜坡    2楼梯
    int slip_vision_mode_flag;//0 内部模型 1外部模型 2智能切换
    int imp_ff_mode_flag;     //0 力模式   1混合模式

    int hu_mode;//动觉智能模型
    int fake_map_guass;
    int guass_map_type;

    float stable_rate;//稳定性
    float power;//功耗统计
    float cpu_used;//算力消耗

    float control_rate_mid;//中枢控制频率
    float control_rate_low;//末梢控制频率

    float time_trig[30];//时间戳
    float time_trig_reg[30];
    float dt_app[30];
}_HU_MODE;
extern  _HU_MODE _hu_model;
#endif 
