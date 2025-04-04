#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "head.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

typedef struct PID_STA{u16 OP,OI,OD,IP,II,ID,YP,YI,YD;}PID_STA;
extern PID_STA HPID,SPID,FIX_PID,NAV_PID;
void Nrf_Check_Event(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
extern unsigned int cnt_timer2;
extern void RC_Send_Task(void);
extern int BLE_DEBUG[16];

extern u8 height_mode,pos_mode;
extern u8	(up_load_set);
extern u8	(up_load_pid);
extern u16 yaw_sb;

extern u8 has_tx,gps_f_connect_flag,send_pid;


extern double GPS_W,GPS_J;
extern float ypr[3];
extern u16 high_f,bat_fly;
extern u8 gps_mode,fly_mode,gps_no;

extern PID_STA HPIDt,SPIDt;		
extern  u16 ax,ay,az,gx,gy,gz,hx,hy,hz,YM,PWM1,PWM2,PWM3,PWM4,fix_pit,fix_rol;
extern u8 data_rate,fly_mode;
extern void CAL_CHECK(void);

extern int rc_rate_cnt[10];
extern int DEBUG[35];

struct RC_GETDATA{
				int16_t ROL;
				int16_t PIT;
				int16_t THROTTLE;
				int16_t YA;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				u8 RST;};

extern struct RC_GETDATA Rc_Get;//接收到的RC数据,1000~2000
				
				
struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //原始值
	   struct _float averag;  //平均值
	   struct _float histor;  //历史值
	   struct _int16 quiet;   //静态值
	   struct _float radian;  //弧度值 
          };

struct _alt{
int32_t altitude;
float altitude_f;
int32_t Temperature;
int32_t Pressure;
int Temperat;
};
					
struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
	struct _trans hmc;
	struct _alt alt;
              };

extern struct _sensor sensor;	

	
struct Module{   
u8 gps,vision,flow,hml,gyro,bmp,acc,rc,sonar;
};

struct _speed{   
	int altitude;
	int bmp;
	int pitch;
	int roll;
	int gps;
	int filter;
	int sonar;
	int acc;
              };

struct _altitude{   
	int bmp;
	int sonar;
	int gps;
	int acc;
	int filter;
              };
struct _angle{   
float pitch;
float roll;
float yaw;
              };

							
struct _get{   
	struct _speed speed;
	struct _altitude altitude;
	struct _angle AngE;
              };

struct Mission{   
	float t_spd[3];
	float t_pos[3];
	float t_att[3];
	char mains,subs,smarts,wayps;
              };

struct _plane{   
	float att[3];
	float spd[3];
	float pos[3];
	u8 lock,mode,state_v,acc3d_step,bat,read_pid;
	int PID[20][3];
	int PID_RX[20][3];
	int rc_lost_cnt,rssi;
	u8 gps_sv,acc_cal,acc_3d_cal,gyro_cal,mag_cal,pos_sensor_state;
	struct Module module;
	struct Mission mission;
	double lat_d,lon_d;
              };

extern struct _plane plane;	
							
							
struct _slam{   
	 int16_t spd[5];
	 int16_t dis[5];
              };
extern struct _slam slam;	
							
							
struct _SPEED_FLOW_NAV{
float west;
float east;
float x;
float y;
float x_f;
float y_f;

};	
struct _POS_FLOW_NAV {
float	east;
float	west;
long LAT;
long LON;
long Weidu_Dig;
long Jingdu_Dig;
u8 flow_qual;
};
struct _POS_GPS_NAV {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
};

struct _FLOW_NAV{
struct _SPEED_FLOW_NAV speed;
struct _SPEED_FLOW_NAV speed_h;	
struct _POS_FLOW_NAV position;
	u8 rate;
};	

struct _IMU_NAV{   
struct _FLOW_NAV flow;
struct _POS_GPS_NAV gps;
	
};
extern struct _IMU_NAV imu_nav;
	

struct _FLOW_DEBUG{   
int ax,ay,az,gx,gy,gz,hx,hy,hz;
u8 en_ble_debug;
};
extern struct _FLOW_DEBUG flow_debug;


struct _PID2{
float p;
float i;
float d;
float i_max;
float limit;
float dead;	
float dead2;	
};
struct _PID1{
struct _PID2 out;
struct _PID2 in;	
};
struct _PID_SET{
struct _PID1 nav;
struct _PID1 high;
struct _PID1 avoid;
struct _PID1 marker;
struct _PID1 circle;
};
extern struct _PID_SET pid;

struct _MODE
{
u8 thr_fix;
u8 en_pid_out_pit;
u8 en_pid_out_rol;
u8 en_pid_out_yaw;
u8 en_pid_fuzzy_p;
u8 en_pid_sb_set;
u8 en_fuzzy_angle_pid;
u8 en_sensor_equal_flp;
u8 pit_rol_pid_out_flp;	
u8 en_pid_yaw_angle_control;	
u8 en_pid_yaw_control_by_pit_rol;	
u8 thr_add_pos;
u8 spid;
u8 mpu6050_bw_42;
u8 en_imu_q_update_mine;	
u8 en_moto_smooth;
u8 pid_mode;
u8 no_head;
u8 sonar_avoid;	
u8 yaw_sel;
u8 sb_smooth;
u8 use_px4_err;
u8 flow_hold_position;
u8 dji_sonar_high;
u8 auto_fly_up,auto_land;
u8 en_circle_nav,circle_miss_fly,en_track_forward,en_circle_locate;
u8 flow_hold_position_use_global;
u8 flow_hold_position_high_fix;
u8 height_safe;
u8 use_ano_bmp_spd;
u8 tune_ctrl_angle_offset;
u8 imu_use_mid_down;
u8 hunman_pid;
u8 yaw_imu_control;	
u8 cal_sel;
u8 flow_sel;
u8 height_in_speed;
u8 height_upload;
u8 en_h_mode_switch;
u8 en_dj_cal;
u8 en_sd_save;
u8 en_break;
u8 use_dji;
u8 en_marker;
u8 rc_control_flow_spd;
u8 rc_control_flow_pos;
u8 rc_control_flow_pos_sel;
u8 dj_by_hand;
u8 en_dj_control;
u8 dj_yaw_follow;
u8 dji_mode;
u8 en_visual_control;
u8 hold_use_flow;
u8 en_sonar_avoid;
u8 thr_fix_test;
u8 en_imu_ekf;
u8 att_pid_tune;
u8 high_pid_tune;
u8 dj_lock;
u8 en_eso;
u8 en_eso_h_out;
u8 en_eso_h_in;
u8 en_hinf_height_spd;
u8 en_circle_control;
u8 save_video;
u8 en_h_inf;
u8 test1;
u8 test2;
u8 test3;
u8 test4;	
u8 mode_fly;

//flow
u8 en_flow_gro_fix;
u8 flow_size;
};
extern int CH[5];
extern int qr_pos[2],tar_pos[2],drone_pos[3];
extern int yaw_gimbal;
extern u8 state_drone,need_avoid,tar_check;

#endif
