#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_
#include "base_struct.h"
#define SPI_BUF_SIZE 256
extern int mem_connect;
extern int mem_connect_c;

typedef struct{
    char connect;
    char connect_lock;
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
    int sbus_rc_main[6];
    int sbus_rc_main_reg[6];
    int sbus_mode,sbus_mode_reg;
    int sbus_mode_e,sbus_mode_e_reg;
    float sbus_height,sbus_height_reg;
    float sbus_ch[6];
    int sbus_aux[6];
    int sbus_aux_reg[6];
}_OCU_RX;

typedef struct{
 int rssi;
 float dis,angle;
}_AOA_RX;


typedef struct
{
  int connect;
  char power;
  int cap;
  int mode;
  int loss_connect;
  float att_set[3];
  END_POS pos_set;
  float att_now[3];
  END_POS pos_now;
  float move_spd;
  float torque;
} _ARMSS;


typedef struct
{
float att[3];
float att_rate[3];
float acc_b[3];
float acc_n[3];
float att_usb[3];
float att_rate_usb[3];
float acc_b_usb[3];
float q[14];
float dq[14];
float tau[14];
float q_servo[14];
float dq_servo[14];
float tau_servo[14];
float bat_v;
char connect[14];
char connect_motor[14];
char connect_servo[14];
char ready[14];
char imu_mems_connect;
_OCU_RX ocu;
_AOA_RX aoa;
_ARMSS arm;
}_SPI_RX;

extern _SPI_RX spi_rx;

typedef struct
{
float q_set[14];
float q_set_servo[14];
float q_reset[14];
float q_reset_servo[14];
float tau_ff[14];
float tau_ff_servo[14];
float max_i[14];
float kp[14],ki[14],kd[14];
float kp_servo[14],ki_servo[14],kd_servo[14];
float stiff[14];
float stiff_servo[14];
char en_motor,reser_q,reser_q_div[14],reser_q_div_servo[14],reset_err;
char en_servo;
char led_enable[2];
char beep_state;
_ARMSS arm_cmd_s;
}_SPI_TX;

extern _SPI_TX spi_tx;

//----------------------------------
typedef struct
{
char en_record,gait_state,rc_mode;
char is_touch[4],is_ground[4];
char force_en_flag[4];
char leg_state[4];
char leg_connect[4],motor_connect[4][3],motor_ready[4][3];
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
float dcom_n_tar[3];
float dcom_n_now[3];
float ground_att_now[3];
float att_now[3];
float datt_now[3];
float acc_n_now[3];
float att_tar[3];
float datt_tar[3];
float temp_record[10];

char sbus_connect;
float sbus_ch[4];
int sbus_aux[4];
}_NAV_RX;

extern _NAV_RX nav_rx;

typedef struct
{
  char flag_map;
  float map2d_local[80][80];
  float odom_pos[3];
  float odom_spd[3];
  float odom_att[3];
  float odom_rate[3];
} _MAP;

typedef struct
{
    char connect;
    char ocu_mode;
    float rc_spd_b[3],rc_rate_b[3];
    int key_ud,key_lr,key_x,key_y,key_a,key_b,key_ll,key_rr,key_st,key_back;
    char sbus_connect;
    float sbus_ch[4];
    int sbus_aux[4];
//SDK
    char request_gait;
    float exp_spd_o[3];
    float exp_datt_o[3];
    float exp_att_o[3];
    float exp_pos_o[3];
    float exp_q_o[4][3];
    END_POS exp_GRF_o[4];
    _MAP map;
}_NAV_TX;

extern _NAV_TX nav_tx;

typedef struct
{
    float x;
    float y;
    float z;
}xyz_f_t;

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
    char cal_sel;
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

#define SPI_TEST 0

#define MEM_SPI 0001
#define MEM_CONTROL 4999
#define MEM_SIZE  2048// 2048

extern pthread_mutex_t lock;
void* Thread_Mem_Servo(void*);//内存管理线程
void* Thread_Mem_Navigation(void*);//内存管理线程
void* Thread_Mem_All(void*);//内存管理线程
#endif
