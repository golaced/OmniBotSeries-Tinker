#ifndef _MEM_NODE_H_
#define _MEM_NODE_H_
#include "base_struct.h"

//----------------------------------
typedef struct
{
char en_record,gait_state,rc_mode;
char is_touch[4],is_ground[4];
char force_en_flag[4];
char leg_state[4];
char leg_connect[4],motor_connect[4][3],motor_ready[4][3];
char motor_connect_all[14];
float q_now[14];
float t_now[14];
float q_set[14];
float t_set[14];
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
}_NAV_RX;

extern _NAV_RX nav_rx;


typedef struct{
 int power;
 int arm_mode;
 int control_mode;
 int cap;
 END_POS pos_set_n;
 END_POS pos_set_b;
 END_POS spd_set_n;
 END_POS spd_set_b;
 END_POS att_set_n;
 END_POS att_set_b;
 END_POS rate_set_n;
 END_POS rate_set_b;
 float q_set[7];
 int lock_q[7];
}_ARM_OCU;

typedef struct
{
  char flag_map;
  int16_t map2d_local[80][80];
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
//SDK
    char request_gait;
    float exp_spd_o[3];
    float exp_att_o[3];
    float exp_datt_o[3];
    float exp_pos_o[3];
    float exp_q_o[4][3];
    END_POS exp_GRF_o[4];
    _ARM_OCU arm_ocu;
    _MAP map;
}_NAV_TX;

extern _NAV_TX nav_tx;

#endif
