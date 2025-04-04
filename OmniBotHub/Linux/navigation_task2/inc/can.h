#ifndef __CAN1_H
#define __CAN1_H	 
#define RUN_WEBOTS 1
#if !RUN_WEBOTS
#include "sys.h"	 
#include "stm32f4xx_can.h" 
#define CAN_ANL_MIT_MODE 
//主控
#define CAN_FB_SYNC 1//同步采用
#define CAN_NART_SEL DISABLE//ENABLE//DISABLE//ENABLE
//DISABLE  底层节点没法收到  但是能保证不加延时
#define BAUD_2M 1


#define CAN_T_DIV 500.0
#define CAN_I_DIV 100.0
#define CAN_F_DIV 100.0
#define CAN_POS_DIV 50.0
#define CAN_DPOS_DIV 20.0
#define CAN_GAIN_DIV_P 1000.0
#define CAN_GAIN_DIV_D 10000.0

#define CAN_SD_PARAM_SYS_HEAD  0
#define CAN_SD_PARAM_T_HEAD  20

#define CAN_FB_STATE_HEAD  40
#define CAN_FB_POS_HEAD  60
#define CAN_FB_SPD_HEAD  80
#define CAN_FB_MIT_HEAD  100

#define CAN_SD_T_HEAD  120
#define CAN_SD_I_HEAD  140
#define CAN_SD_POS_HEAD  160
#define CAN_SD_SPD_HEAD  180
#define CAN_SD_MIT_HEAD  200

#define CAN_FB_REMOTE_FB1    400
#define CAN_FB_REMOTE_FB2    401

#define P_MIN_MIT -180 //Degree
#define P_MAX_MIT  180
#define V_MIN_MIT -2000
#define V_MAX_MIT  2000
#define T_MIN_MIT -10.0f
#define T_MAX_MIT  10.0f
#define KP_MIN_MIT 0.0f
#define KP_MAX_MIT 10.0f
#define KD_MIN_MIT 0.0f
#define KD_MAX_MIT 10.0f

#define MIT_R_T_D 9.549
#define MIT_D_T_R 0.1047

//CAN1接收RX0中断使能
#define CAN_ABOM_E 1

#define CAN1_RX0_INT_ENABLE	1	//0,不使能;1,使能.								    
extern int can_rx_over[5];
extern int can_rx_cnt[5];							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
extern u8 canbuft1[8];
extern u8 canbufr1[8];
extern uint32_t can1_rx_id;
extern float cnt_rst1;
extern int can1_rx_cnt;
void data_can_anal_master(u8 rx_data[8]);

//CAN1接收RX0中断使能
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
								 							 				    
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode);//CAN初始化
 
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN2_Receive_Msg(u8 *buf);							//接收数据
extern u8 canbuft2[8];
extern u8 canbufr2[8];
extern uint32_t can2_rx_id;
extern float cnt_rst2;
extern int can2_rx_cnt;
#endif
#define MOTOR_MODE_POS 0
#define MOTOR_MODE_T	 1
#define MOTOR_MODE_CURRENT	2
typedef struct 
{
	char connect,connect_motor[3];
	char ready[3];
	int loss_cnt;
	char reset_q,reset_err;
	char motor_en,motor_en_reg;
	char motor_mode,motor_mode_reg;
	char err_flag[2];
	float bat_v[2];
	float temp[2];
	float q_now[3],qd_now[3],qdd_now[3];
	float q_set[3],qd_set[3],qdd_set[3];
	float f_now[3];
	float f_set[3];
	float t_now[3];
	float q_reset[3];
	float set_t[3],set_t_flt[3];
	float set_i[3],set_i_flt[3];
	float max_t[3];
	float max_i[3];
	float t_to_i[3];
	char can_bus_id;
}_LEG_MOTOR;
extern _LEG_MOTOR leg_motor[4];
extern char reset_err_flag;
void CAN_motor_init(void);
void reset_current_cmd(char id);
#if !RUN_WEBOTS
void CAN_set_torque(char id);
void CAN_reset_q(char id);
void CAN_motor_sm(float dt);
void CAN_set_zero_off(char id);
void CAN_get_fb(char can_sel);//遥控反馈获取
extern  float can_dt[4];
#endif
#endif

















