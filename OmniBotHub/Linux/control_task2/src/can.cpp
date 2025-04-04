#include "include.h"
#include "can.h"
#include "gait_math.h"
#include "locomotion_header.h"
#if !RUN_WEBOTS
#include "delay.h"
#include "led_fc.h"
#endif
_LEG_MOTOR leg_motor[4];
_LEG_MOTOR_ALL leg_motor_all,arm_motor_all;
char reset_err_flag = 0;
void reset_current_cmd(char id)//清除电流
{
	leg_motor[id].set_t[0]=leg_motor[id].set_t[1]=leg_motor[id].set_t[2]=0;
	leg_motor[id].set_i[0]=leg_motor[id].set_i[1]=leg_motor[id].set_i[2]=0;
    for(int i=0;i<14;i++){
        leg_motor_all.set_t[i]=0;
        arm_motor_all.set_t[i]=0;
    }
}
	
void CAN_motor_init(void){
	char i;
	
	for(i=0;i<4;i++){
		leg_motor[i].connect=0;
		leg_motor[i].motor_en=0;
		//leg_motor[i].motor_mode=MOTOR_MODE_CURRENT;	
		leg_motor[i].motor_mode=MOTOR_MODE_T;	//  目前采用力矩模式
		
		reset_current_cmd(i);
		
		leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=3;
		leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=25;
	}
    for(int i=0;i<14;i++){
        leg_motor_all.motor_en=leg_motor_all.servo_en=0;
        arm_motor_all.motor_en=arm_motor_all.servo_en=0;
    }
}



