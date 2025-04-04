#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);



u32 Get_Cycle_T(u8 );
void Cycle_Time_Init(void);
uint32_t GetSysTime_us(void);
#define GET_T_OUTTER 0
#define GET_T_INNER 1
#define GET_T_FLOW_UART 2
#define GET_T_SONAR_SAMPLE 3
#define GET_T_BARO_UKF 4
#define GET_T_EKF 5
#define GET_T_MS5611 6
#define GET_T_INNER_TIM 7
#define GET_T_INNER_TIM_USE 12
#define GET_T_INNER_TIM_IMU 8
#define GET_T_HIGH_CONTROL_I 9
#define GET_T_HIGH_CONTROL_O 10
#define GET_T_OUTTER_C 11
#define GET_T_HIGH_CONTROL 12
#define GET_T_HML_CAL 13
#define GET_T_OUT_NAV 14
#define GET_T_IN_NAV 15
#define GET_T_MEMS 16
#define GET_T_IMU 17
#define GET_T_IMU_UKF 18
#define GET_T_IMU_YAW 
#define micros() TIM3->CNT
void TIM3_Config(void);
#endif

//------------------End of File----------------------------
