#ifndef _ESO_RL_H_
#define	_ESO_RL_H_
#include "math.h"
#include "base_struct.h"
/*
Copyright(C) 2017 DianYin Innovations Corporation. All rights reserved.

OLDX�ɿ�ʹ�� ADRC controller �Կ��ſ����� designed by Golaced from ���ݴ���(��ӥ�Ƽ�)

qqȺ:
567423074
�Ա����̣�
https://shop124436104.taobao.com/?spm=2013.1.1000126.2.iNj4nZ
�ֻ����̵�ַ��
http://shop124436104.m.taobao.com

ADRC.lib ��װ����̬���ƺ�λ�ÿ��ƿ⺯��������ʵ�ֿ��Ķ����ϵ��Կ���ԭ��
����v�ǿ���ϵͳ�����룬y�ǿ���ϵͳ�������������ESO��u��ADRC�����������
ע�⣺������������ʹ�ú���ɵ�ը������
*/

#define ESO_RL_PARA_USE_REAL_TIME 1
typedef struct
{ 	//ESO parameters
	float alpha_v=20.0;
	float beta_ratio=1.0;
	float wn=50.0;
	float ctrl_gain=100.0;
	float torque_comp_limit=10.0;
	float dt = 0.001;

	//state variables
	float dof_pos_obsr, dof_pos_obsr_error, dof_pos_obsrd; 
	float dof_vel_obsr, dof_vel_obsr_error, dof_vel_obsrd;
	float dof_dist_obsr, dof_dist_obsrd;
	float torque_comp;
	float l1, l2, l3;

	//Flags
	u8 Is_initialized = 0;
}ESO_RL;

void ESO_Init(ESO_RL *eso_in,float dof_pos,float dof_vel);
void ESO_update(ESO_RL *eso_in,float dof_pos,float torque,float dt);
void compute_torque_comp(ESO_RL *eso_in);

extern ESO_RL joint_eso[4][3];


	
#endif

