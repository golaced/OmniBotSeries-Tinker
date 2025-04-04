#include "locomotion_header.h"
#include "gait_math.h"
#include "eso.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "include.h"
float k_roll_reset=1.5;
char self_right_state=0;
static float timer_self_reset[5]={0};
float side_press_safe_45[3]={120,45,-35};
float side_free_safe_45[3]={60,110,-3};

float side_press_reset_1[3]={120,110,-35};
float side_free_reset_1[3]={60,110,-3};

float side_press_reset_2[3]={120,110,0};
float side_free_reset_2[3]={90,50,-3};

float side_press_reset_3[3]={120,90,0};
float side_free_reset_3[3]={120,35,-3};

float side_press_reset_4[3]={130,35,-3};
float side_free_reset_4[3]={130,35,-3};

float side_press_reset_5[3]={95,30,-3};
float side_free_reset_5[3]={95,30,-3};

float k_self_right_p=35;
float max_spd_reset=66;
char move_joint_to_pos(VMC * in, int joint, float tar_angle, float max_spd, float dt)
{
	float now=0;
	float err=0;
	float out=0;
	switch(joint)
	{
		case 0:
		now=in->sita1;
		break;
		case 1:
		now=in->sita2;
		break;
		case 2:
		now=in->sita3;
		break;		
	}
	
	err=tar_angle-now;
	out=now+LIMIT(err*k_self_right_p,-max_spd,max_spd)*dt;
	
	switch(joint)
	{
		case 0:
		in->sita1=out;
		break;
		case 1:
		in->sita2=out;
		break;
		case 2:
		in->sita3=out;
		break;		
	}
	
	if(fabs(err)<0.5)
		return 1;
	else
		return 0;
}


float move_joint_to_pos_all(float now, float tar_angle, float max_spd,float dt)
{

    float err = 0;
    float out = 0;

    err = (tar_angle - now);

    out = now + LIMIT(err*k_self_right_p, -max_spd, max_spd) / 2.*dt;

    return out;
}

char move_joint_to_pos_sel(VMC * in, int joint, float tar_angle, float max_spd, float err_check, char sel,float dt)
{
    float now = 0;
    float err = 0;
    float out = 0;
    switch (joint)
    {
    case 0:
        now = in->tar_sita1;
        break;
    case 1:
        now = in->tar_sita2;
        break;
    case 2:
        now = in->tar_sita3;
        break;
    }

    float alfa=(in->sita2-in->sita1)/2;
    float beta=(in->sita2+in->sita1)/2;
    if(sel){
        err = To_180_degreesw(tar_angle - now);
        if((in->param.id==0||in->param.id==2)&&0)
            printf("id=%d joint=%d err=%f now=%f beta=%f alfa=%f\n",in->param.id,joint,err,now,beta,alfa);
        if(beta>-30&&beta<0&&(in->param.id==0||in->param.id==2)&&1)
            ;
        else if(err<-90&&(in->param.id==0||in->param.id==2)&&1)
            err = (tar_angle - now);
    }
    else
        err = (tar_angle - now);

    out = now + LIMIT(err*k_self_right_p, -max_spd, max_spd) / 2.*dt;

    switch (joint)
    {
    case 0:
        in->tar_sita1 = out;
        break;
    case 1:
        in->tar_sita2 = out;
        break;
    case 2:
        in->tar_sita3 = out;
        break;
    }

    if (fabs(err) < err_check)
        return 1;
    else
        return 0;
}

char move_joint_to_pos1(VMC * in, int joint, float tar_angle, float max_spd, float err_check, float dt)
{
	float now=0;
	float err=0;
	float out=0;
	switch(joint)
	{
		case 0:
		now=in->tar_sita1;
		break;
		case 1:
		now=in->tar_sita2;
		break;
		case 2:
		now=in->tar_sita3;
		break;		
	}
	
	err=tar_angle-now;
	out=now+LIMIT(err*k_self_right_p,-max_spd,max_spd)/2.*dt;
	
	switch(joint)
	{
		case 0:
		in->tar_sita1=out;
		break;
		case 1:
		in->tar_sita2=out;
		break;
		case 2:
		in->tar_sita3=out;
		break;		
	}
	
	if(fabs(err)<err_check)
		return 1;
	else
		return 0;
}

void move_joint_with_spd(VMC * in, int joint, float tar_spd, float dt)
{
	float now=0;
	float err=0;
	float out=0;
	switch(joint)
	{
		case 0:
		now=in->tar_sita1;
		break;
		case 1:
		now=in->tar_sita2;
		break;
		case 2:
		now=in->tar_sita3;
		break;		
	}
	
	out=now+tar_spd*dt;
	
	switch(joint)
	{
		case 0:
		in->tar_sita1=out;
		break;
		case 1:
		in->tar_sita2=out;
		break;
		case 2:
		in->tar_sita3=out;
		break;		
	}
}

void  Gait_Recovery_Active(void)
{
	int i=0;
    vmc_all.param.robot_mode=M_RECOVER;//����ʾ��
    vmc_all.gait_mode=RECOVER;
	vmc_all.fall_self=1;

	Gait_Recovery_Falling(0.005);
	
	for(i=0;i<4;i++){
	vmc[i].ground=0; 
	}

	for(i=0;i<5;i++)
		timer_self_reset[i]=0;
	self_right_state=0;
}

void  Gait_Recovery_Falling(float dt)
{
	char i=0;
	float side_left_safe[3]={0,0,0};
	float side_right_safe[3]={0,0,0};
	float k_att=LIMIT(fabs(vmc_all.att_ctrl[ROLr])/45,0,1)*k_roll_reset;

	if(vmc_all.att_ctrl[ROLr]<0)
	{
        side_left_safe[0]=side_free_safe_45[0];//
        side_left_safe[1]=side_free_safe_45[1];//
        side_left_safe[2]=side_free_safe_45[2]*k_att;//
		
        side_right_safe[0]=side_press_safe_45[0];//
        side_right_safe[1]=side_press_safe_45[1];//
        side_right_safe[2]=side_press_safe_45[2]*k_att;//
		
//		vmc[0].sita1=vmc[1].sita1=side_left_safe[0];
//		vmc[0].sita2=vmc[1].sita2=side_left_safe[1];
//		vmc[0].sita3=vmc[1].sita3=side_left_safe[2];
		move_joint_to_pos(&vmc[0],0,side_left_safe[0],160,dt);
		move_joint_to_pos(&vmc[0],1,side_left_safe[1],160,dt);
		move_joint_to_pos(&vmc[0],2,side_left_safe[2],160,dt);
		move_joint_to_pos(&vmc[1],0,side_left_safe[0],160,dt);
		move_joint_to_pos(&vmc[1],1,side_left_safe[1],160,dt);
		move_joint_to_pos(&vmc[1],2,side_left_safe[2],160,dt);		
		
		vmc[2].sita1=vmc[3].sita1=side_right_safe[0];
		vmc[2].sita2=vmc[3].sita2=side_right_safe[1];
		vmc[2].sita3=vmc[3].sita3=side_right_safe[2];
		
	}else
	{	
        side_right_safe[0]=side_free_safe_45[0];//
        side_right_safe[1]=side_free_safe_45[1];//
        side_right_safe[2]=side_free_safe_45[2]*k_att;//
		
        side_left_safe[0]=side_press_safe_45[0];//
        side_left_safe[1]=side_press_safe_45[1];//
        side_left_safe[2]=side_press_safe_45[2]*k_att;//
		
		
		vmc[0].sita1=vmc[1].sita1=side_left_safe[0];
		vmc[0].sita2=vmc[1].sita2=side_left_safe[1];
		vmc[0].sita3=vmc[1].sita3=side_left_safe[2];

//		vmc[2].sita1=vmc[3].sita1=side_right_safe[0];
//		vmc[2].sita2=vmc[3].sita2=side_right_safe[1];
//		vmc[2].sita3=vmc[3].sita3=side_right_safe[2];
		move_joint_to_pos(&vmc[2],0,side_right_safe[0],160,dt);
		move_joint_to_pos(&vmc[2],1,side_right_safe[1],160,dt);
		move_joint_to_pos(&vmc[2],2,side_right_safe[2],160,dt);
		move_joint_to_pos(&vmc[3],0,side_right_safe[0],160,dt);
		move_joint_to_pos(&vmc[3],1,side_right_safe[1],160,dt);
		move_joint_to_pos(&vmc[3],2,side_right_safe[2],160,dt);			
	}
	
	for(i=0;i<4;i++){
	vmc[i].ground=0; 
	}
}	

void  Gait_Recovery_Update(float dt)
{
	char i;
	char all_angle_reach[4]={0};
	float k_att=0;
	float side_left_safe[3]={0,0,0};
	float side_right_safe[3]={0,0,0};
	static int press_id[2]={0};
	static int free_id[2]={0};

	switch(self_right_state)
	{
		case 0:
				Gait_Recovery_Falling(dt);
				if(fabs(vmc_all.att_rate_ctrl[ROLr])<100&&vmc_all.gait_mode==RECOVER)
					timer_self_reset[0]+=dt;
				else
					timer_self_reset[0]=0;
				
				if(timer_self_reset[0]>1){
					timer_self_reset[0]=0;
					self_right_state++;
				}
		break;
		case 1:
				k_att=LIMIT(fabs(vmc_all.att[ROLr])/45,0,1)*k_roll_reset;
				if(vmc_all.att[ROLr]<0)
				{
				side_left_safe[0]=side_free_safe_45[0];//����
				side_left_safe[1]=side_free_safe_45[1];//С��
				side_left_safe[2]=side_free_safe_45[2]*k_att;//���

				side_right_safe[0]=side_press_safe_45[0];//����
				side_right_safe[1]=side_press_safe_45[1];//С��
				side_right_safe[2]=side_press_safe_45[2]*k_att;//���	
				}else
				{	
				side_right_safe[0]=side_free_safe_45[0];//����
				side_right_safe[1]=side_free_safe_45[1];//С��
				side_right_safe[2]=side_free_safe_45[2]*k_att;//���

				side_left_safe[0]=side_press_safe_45[0];//����
				side_left_safe[1]=side_press_safe_45[1];//С��
				side_left_safe[2]=side_press_safe_45[2]*k_att;//���	
				}

				vmc[0].sita1=vmc[1].sita1=side_left_safe[0];
				vmc[0].sita2=vmc[1].sita2=side_left_safe[1];
				vmc[0].sita3=vmc[1].sita3=side_left_safe[2];

				vmc[2].sita1=vmc[3].sita1=side_right_safe[0];
				vmc[2].sita2=vmc[3].sita2=side_right_safe[1];
				vmc[2].sita3=vmc[3].sita3=side_right_safe[2];	
				if(fabs(vmc_all.att_rate_ctrl[ROLr])<10)
					timer_self_reset[0]+=dt;
				else
					timer_self_reset[0]=0;
				
				if(timer_self_reset[0]>1){
					timer_self_reset[0]=0;
					if(vmc_all.att_ctrl[ROLr]>0)
					{
						press_id[0]=0;
						press_id[1]=1;
						free_id[0]=2;
						free_id[1]=3;
					}else{
						press_id[0]=2;
						press_id[1]=3;
						free_id[0]=0;
						free_id[1]=1;
					}
					self_right_state++;
				}
		break;
		case 2:
			all_angle_reach[0]=all_angle_reach[1]=1;
			for(i=0;i<2;i++){
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],0,side_press_reset_1[0],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],1,side_press_reset_1[1],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],2,side_press_reset_1[2],max_spd_reset,dt);
				
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],0,side_free_reset_1[0],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],1,side_free_reset_1[1],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],2,side_free_reset_1[2],max_spd_reset,dt);
			}
			if(all_angle_reach[0]&&all_angle_reach[1]==1)
				self_right_state++;
		break;
		case 3:
			all_angle_reach[0]=all_angle_reach[1]=1;
			for(i=0;i<2;i++){
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],0,side_press_reset_2[0],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],1,side_press_reset_2[1],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],2,side_press_reset_2[2],max_spd_reset,dt);
				
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],0,side_free_reset_2[0],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],1,side_free_reset_2[1],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],2,side_free_reset_2[2],max_spd_reset,dt);
			}
			if(all_angle_reach[0]&&all_angle_reach[1]==1)
				self_right_state++;
		break;
		case 4:
			all_angle_reach[0]=all_angle_reach[1]=1;
			for(i=0;i<2;i++){
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],0,side_press_reset_3[0],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],1,side_press_reset_3[1],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],2,side_press_reset_3[2],max_spd_reset,dt);
				
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],0,side_free_reset_3[0],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],1,side_free_reset_3[1],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],2,side_free_reset_3[2],max_spd_reset,dt);
			}
			if(all_angle_reach[0]&&all_angle_reach[1]==1&&1)
				self_right_state++;
		break;
		case 5:
			all_angle_reach[0]=all_angle_reach[1]=1;
			for(i=0;i<2;i++){
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],0,side_press_reset_4[0],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],1,side_press_reset_4[1],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],2,side_press_reset_4[2],max_spd_reset,dt);
				
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],0,side_free_reset_4[0],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],1,side_free_reset_4[1],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]],2,side_free_reset_4[2],max_spd_reset,dt);
			}
			if(all_angle_reach[0]&&all_angle_reach[1]==1){
				self_right_state++;
			}
		break;
		case 6://safe position
			all_angle_reach[0]=all_angle_reach[1]=1;
			for(i=0;i<2;i++){
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],0,vmc_all.param.safe_sita[0],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],1,vmc_all.param.safe_sita[1],max_spd_reset,dt);
			all_angle_reach[0]&=move_joint_to_pos(&vmc[press_id[i]],2,vmc_all.param.safe_sita[2],max_spd_reset,dt);
				
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]] ,0,vmc_all.param.safe_sita[0],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]] ,1,vmc_all.param.safe_sita[1],max_spd_reset,dt);
			all_angle_reach[1]&=move_joint_to_pos(&vmc[free_id[i]] ,2,vmc_all.param.safe_sita[2],max_spd_reset,dt);
			}
			if(all_angle_reach[0]&&all_angle_reach[1]==1&&fabs(vmc_all.att_ctrl[ROLr])<10){
				vmc_all.fall_self=0;		
				self_right_state++;
				
				#if EN_AUTO_STAND_SELFRIGHT
				ocu.cmd_robot_state=2;
				vmc_all.param.robot_mode=M_STAND_RC;	
				vmc_all.gait_mode=STAND_RC;	
				vmc_all.param.stand_switch_flag[0]=vmc_all.param.stand_switch_flag[1]=1;//�ϵ������
				vmc_all.param.stand_switch_cnt[0]=vmc_all.param.stand_switch_cnt[1]=0;
				vmc_all.param.stand_trot_switch_flag=0;
				Gait_Stand_Active();//����վ����̬
				#endif
			}
		break;				
	}

}	
