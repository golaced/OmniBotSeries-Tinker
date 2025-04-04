#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "can.h"
#include "spi_node.h"
#include "eso.h"
#include "include.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static float k_att_test[3] = { -60,0.5,1 };
static char en_yaw_test = 0;
static float off_y = 0.015;
static int en_att_mask = 1;
static float FLT_PIT_TEST = 10;

END_POS end_pos_stand_h[4], end_pos_stand_n[4];
char st_mode_reg = 0;

float stand_gcheck_time[2] = { 0.05,0.1 };//TD  LF 着地时间判断与离地时间
END_POS vmc_all_pos_n_air_st;
END_POS vmc_all_spd_n_air_st;

void  Gait_Stand_Active(void)
{
    int i = 0;
    vmc_all.param.robot_mode=M_STAND_RC;vmc_all.gait_mode=STAND_RC;

    for(i=0;i<14;i++){
        leg_motor_all.q_set[i]=leg_motor_all.q_init[i];//doghome
        leg_motor_all.stiff[i]=leg_motor_all.stiff_stand;
    }

}


void  Gait_Stand_Update(float dt)
{
    static char init[4], init_lisence = 0, state, cnt_time_change, switch_flag;
    static float time_delay, yaw_force;
    static float timer_task[10];
    char i, sel;
    float test_epos_b[3], test_epos_leg[3];
    static END_POS end_pos_n[4], end_pos_n_use[4];//ĩ�˹̶���ȫ������ϵ�µ�λ��
    static float cnt_mode_can_switch = 0, mode_en_switch_flag = 0;
    static float att_test_use[3];
    static char mode_reg;
    float att_off[2];
#if 0
    //state estimateor
    static float ground_timer[4][2] = { 0 };
    vmc_all.ground_num = 0;
    for (i = 0; i < 4; i++) {

        vmc[i].ground = vmc[i].is_touch;

        if (vmc[i].ground)
            vmc_all.ground_num++;

        if (st_mode_reg != vmc_all.gait_mode) {//模式切换下赋为当前值
            end_pos_stand_h[i] = vmc[i].epos;
            end_pos_stand_n[i] = vmc[i].epos_n;
            vmc_all.param.test_pos_flt[Zr] = -vmc_all.pos_n.z;
            vmc_all.param.test_pos_flt[Xr] = vmc_all.param.test_pos_flt[Yr] = 0;
        }
    }
    //printf("%d %d %d %d\n",vmc[0].ground,vmc[1].ground,vmc[2].ground,vmc[3].ground);
    vmc_all_pos_n_air_st.x = (vmc[0].epos_n.x + vmc[1].epos_n.x + vmc[2].epos_n.x + vmc[3].epos_n.x) / 4;
    vmc_all_pos_n_air_st.y = (vmc[0].epos_n.y + vmc[1].epos_n.y + vmc[2].epos_n.y + vmc[3].epos_n.y) / 4;
    vmc_all_pos_n_air_st.z = -(vmc[0].epos_n.z + vmc[1].epos_n.z + vmc[2].epos_n.z + vmc[3].epos_n.z) / 4;
    vmc_all_spd_n_air_st.z = (vmc[0].spd_n.z + vmc[1].spd_n.z + vmc[2].spd_n.z + vmc[3].spd_n.z) / 4;

    st_mode_reg = vmc_all.gait_mode;

    static float en_force_timer[2] = { 0 };

#if STAND_GROUND_CHECK_TEST&&!MESS_FIX_TEST&&1//力控标志位赋值
    if (vmc_all.ground_num >= 3 && !stand_force_enable_flag[4] && sit_down_flag == 1)//着地
        en_force_timer[0] += dt;
    else
        en_force_timer[0] = 0;

    if (vmc_all.ground_num <= 1 && stand_force_enable_flag[4])
        en_force_timer[1] += dt;
    else
        en_force_timer[1] = 0;
    //printf("vmc_all.ground_num=%d sit_down_flag=%d\n",vmc_all.ground_num,sit_down_flag);
    //------------------------------------------------------------------------------
    if (en_force_timer[0] > stand_gcheck_time[0] - dt && !stand_force_enable_flag[4] && sit_down_flag == 1&&!EN_SWING_TEST) {//着地切换力控
        en_force_timer[0] = 0;
        reset_servo_interge();//清除积分
        for (i = 0; i < 4; i++) {//清除力复位力控期望
            robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.y =  robotwb.Leg[i].tar_force_dis_n.z = 0;
            reset_tar_pos(i);
            vmc[i].tar_sita1 = vmc[i].sita1;
            vmc[i].tar_sita2 = vmc[i].sita2;
            vmc[i].tar_sita3 = vmc[i].sita3;
        }
        printf("Stand::In Ground Mode!!\n");

        robotwb.exp_pos_n.x = vmc_all.tar_pos.x = 0;//vmc_all.pos_n.x;//复位力控期望
        robotwb.exp_pos_n.y = vmc_all.tar_pos.y = 0;//vmc_all.pos_n.y;//复位力控期望
        robotwb.exp_pos_n.z = fabs(vmc_all_pos_n_air_st.z);
        vmc_all.param.test_pos_flt[Zr] = -fabs(vmc_all_pos_n_air_st.z);

        stand_force_enable_flag[4] = 1;
        stand_force_enable_flag[3] = stand_force_enable_flag[2] = stand_force_enable_flag[1] = stand_force_enable_flag[0] = 1;
    }
    else if (en_force_timer[1] > stand_gcheck_time[1] - dt && stand_force_enable_flag[4] == 1) {//离地AIR模式
        en_force_timer[1] = 0;
        reset_servo_interge();//清除积分
        for (i = 0; i < 4; i++) {//清除力复位力控期望
            robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.y = robotwb.Leg[i].tar_force_dis_n.z = 0;
            reset_tar_pos(i);

            vmc[i].tar_sita1 = vmc[i].sita1;
            vmc[i].tar_sita2 = vmc[i].sita2;
            vmc[i].tar_sita3 = vmc[i].sita3;
        }
        printf("Stand::In Air Mode!!\n");
        robotwb.exp_pos_n.x = vmc_all.tar_pos.x = 0;//vmc_all.pos_n.x;//复位力控期望
        robotwb.exp_pos_n.y = vmc_all.tar_pos.y = 0;//vmc_all.pos_n.y;//复位力控期望
        robotwb.exp_pos_n.z = fabs(vmc_all_pos_n_air_st.z);
        vmc_all.param.test_pos_flt[Zr] = -fabs(vmc_all_pos_n_air_st.z);//足端位置

        stand_force_enable_flag[4] = 0;
        stand_force_enable_flag[3] = stand_force_enable_flag[2] = stand_force_enable_flag[1] = stand_force_enable_flag[0] = 0;
    }
#endif

    if (stand_force_enable_flag[4] == 0)//空中位置模式式
    {
        for (i = 0; i < 4; i++) {//期望末端位置
            end_pos_n[i].x = end_pos_stand_n[i].x + vmc_all.param.param_vmc.posz_idle[Xr];
            end_pos_n[i].y = end_pos_stand_n[i].y + vmc_all.param.param_vmc.posz_idle[Yr];
            end_pos_n[i].z = end_pos_stand_n[i].z;
        }

        DigitalLPF(-ocu.rc_att_w[PITr] * vmc_all.param.MAX_PIT, &vmc_all.param.test_att_flt[PITr], 0.85, dt);
        DigitalLPF(-ocu.rc_att_w[ROLr] * vmc_all.param.MAX_ROL, &vmc_all.param.test_att_flt[ROLr], 0.85, dt);
        DigitalLPF(ocu.rate_yaw_w*vmc_all.param.MAX_YAW, &vmc_all.param.test_att_flt[YAWr], 0.85, dt);

        //位置
        vmc_all.param.test_pos_flt[Xr] += ocu.rc_spd_w[Xr] * 0.06*dt;
        vmc_all.param.test_pos_flt[Xr] = LIMIT(vmc_all.param.test_pos_flt[Xr], 0.35*MIN_X, 0.35*MAX_X);

        vmc_all.param.test_pos_flt[Yr] += ocu.rc_spd_w[Yr] * 0.06*dt;
        vmc_all.param.test_pos_flt[Yr] = LIMIT(vmc_all.param.test_pos_flt[Yr], 0.5*MIN_Y, 0.5*MAX_Y);

        DigitalLPF(-fabs(vmc_all.tar_pos.z), &vmc_all.param.test_pos_flt[Zr], 0.7, dt);
        vmc_all.param.test_pos_flt[Zr] = LIMIT(vmc_all.param.test_pos_flt[Zr], 0.9*MAX_Z, 1.1*MIN_Z);

        if (sit_down_flag == 0) {
            DigitalLPF(LIMIT(vmc_all.att[ROLr] + vmc_all.tar_att_bias[ROLr], -66, 66), &att_test_use[ROLr], 10, dt);//
            DigitalLPF(LIMIT(-vmc_all.att[PITr] + vmc_all.tar_att_bias[PITr], -30, 30), &att_test_use[PITr], 10, dt);//
        }
        else {
            DigitalLPF(LIMIT(-vmc_all.att[ROLr] + vmc_all.tar_att_bias[ROLr], -66, 66), &att_test_use[ROLr], 10, dt);//
            DigitalLPF(LIMIT(-vmc_all.att[PITr] + vmc_all.tar_att_bias[PITr], -66, 66), &att_test_use[PITr], 10, dt);//
        }
        DigitalLPF(LIMIT(vmc_all.tar_spd.z*k_att_test[YAWr], -45, 45) * !en_yaw_test, &att_test_use[YAWr], 10, dt);

        float RTb_n_target[3][3];//
        RTb_n_target[0][0] = cosd(-att_test_use[PITr]);  RTb_n_target[0][1] = sind(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[0][2] = sind(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);
        RTb_n_target[1][0] = 0;												   RTb_n_target[1][1] = cosd(-att_test_use[ROLr]);													 	RTb_n_target[1][2] = -sind(-att_test_use[ROLr]);
        RTb_n_target[2][0] = -sind(-att_test_use[PITr]); RTb_n_target[2][1] = cosd(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[2][2] = cosd(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);

        END_POS angke_n[4];//
        for (i = 0; i < 4; i++)
            converV_b_to_n_RT(RTb_n_target, att_test_use[YAWr], vmc[i].flag_fb*vmc_all.H / 2, vmc[i].flag_rl*vmc_all.W / 2, 0, &angke_n[i].x, &angke_n[i].y, &angke_n[i].z);
        //----
        END_POS leg_2_angke_n[4];//
        for (i = 0; i < 4; i++) {
            leg_2_angke_n[i].x = end_pos_n[i].x + vmc_all.param.test_pos_flt[Xr] - angke_n[i].x + vmc_all.param.param_vmc.side_off[Xr] * vmc[i].flag_fb * 0;
            leg_2_angke_n[i].y = end_pos_n[i].y + vmc_all.param.test_pos_flt[Yr] - angke_n[i].y + vmc_all.param.param_vmc.side_off[Yr] * vmc[i].flag_rl * 0;
            leg_2_angke_n[i].z = vmc_all.param.test_pos_flt[Zr] - angke_n[i].z;
        }
        //----
        END_POS leg_2_angke_b[4];
        for (i = 0; i < 4; i++) {
            converV_n_to_b_w_yaw(-att_test_use[YAWr], leg_2_angke_n[i].x, leg_2_angke_n[i].y, leg_2_angke_n[i].z,
                &leg_2_angke_b[i].x, &leg_2_angke_b[i].y, &leg_2_angke_b[i].z);
            leg_2_angke_b[i].x += vmc[i].flag_fb*vmc_all.H / 2;
            leg_2_angke_b[i].y += vmc[i].flag_rl*vmc_all.W / 2;
        }
        //----
        END_POS leg_2_angke_leg[4];
        for (i = 0; i < 4; i++)
            converV_b_to_leg(i, leg_2_angke_b[i].x, leg_2_angke_b[i].y, leg_2_angke_b[i].z,
                &leg_2_angke_leg[i].x, &leg_2_angke_leg[i].y, &leg_2_angke_leg[i].z);
#if !EN_END_SPD_MODE//我的VMC 直接输出期望角度到底层
        for (i = 0; i < 4; i++)
            inv_end_state_new(&vmc[i], leg_2_angke_leg[i].x, leg_2_angke_leg[i].y, leg_2_angke_leg[i].z,
                &vmc[i].tar_sita1, &vmc[i].tar_sita2, &vmc[i].tar_sita3);
#else
        for (i = 0; i < 4; i++) {
            vmc[i].tar_epos_h = leg_2_angke_leg[i];//输出给阻抗控制器 进一步平滑滤波
            //vmc[i].tar_epos_h.x=LIMIT(vmc[i].tar_epos_h.x,MIN_X,MAX_X);//期望设定位置限制幅度
            vmc[i].tar_epos_h.z = LIMIT(vmc[i].tar_epos_h.z, MAX_Z, MIN_Z);
        }
#endif
    }
#endif
    //out put for PD stand
    for(i=0;i<14;i++){
        leg_motor_all.q_set[i]=move_joint_to_pos_all(leg_motor_all.q_set[i],leg_motor_all.q_init[i],180,dt);//doghome
        //printf("id=%d set=%f init=%f\n",i,leg_motor_all.q_set[i],leg_motor_all.q_init[i]);
    }

    for (i = 0; i < 10; i++)
        timer_task[i] += dt;
}
