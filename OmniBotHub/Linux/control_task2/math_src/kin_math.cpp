#include "include.h"
#include "locomotion_header.h"
#include "math.h"
#include "eso.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif
//------------------------------运动学运算库
float FLT_SPD_END = 200;

void espd_to_neg_dq(int id, float dt)
{
    float flt_espd = 1;
    float gain_espd = 1;
    float temp[3] = { 0,0,0 };
    if(vmc_all.param.leg_dof==3){
        temp[0] = (-robotwb.Leg[id].jacobi_inv[0] * robotwb.Leg[id].tar_espd_h.x*vmc[id].param.invert_knee_epos[Xr]
            - robotwb.Leg[id].jacobi_inv[1] * robotwb.Leg[id].tar_espd_h.y//*robotwb.Leg[id].flag_rl
            + (-robotwb.Leg[id].jacobi_inv[2] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;
        temp[1] = (-robotwb.Leg[id].jacobi_inv[3] * robotwb.Leg[id].tar_espd_h.x*vmc[id].param.invert_knee_epos[Xr]
            - robotwb.Leg[id].jacobi_inv[4] * robotwb.Leg[id].tar_espd_h.y//*robotwb.Leg[id].flag_rl
            + (-robotwb.Leg[id].jacobi_inv[5] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;
        temp[2] = (-robotwb.Leg[id].jacobi_inv[6] * robotwb.Leg[id].tar_espd_h.x*vmc[id].param.invert_knee_epos[Xr]
            - robotwb.Leg[id].jacobi_inv[7] * robotwb.Leg[id].tar_espd_h.y//*robotwb.Leg[id].flag_rl
            + (-robotwb.Leg[id].jacobi_inv[8] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;

        robotwb.Leg[id].tar_sita_d[0] = temp[0];// * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[0];
        robotwb.Leg[id].tar_sita_d[1] = temp[1];// * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[1];
        robotwb.Leg[id].tar_sita_d[2] = temp[2];// * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[2];
    }
    else{
        temp[0] = (-robotwb.Leg[id].jacobi_inv[0] * robotwb.Leg[id].tar_espd_h.x + (-robotwb.Leg[id].jacobi_inv[1] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;
        temp[1] = (-robotwb.Leg[id].jacobi_inv[2] * robotwb.Leg[id].tar_espd_h.x + (-robotwb.Leg[id].jacobi_inv[3] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;

        robotwb.Leg[id].tar_sita_d[0] = temp[0] * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[0];
        robotwb.Leg[id].tar_sita_d[1] = temp[1] * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[1];
    }
}

void force_to_tao(int id, float dt)
{
    float flt_t = 1;//滤波系数
    float gain_t = -1;//输出符号
    float temp[3] = { 0,0,0 };
    if(vmc_all.param.leg_dof==3){
        temp[0] = (robotwb.Leg[id].jacobi[0] * robotwb.Leg[id].tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
            + (robotwb.Leg[id].jacobi[3] * robotwb.Leg[id].tar_force_h.y)
            + (robotwb.Leg[id].jacobi[6] * robotwb.Leg[id].tar_force_h.z)
            )*gain_t;
        temp[1] = (robotwb.Leg[id].jacobi[1] * robotwb.Leg[id].tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
            + (robotwb.Leg[id].jacobi[4] * robotwb.Leg[id].tar_force_h.y)
            + (robotwb.Leg[id].jacobi[7] * robotwb.Leg[id].tar_force_h.z)
            )*gain_t;
        temp[2] = (robotwb.Leg[id].jacobi[2] * robotwb.Leg[id].tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
            + (robotwb.Leg[id].jacobi[5] * robotwb.Leg[id].tar_force_h.y)
            + (robotwb.Leg[id].jacobi[8] * robotwb.Leg[id].tar_force_h.z)
            )*gain_t;
        robotwb.Leg[id].taod[0] = temp[0];// * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[0];
        robotwb.Leg[id].taod[1] = temp[1];// * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[1];
        robotwb.Leg[id].taod[2] = temp[2];// * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[2];

        robotwb.Leg[id].taod[0] = limitw(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
        robotwb.Leg[id].taod[1] = limitw(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
        robotwb.Leg[id].taod[2] = limitw(robotwb.Leg[id].taod[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
    }else{
        temp[0] = (robotwb.Leg[id].jacobi[0] * robotwb.Leg[id].tar_force_h.x + robotwb.Leg[id].jacobi[2] * robotwb.Leg[id].tar_force_h.z)*gain_t;
        temp[1] = (robotwb.Leg[id].jacobi[1] * robotwb.Leg[id].tar_force_h.x + robotwb.Leg[id].jacobi[3] * robotwb.Leg[id].tar_force_h.z)*gain_t;
        //力梯度Jerk
        robotwb.Leg[id].taod[0] = temp[0] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[0];
        robotwb.Leg[id].taod[1] = temp[1] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[1];

        robotwb.Leg[id].taod[0] = LIMIT(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
        robotwb.Leg[id].taod[1] = LIMIT(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
    }
}


void force_to_tao_kin1(Vect3 tar_force_h,int id, float dt)
{
    float flt_t = 1;//滤波系数
    float gain_t = -1;//输出符号
    float temp[3] = { 0,0,0 };

    temp[0] = (robotwb.Leg[id].jacobi_kin1[0] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin1[3] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin1[6] * tar_force_h.z)
        )*gain_t;
    temp[1] = (robotwb.Leg[id].jacobi_kin1[1] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin1[4] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin1[7] * tar_force_h.z)
        )*gain_t;
    temp[2] = (robotwb.Leg[id].jacobi_kin1[2] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin1[5] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin1[8] * tar_force_h.z)
        )*gain_t;
    robotwb.Leg[id].taod_kin1[0] = limitw(temp[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
    robotwb.Leg[id].taod_kin1[1] = limitw(temp[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
    robotwb.Leg[id].taod_kin1[2] = limitw(temp[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
}



void force_to_tao_kin12(Vect3 tar_force_h,int id, float dt)
{
    float flt_t = 1;//滤波系数
    float gain_t = -1;//输出符号
    float temp[3] = { 0,0,0 };

    temp[0] = (robotwb.Leg[id].jacobi_kin12[0] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin12[3] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin12[6] * tar_force_h.z)
        )*gain_t;
    temp[1] = (robotwb.Leg[id].jacobi_kin12[1] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin12[4] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin12[7] * tar_force_h.z)
        )*gain_t;
    temp[2] = (robotwb.Leg[id].jacobi_kin12[2] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin12[5] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin12[8] * tar_force_h.z)
        )*gain_t;
    robotwb.Leg[id].taod_kin12[0] = limitw(temp[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
    robotwb.Leg[id].taod_kin12[1] = limitw(temp[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
    robotwb.Leg[id].taod_kin12[2] = limitw(temp[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
}


void force_to_tao_kin2(Vect3 tar_force_h,int id, float dt)
{
    float flt_t = 1;//滤波系数
    float gain_t = -1;//输出符号
    float temp[3] = { 0,0,0 };

    temp[0] = (robotwb.Leg[id].jacobi_kin2[0] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin2[3] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin2[6] * tar_force_h.z)
        )*gain_t;
    temp[1] = (robotwb.Leg[id].jacobi_kin2[1] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin2[4] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin2[7] * tar_force_h.z)
        )*gain_t;
    temp[2] = (robotwb.Leg[id].jacobi_kin2[2] * tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
        + (robotwb.Leg[id].jacobi_kin2[5] * tar_force_h.y)
        + (robotwb.Leg[id].jacobi_kin2[8] * tar_force_h.z)
        )*gain_t;
    robotwb.Leg[id].taod_kin2[0] = limitw(temp[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
    robotwb.Leg[id].taod_kin2[1] = limitw(temp[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
    robotwb.Leg[id].taod_kin2[2] = limitw(temp[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
}


void force_to_tao_input(int id, Vect3 force_h, float dt)
{
    float flt_t = 1;
    float gain_t = -1;
    float temp[3] = { 0,0,0 };
    if(vmc_all.param.leg_dof==3){
        temp[0] = (robotwb.Leg[id].jacobi[0] * force_h.x*vmc[id].param.invert_knee_epos[Xr]
            + (robotwb.Leg[id].jacobi[3] * force_h.y)
            + (robotwb.Leg[id].jacobi[6] * force_h.z)
            )*gain_t;
        temp[1] = (robotwb.Leg[id].jacobi[1] * force_h.x*vmc[id].param.invert_knee_epos[Xr]
            + (robotwb.Leg[id].jacobi[4] * force_h.y)
            + (robotwb.Leg[id].jacobi[7] * force_h.z)
            )*gain_t;
        temp[2] = (robotwb.Leg[id].jacobi[2] * force_h.x*vmc[id].param.invert_knee_epos[Xr]
            + (robotwb.Leg[id].jacobi[5] * force_h.y)
            + (robotwb.Leg[id].jacobi[8] * force_h.z)
            )*gain_t;
        robotwb.Leg[id].taod[0] = temp[0];// * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[0];
        robotwb.Leg[id].taod[1] = temp[1];// * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[1];
        robotwb.Leg[id].taod[2] = temp[2];// * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[2];

        robotwb.Leg[id].taod[0] = limitw(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
        robotwb.Leg[id].taod[1] = limitw(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
        robotwb.Leg[id].taod[2] = limitw(robotwb.Leg[id].taod[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
    }else{
        temp[0] = (robotwb.Leg[id].jacobi[0] * force_h.x + (robotwb.Leg[id].jacobi[2] * force_h.z))*gain_t;
        temp[1] = (robotwb.Leg[id].jacobi[1] * force_h.x + (robotwb.Leg[id].jacobi[3] * force_h.z))*gain_t;

        robotwb.Leg[id].taod[0] = temp[0] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[0];
        robotwb.Leg[id].taod[1] = temp[1] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[1];

        robotwb.Leg[id].taod[0] = LIMIT(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
        robotwb.Leg[id].taod[1] = LIMIT(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
    }
}

void inv_KI(int id, Vect3 epos_h, float *s0, float *s1,float *s2)
{
    float end_pos[3];
    float temp[6];
    float x = epos_h.x;
    float z = epos_h.z;
    if(vmc_all.param.leg_dof==3){
        end_pos[Xrw] = vmc[id].param.invert_knee_epos[Xr]*epos_h.x;
        end_pos[Yrw] = robotwb.Leg[id].flag_rl* epos_h.y;
        end_pos[Zrw] = epos_h.z;
        float Rr = end_pos[Xrw] * end_pos[Xrw] + end_pos[Yrw] * end_pos[Yrw] + end_pos[Zrw] * end_pos[Zrw];
        float Rr1 = end_pos[Yrw] * end_pos[Yrw] + end_pos[Zrw] * end_pos[Zrw];
        temp[2] = sqrtf(limitw(-L3w * L3w + Rr1, 0, 99));
        *s2 = (90 - atan2(fabs(end_pos[Zrw]), end_pos[Yrw] + 0.000001)*RAD_TO_DEGw -
            atan2(L3w, temp[2] + 0.000001)*RAD_TO_DEGw)*robotwb.Leg[id].flag_rl;
        temp[0] = sqrtf(limitw(Rr1 - L3w * L3w, 0, 99));//h1
        temp[1] = sqrtf(limitw(temp[0] * temp[0] + end_pos[Xrw] * end_pos[Xrw], 0, 99));//r
        temp[2] = atan2(end_pos[Xrw], temp[0] + 0.000001)*RAD_TO_DEGw;//sita
        temp[3] = (temp[1] * temp[1] + L1w * L1w - L2w * L2w) / (2 * L1w*temp[1] + 0.000001);
        temp[3] = limitw(temp[3], -1, 1);
        *s0 = 180 - (180 + temp[2] - acos(temp[3]) *RAD_TO_DEGw);

        temp[0] = (L1w*L1w + L2w * L2w + L3w * L3w - Rr) / (2 * L1w*L2w);
        temp[0] = limitw(temp[0], -1, 1);
        *s1 = acos(temp[0])*RAD_TO_DEGw;
    }
}

//正运动学
float k_end_v = 1;
char estimate_end_state_new(VMC *in, float dt)//??????
{
    int id=in->param.id;
    float alfa = (in->sita2 - in->sita1) / 2;
    float beta = (in->sita2 + in->sita1) / 2;
    float temp;
    float H = 0;
    float L1, L2, L3, L4, L5, L6;
    float A1, B1, C1, lengthAC;
    float Xa, Ya, Xb, Yb, Xc, Yc, Xe, Ye;
    float theta1, theta2;
    float Param[6] = { 0 };
    float S2 = -in->sita2;
    float S1 = -in->sita1;
    switch (vmc_all.param.leg_dof) {
    case 3://MIT

        float H = in->l1 * cosdw(in->sita1) + in->l2 * cosdw(180 - in->sita2- in->sita1);
        in->epos.x = -in->l1 * sindw(in->sita1) + in->l2 * sindw(180 - in->sita2 - in->sita1);
        in->epos.x *= in->param.invert_knee_epos[Xr];

        in->epos.y = in->l3 * cosdw(in->sita3 * in->flag_rl)
            + H * sindw(in->sita3 * in->flag_rl);
        in->epos.y *= in->flag_rl;

        in->epos.z = -H * cosdw(in->sita3 * in->flag_rl)
            + in->l3 * sindw(in->sita3 * in->flag_rl);

        in->r = sqrtf(in->epos.x*in->epos.x
            + in->epos.y*in->epos.y
            + in->epos.z*in->epos.z);
        break;
    }

    in->epos_b.x = in->epos.x + in->flag_fb* vmc_all.H / 2;
    in->epos_b.y = in->epos.y + in->flag_rl* vmc_all.W / 2;
    in->epos_b.z = in->epos.z;


    in->epos_n.x = vmc_all.Rb_n[0][0] * in->epos_b.x + vmc_all.Rb_n[0][1] * in->epos_b.y + vmc_all.Rb_n[0][2] * in->epos_b.z;
    in->epos_n.y = vmc_all.Rb_n[1][0] * in->epos_b.x + vmc_all.Rb_n[1][1] * in->epos_b.y + vmc_all.Rb_n[1][2] * in->epos_b.z;
    in->epos_n.z = vmc_all.Rb_n[2][0] * in->epos_b.x + vmc_all.Rb_n[2][1] * in->epos_b.y + vmc_all.Rb_n[2][2] * in->epos_b.z;

    in->epos_nnn.x = vmc_all.Rb_nn[0][0] * in->epos_b.x + vmc_all.Rb_nn[0][1] * in->epos_b.y + vmc_all.Rb_nn[0][2] * in->epos_b.z;
    in->epos_nnn.y = vmc_all.Rb_nn[1][0] * in->epos_b.x + vmc_all.Rb_nn[1][1] * in->epos_b.y + vmc_all.Rb_nn[1][2] * in->epos_b.z;
    in->epos_nnn.z = vmc_all.Rb_nn[2][0] * in->epos_b.x + vmc_all.Rb_nn[2][1] * in->epos_b.y + vmc_all.Rb_nn[2][2] * in->epos_b.z;

    in->epos_n_b.x = vmc_all.Rb_n[0][0] * in->epos.x + vmc_all.Rb_n[0][1] * in->epos.y + vmc_all.Rb_n[0][2] * in->epos.z;
    in->epos_n_b.y = vmc_all.Rb_n[1][0] * in->epos.x + vmc_all.Rb_n[1][1] * in->epos.y + vmc_all.Rb_n[1][2] * in->epos.z;
    in->epos_n_b.z = vmc_all.Rb_n[2][0] * in->epos.x + vmc_all.Rb_n[2][1] * in->epos.y + vmc_all.Rb_n[2][2] * in->epos.z;
    //if(id==1)
    //  printf("hip id=%d q0=%f q1=%f q2=%f ex=%f ey=%f ez=%f %d,%f %f %f %d\n", id,in->sita1,in->sita2,in->sita3, in->epos.x, in->epos.y, in->epos.z,vmc[id].param.invert_knee_epos[Xr],in->l1,in->l2,in->l3,vmc_all.param.leg_dof);
   // printf("body id=%d ex=%f ey=%f ez=%f\n", in->param.id, in->epos_b.x, in->epos_b.y, in->epos_b.z);
    //printf("n id=%d ex=%f ey=%f ez=%f\n", in->param.id, in->epos_n.x, in->epos_n.y, in->epos_n.z);
    float d_sita = 0;
    float d_alfa = 0;
    float d_sita1, d_sita2, d_sita3;
    float d_x, d_y, d_z;
    float dr_dt;
    in->param.spd_est_cnt += dt;
    //--------------------------末端速度-------------------------
    Vect3 spd_bb;
    Vect3 q_spd;
    if(vmc_all.param.leg_dof==3){
        float jacobi3[3][3] = { 0 };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                jacobi3[i][j] = in->jacobi33[i * 3 + j];
        q_spd.x = in->dsita1 / 57.3;
        q_spd.y = in->dsita2 / 57.3;
        q_spd.z = in->dsita3 / 57.3;//角速度有值 会后退在支撑
        matrx33_mult_vect3(jacobi3, q_spd, &spd_bb);//doghome
        spd_bb.x *= in->param.invert_knee_epos[Xr];
    }else{
        float jacobi2[2][2] = { 0 };
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                jacobi2[i][j] = in->jacobi22[i * 2 + j];
        q_spd.x = in->dsita1 / 57.3;
        q_spd.y = 0;
        q_spd.z = in->dsita2 / 57.3;//角速度有值 会后退在支撑
        matrx33_mult_vect2(jacobi2, q_spd, &spd_bb);//doghome
    }

#if 0
    if (in->param.spd_est_cnt > vmc_all.param.end_sample_dt) {
        d_x = (in->epos.x - in->epos_reg.x) / in->param.spd_est_cnt;
        d_y = (in->epos.y - in->epos_reg.y) / in->param.spd_est_cnt;
        d_z = (in->epos.z - in->epos_reg.z) / in->param.spd_est_cnt;

        if (in->ground || 1) {
            in->spd_o.x = -LIMIT(d_x, -5, 5);
            in->spd_o.y = -LIMIT(d_y, -5, 5);
            in->spd_o.z = LIMIT(d_z, -5, 5);
        }
        in->epos_reg.x = in->epos.x;
        in->epos_reg.y = in->epos.y;
        in->epos_reg.z = in->epos.z;
        in->param.spd_est_cnt = 0;
    }
#else
    in->spd_o.x = -LIMIT(spd_bb.x, -1, 1);
    in->spd_o.y = -LIMIT(spd_bb.y, -1, 1);
    in->spd_o.z =  LIMIT(spd_bb.z, -1, 1);
#endif
    DigitalLPF(in->spd_o.x, &in->spd.x, FLT_SPD_END, dt);
    DigitalLPF(in->spd_o.y, &in->spd.y, FLT_SPD_END, dt);
    DigitalLPF(in->spd_o.z, &in->spd.z, FLT_SPD_END, dt);
   // printf("body id=%d vx=%f vy=%f vz=%f\n", in->param.id, in->spd.x, in->spd.y, in->spd.z);
    in->spd_b.x = in->spd.x;
    in->spd_b.y = in->spd.y;
    in->spd_b.z = in->spd.z;

    END_POS spd_n = in->spd_o;
    spd_n.x = vmc_all.Rb_n[0][0] * in->spd_o.x + vmc_all.Rb_n[0][1] * in->spd_o.y + vmc_all.Rb_n[0][2] * in->spd_o.z;
    spd_n.y = vmc_all.Rb_n[1][0] * in->spd_o.x + vmc_all.Rb_n[1][1] * in->spd_o.y + vmc_all.Rb_n[1][2] * in->spd_o.z;
    spd_n.z = vmc_all.Rb_n[2][0] * in->spd_o.x + vmc_all.Rb_n[2][1] * in->spd_o.y + vmc_all.Rb_n[2][2] * in->spd_o.z;

#if EN_ROTATE_END_COMPASS_EST&&1//旋转速度补偿
    Vect3 Hip_b;
    Vect3 com;
    com.x = com.y = com.z = 0;
    Hip_b.x = com.x - in->epos_b.x;
    Hip_b.y = com.y - in->epos_b.y;
    Hip_b.z = com.z - in->epos_b.z;

    Vect3 w_b;
    w_b.x = vmc_all.att_rate_trig[ROLr] * 0.0173 * (vmc_all.param.leg_dof==3);
    w_b.y = vmc_all.att_rate_trig[PITr] * 0.0173 * 1;
    w_b.z = vmc_all.att_rate_trig[YAWr] * 0.0173 * 0;
    float w_cross_b[3][3] = { 0 };
    if (vmc_all.gait_mode == STAND_RC){
        vect3_2_cross(w_b, w_cross_b);
        Vect3 hip_rotate_spd_b, hip_rotate_spd_n;

        matrx33_mult_vect3(w_cross_b, Hip_b, &hip_rotate_spd_b);
        converV_b_to_n(hip_rotate_spd_b.x, hip_rotate_spd_b.y, hip_rotate_spd_b.z,
            &hip_rotate_spd_n.x, &hip_rotate_spd_n.y, &hip_rotate_spd_n.z);//转换速度直接输出到IMP  控制频率不够目前？？

        spd_n.x -= hip_rotate_spd_n.x;
        spd_n.y -= hip_rotate_spd_n.y;
        spd_n.z -= hip_rotate_spd_n.z;
    }
#endif
    DigitalLPF(spd_n.x, &in->spd_n.x, FLT_SPD_END, dt);
    DigitalLPF(spd_n.y, &in->spd_n.y, FLT_SPD_END, dt);
    DigitalLPF(spd_n.z, &in->spd_n.z, FLT_SPD_END, dt);

    //胯关节速度
    Vect3 v_hip_R;
    v_hip_R.x = vmc_all.H / 2 * in->flag_fb;
    v_hip_R.y = vmc_all.W / 2 * in->flag_rl;
    v_hip_R.z = 0;
    Vect3 p_hip_R;
    converV_b_to_nw(v_hip_R, &p_hip_R);
    Vect3 ws;
    Vect3 w_bb;
    float w_cross_bs[3][3] = { 0 };

    w_bb.x = vmc_all.att_rate_swing[ROLr] * 0.0173 * 1;
    w_bb.y = vmc_all.att_rate_swing[PITr] * 0.0173 * 1;//PIT -1 Trot good   1 bound good
    w_bb.z = vmc_all.att_rate_swing[YAWr] * 0.0173 * 1;

    converV_b_to_nw(w_bb, &ws);
    vect3_2_cross(ws, w_cross_bs);//转换成 反对称矩阵

    Vect3 hip_rotate_spd_bs;
    matrx33_mult_vect3(w_cross_bs, p_hip_R, &hip_rotate_spd_bs);
    in->spd_hip_n.x = vmc_all.spd_n.x + hip_rotate_spd_bs.x;
    in->spd_hip_n.y = vmc_all.spd_n.y + hip_rotate_spd_bs.y;
    in->spd_hip_n.z = vmc_all.spd_n.z + hip_rotate_spd_bs.z;
    if (in->param.id == 0 && 0)
        printf("%f %f %f\n", hip_rotate_spd_bs.x, hip_rotate_spd_bs.y, hip_rotate_spd_bs.z);

    return 0;
}


char inv_end_state_new(VMC *vmc, float x, float y, float z, float *sita1, float *sita2, float *sita3)
{
    int i = vmc->param.id;
    robotwb.Leg[i].tar_epos_h.x = x;
    robotwb.Leg[i].tar_epos_h.y = y;
    robotwb.Leg[i].tar_epos_h.z = z;
    inv_KI(i, robotwb.Leg[i].tar_epos_h, sita1, sita2, sita3);
    return 1;
}

char cal_invjacobi(VMC *in)
{
    float det=0;
    char i = 0;
    if(vmc_all.param.leg_dof==3)
        invet33(in->jacobi33, &det, in->ijacobi33);
    else
        invet22(in->jacobi22, &det, in->ijacobi22);
    if (fabs(det) < 0.00001)
        return 0;
    return 1;
}

char cal_jacobi_new(VMC *in,float dt)
{
    float Jcobi[9];

    if(vmc_all.param.leg_dof==2){
        int i = 0;
        float Alpha1 = in->sita1;
        float Alpha2 = in->sita2;
        float Alpha3 = in->sita3;
        float Param[6] = { in->l1 ,in->l2,in->l2,in->l1 ,0, in->l3 };

        float L1 = Param[0];
        float L2 = Param[1];
        float L3 = Param[2];
        float L4 = Param[3];
        float L5 = Param[4];
        float L6 = Param[5];
        float s1 = sind(Alpha1);
        float c1 = cosd(Alpha1);
        float s2 = sind(Alpha2);
        float c2 = cosd(Alpha2);
        float s3 = sind(Alpha3);
        float c3 = cosd(Alpha3);

        float xa = L1 * c2;//
        float xa_1 = 0;
        float xa_2 = -L1 * s2;
        float xa_3 = 0;

        float ya = L1 * s2;//
        float ya_1 = 0;
        float ya_2 = L1 * c2;
        float ya_3 = 0;

        float xc = L5 + L4 * c1;//
        float xc_1 = -L4 * s1;
        float xc_2 = 0;
        float xc_3 = 0;

        float yc = L4 * s1;//
        float yc_1 = L4 * c1;
        float yc_2 = 0;
        float yc_3 = 0;

        float ll = sqrtf(powf(xc - xa, 2) + powf(yc - ya, 2));//
        float ll_1 = 0.5 / ll * (2.0 * (xc - xa)*(xc_1 - xa_1) + 2.0 * (yc - ya)*(yc_1 - ya_1));
        float ll_2 = 0.5 / ll * (2.0 * (xc - xa)*(xc_2 - xa_2) + 2.0 * (yc - ya)*(yc_2 - ya_2));
        float ll_3 = 0.5 / ll * (2.0 * (xc - xa)*(xc_3 - xa_3) + 2.0 * (yc - ya)*(yc_3 - ya_3));

        float Aa = 2.0 * L2*(xc - xa);//
        float Aa_1 = 2.0 * L2*(xc_1 - xa_1);
        float Aa_2 = 2.0 * L2*(xc_2 - xa_2);
        float Aa_3 = 2.0 * L2*(xc_3 - xa_3);

        float Bb = 2.0 * L2*(yc - ya);//
        float Bb_1 = 2.0 * L2*(yc_1 - ya_1);
        float Bb_2 = 2.0 * L2*(yc_2 - ya_2);
        float Bb_3 = 2.0 * L2*(yc_3 - ya_3);

        float Cc = L2 * L2 + ll * ll - L3 * L3;//
        float Cc_1 = 2.0 * ll*ll_1;
        float Cc_2 = 2.0 * ll*ll_2;
        float Cc_3 = 2.0 * ll*ll_3;

        float Tt = Aa * Aa + Bb * Bb - Cc * Cc;//
        float Tt_1 = 2.0 * Aa*Aa_1 + 2.0 * Bb * Bb_1 - 2.0 * Cc * Cc_1;
        float Tt_2 = 2.0 * Aa*Aa_2 + 2.0 * Bb * Bb_2 - 2.0 * Cc * Cc_2;
        float Tt_3 = 2.0 * Aa*Aa_3 + 2.0 * Bb * Bb_3 - 2.0 * Cc * Cc_3;

        float Ttt = (Bb + sqrtf(Tt)) / (Aa + Cc);
        float Ttt_1 = (Bb_1 + 0.5 / sqrtf(Tt)*Tt_1) / (Aa + Cc) + (Bb + sqrtf(Tt))*-1.0 / powf(Aa + Cc, 2.0)*(Aa_1 + Cc_1);
        float Ttt_2 = (Bb_2 + 0.5 / sqrtf(Tt)*Tt_2) / (Aa + Cc) + (Bb + sqrtf(Tt))*-1.0 / powf(Aa + Cc, 2.0)*(Aa_2 + Cc_2);
        float Ttt_3 = (Bb_3 + 0.5 / sqrtf(Tt)*Tt_3) / (Aa + Cc) + (Bb + sqrtf(Tt))*-1.0 / powf(Aa + Cc, 2.0)*(Aa_3 + Cc_3);

        float a1 = 2.0 * atan(Ttt);//
        float a1_1 = 2.0 / (1.0 + powf(Ttt, 2.0))*Ttt_1;
        float a1_2 = 2.0 / (1.0 + powf(Ttt, 2.0))*Ttt_2;
        float a1_3 = 2.0 / (1.0 + powf(Ttt, 2.0))*Ttt_3;

        float xe = xa + (L2 + L6)*cos(a1);
        float xe_1 = xa_1 + (L2 + L6)*-1.0 * sin(a1)*a1_1;
        float xe_2 = xa_2 + (L2 + L6)*-1.0 * sin(a1)*a1_2;
        float xe_3 = xa_3 + (L2 + L6)*-1.0 * sin(a1)*a1_3;

        float ze = ya + (L2 + L6)*sin(a1);
        float ze_1 = ya_1 + (L2 + L6)*cos(a1)*a1_1;
        float ze_2 = ya_2 + (L2 + L6)*cos(a1)*a1_2;
        float ze_3 = ya_3 + (L2 + L6)*cos(a1)*a1_3;

        Jcobi[0] = xe_1;//x / s0
        Jcobi[1] = xe_2;//x / s1
        Jcobi[2] = xe_3;//x / s2

        Jcobi[3] = -s3 * ze_1;//y / s0
        Jcobi[4] = -s3 * ze_2;//y / s1
        Jcobi[5] = -c3 * ze - s3 * ze_3;//y / s2

        Jcobi[6] = -c3 * ze_1;//z / s0
        Jcobi[7] = -c3 * ze_2;//z / s1
        Jcobi[8] =  s3 * ze - c3 * ze_3;//z / s2

        for (i = 0; i < 9; i++)
            in->jacobi33[i] = Jcobi[i];

        in->jacobi22[2 * 0 + 0] = in->jacobi33[0 * 3 + 0];
        in->jacobi22[2 * 0 + 1] = in->jacobi33[0 * 3 + 1];
        in->jacobi22[2 * 1 + 0] = in->jacobi33[2 * 3 + 0];
        in->jacobi22[2 * 1 + 1] = in->jacobi33[2 * 3 + 1];
    }
    else{
        float a = in->l3;
        float L1 = in->l1;
        float L2 = in->l2;

        float s0 = in->flag_rl*in->sita3;
        float s1 = in->sita1;
        float s2 = 180 - in->sita2;

        float H = L1 * cosd(s1) + L2 * cosd(-s1 + s2);
        float Hs0 = 0;
        float Hs1 = L1 * -sind(s1) + L2 * -sind(-s1 + s2)*-1;
        float Hs2 = L2 * -sind(-s1 + s2)*-1;

        float x  = -(L1 * sind(-s1) + L2 * sind(-s1 + s2));
        Jcobi[0] = -1 * -(L1 * cosd(-s1)*-1 + L2 * cosd(-s1 + s2)*-1);   //s1
        //Jcobi[0] =//in->param.invert_knee_epos[Xr]*
        //	-in->l1 * cosdw(in->sita1) - in->l2 * cosdw(180 - in->sita2 - in->sita1);//x p_D
        Jcobi[1] = -1 * -(0 + L2 * cosd(-s1 + s2)*-1);					       //s2
        Jcobi[2] = 0;//s0
        float y = -in->flag_rl*(a * cosd(s0) + H * sind(s0));
        Jcobi[3] = -1 *-in->flag_rl*(Hs1 * sind(s0));//s1
        Jcobi[4] = -1 * -in->flag_rl*(Hs2 * sind(s0));//s2
        Jcobi[5] = -1 * -in->flag_rl*(-a * sind(s0)*in->flag_rl + Hs0 * sind(s0) + H * cosd(s0)*in->flag_rl);	//s0
        float z = -H * cosd(s0) + a * sind(s0);
        Jcobi[6] = -Hs1 * cosd(s0) ;//s1
        Jcobi[7] = -Hs2 * cosd(s0);//s2
        Jcobi[8] = -Hs0 * cosd(s0) - Hs0 * -sind(s0)*in->flag_rl + a * cosd(s0)*in->flag_rl;	//s0


#if 1
        Jcobi[0] =//in->param.invert_knee_epos[Xr]*
            -in->l1 * cosdw(in->sita1) - in->l2 * cosdw(180 - in->sita2 - in->sita1);//x p_D
        Jcobi[1] =//in->param.invert_knee_epos[Xr]*
            -in->l2 * cosdw(180 - in->sita2 - in->sita1);//x p_X
        Jcobi[2] =//in->param.invert_knee_epos[Xr]*
            0;//x p_T
        float h = in->l1 * cosdw(in->sita1) + in->l2 * cosdw(180 - in->sita2 - in->sita1);
        float h_s1 = -in->l1 * sindw(in->sita1) + in->l2 * sindw(180 - in->sita2 - in->sita1);
        float h_s2 = in->l2 * sindw(180 - in->sita2 - in->sita1);
        float h_s3 = 0;

        Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
            h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
        Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
            h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
        Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
            (-in->l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
                h_s3 * sindw(in->sita3 * in->flag_rl) +
                h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

        Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
        Jcobi[7] = -h_s2 * cosdw(in->sita3 * in->flag_rl);//z p_X
        Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
            + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
            + in->l3 * cosdw(in->sita3)*in->flag_rl;//z p_T
#endif
        for (int i = 0; i < 9; i++)
            in->jacobi33[i] = Jcobi[i];

        for (int i = 0; i < 9; i++)
            in->d_jacobi33[i] = (Jcobi[i]-in->jacobi33_reg[i])/dt;

        for (int i = 0; i < 9; i++)
            in->jacobi33_reg[i] = Jcobi[i];
    }
if(vmc[0].param.invert_knee_epos[Xr]==1&&
   vmc[1].param.invert_knee_epos[Xr]==1&&
   vmc[2].param.invert_knee_epos[Xr]==1&&
   vmc[3].param.invert_knee_epos[Xr]==1){//mini cheetah unuse
    //------------------------------------------------------------------
//#define MESS_KIN1 0.63 //kg
//#define MESS_KIN12 0.35 //kg
//#define MESS_KIN2 0.63 //0.46+0.335 //kg
//#define L_MESS_KIN1 0.045
//#define L_MESS_KIN12 0.08
//#define L_MESS_KIN2 0.05
        //jacobi MIT Leg1
        float l3=-L_MESS_KIN1;
        Jcobi[0] =//in->param.invert_knee_epos[Xr]*
        -0 * cosdw(in->sita1) - 0 * cosdw(180 - in->sita2 - in->sita1);//x p_D
        Jcobi[1] =//in->param.invert_knee_epos[Xr]*
        0 * cosdw(180 - in->sita2 - in->sita1);//x p_X
        Jcobi[2] =//in->param.invert_knee_epos[Xr]*
        0;//x p_T

        float h = 0* cosdw(in->sita1) + 0 * cosdw(180 - in->sita2 - in->sita1);
        float h_s1 = -0 * sindw(in->sita1) + 0 * sindw(180 - in->sita2- in->sita1);
        float h_s2 = 0 * sindw(180 - in->sita2 - in->sita1);
        float h_s3 = 0;

        Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
        Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
        Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        (-l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
            h_s3 * sindw(in->sita3 * in->flag_rl) +
            h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

        Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
        Jcobi[7] = -h_s2 * cosdw(in->sita3* in->flag_rl);//z p_X
        Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
        + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
        + l3 * cosdw(in->sita3)*in->flag_rl;//z p_T

        for (int i = 0; i < 9; i++)
            in->jacobi33_kin1[i] = Jcobi[i];

        //jacobi MIT Leg12-------------------------------
        float cog_rate=0.5;
        l3=L_MESS_KIN12;
        Jcobi[0] =//in->param.invert_knee_epos[Xr]*
        -in->l1*cog_rate * cosdw(in->sita1) - 0 * cosdw(180 - in->sita2 - in->sita1);//x p_D
        Jcobi[1] =//in->param.invert_knee_epos[Xr]*
        0 * cosdw(180 - in->sita2 - in->sita1);//x p_X
        Jcobi[2] =//in->param.invert_knee_epos[Xr]*
        0;//x p_T

        h = in->l1*cog_rate * cosdw(in->sita1) + 0 * cosdw(180 - in->sita2 - in->sita1);
        h_s1 = -in->l1*cog_rate * sindw(in->sita1) + 0 * sindw(180 - in->sita2- in->sita1);
        h_s2 = 0 * sindw(180 - in->sita2 - in->sita1);
        h_s3 = 0;

        Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
        Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
        Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        (-l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
            h_s3 * sindw(in->sita3 * in->flag_rl) +
            h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

        Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
        Jcobi[7] = -h_s2 * cosdw(in->sita3* in->flag_rl);//z p_X
        Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
        + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
        + l3 * cosdw(in->sita3)*in->flag_rl;//z p_T

        for (int i = 0; i < 9; i++)
            in->jacobi33_kin12[i] = Jcobi[i];

        //jacobi MIT Leg2--------------------------------------------
        l3=L_MESS_KIN2;
        Jcobi[0] =//in->param.invert_knee_epos[Xr]*
        -0 * cosdw(in->sita1) - 0 * cosdw(180 - in->sita2 - in->sita1);//x p_D
        Jcobi[1] =//in->param.invert_knee_epos[Xr]*
        0 * cosdw(180 - in->sita2 - in->sita1);//x p_X
        Jcobi[2] =//in->param.invert_knee_epos[Xr]*
        0;//x p_T

        h = 0* cosdw(in->sita1) + 0 * cosdw(180 - in->sita2 - in->sita1);
        h_s1 = -0 * sindw(in->sita1) + 0 * sindw(180 - in->sita2- in->sita1);
        h_s2 = 0 * sindw(180 - in->sita2 - in->sita1);
        h_s3 = 0;

        Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
        Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
        Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
        (-l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
            h_s3 * sindw(in->sita3 * in->flag_rl) +
            h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

        Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
        Jcobi[7] = -h_s2 * cosdw(in->sita3* in->flag_rl);//z p_X
        Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
        + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
        + l3 * cosdw(in->sita3)*in->flag_rl;//z p_T

        for (int i = 0; i < 9; i++)
            in->jacobi33_kin2[i] = Jcobi[i];

}else{//ETH
//------------------------------------------------------------------
    //jacobi MIT Leg1
    float l3=L_MESS_KIN1;
    Jcobi[0] =//in->param.invert_knee_epos[Xr]*
    -0 * cosdw(in->sita1) - 0 * cosdw(180 - in->sita2 - in->sita1);//x p_D
    Jcobi[1] =//in->param.invert_knee_epos[Xr]*
    0 * cosdw(180 - in->sita2 - in->sita1);//x p_X
    Jcobi[2] =//in->param.invert_knee_epos[Xr]*
    0;//x p_T

    float h = 0* cosdw(in->sita1) + 0 * cosdw(180 - in->sita2 - in->sita1);
    float h_s1 = -0 * sindw(in->sita1) + 0 * sindw(180 - in->sita2- in->sita1);
    float h_s2 = 0 * sindw(180 - in->sita2 - in->sita1);
    float h_s3 = 0;

    Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
    Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
    Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    (-l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
        h_s3 * sindw(in->sita3 * in->flag_rl) +
        h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

    Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
    Jcobi[7] = -h_s2 * cosdw(in->sita3* in->flag_rl);//z p_X
    Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
    + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
    + l3 * cosdw(in->sita3)*in->flag_rl;//z p_T

    for (int i = 0; i < 9; i++)
        in->jacobi33_kin1[i] = Jcobi[i];

    //jacobi MIT Leg12-------------------------------
    float cog_rate=L_MESS_KIN12;
    l3=L_MESS_KIN2;
    Jcobi[0] =//in->param.invert_knee_epos[Xr]*
    -in->l1*cog_rate * cosdw(in->sita1) - 0 * cosdw(180 - in->sita2 - in->sita1);//x p_D
    Jcobi[1] =//in->param.invert_knee_epos[Xr]*
    0 * cosdw(180 - in->sita2 - in->sita1);//x p_X
    Jcobi[2] =//in->param.invert_knee_epos[Xr]*
    0;//x p_T

    h = in->l1*cog_rate * cosdw(in->sita1) + 0 * cosdw(180 - in->sita2 - in->sita1);
    h_s1 = -in->l1*cog_rate * sindw(in->sita1) + 0 * sindw(180 - in->sita2- in->sita1);
    h_s2 = 0 * sindw(180 - in->sita2 - in->sita1);
    h_s3 = 0;

    Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
    Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
    Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    (-l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
        h_s3 * sindw(in->sita3 * in->flag_rl) +
        h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

    Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
    Jcobi[7] = -h_s2 * cosdw(in->sita3* in->flag_rl);//z p_X
    Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
    + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
    + l3 * cosdw(in->sita3)*in->flag_rl;//z p_T

    for (int i = 0; i < 9; i++)
        in->jacobi33_kin12[i] = Jcobi[i];

    //jacobi MIT Leg2--------------------------------------------
    l3=L_MESS_KIN2;
    Jcobi[0] =//in->param.invert_knee_epos[Xr]*
    -in->l1 * cosdw(in->sita1) - 0 * cosdw(180 - in->sita2 - in->sita1);//x p_D
    Jcobi[1] =//in->param.invert_knee_epos[Xr]*
    0 * cosdw(180 - in->sita2 - in->sita1);//x p_X
    Jcobi[2] =//in->param.invert_knee_epos[Xr]*
    0;//x p_T

    h = in->l1 * cosdw(in->sita1) + 0 * cosdw(180 - in->sita2 - in->sita1);
    h_s1 = -in->l1 * sindw(in->sita1) + 0 * sindw(180 - in->sita2- in->sita1);
    h_s2 = 0 * sindw(180 - in->sita2 - in->sita1);
    h_s3 = 0;

    Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
    Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
    Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
    (-l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
        h_s3 * sindw(in->sita3 * in->flag_rl) +
        h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

    Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
    Jcobi[7] = -h_s2 * cosdw(in->sita3 * in->flag_rl);//z p_X
    Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
    + h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
    + l3 * cosdw(in->sita3)*in->flag_rl;//z p_T

    for (int i = 0; i < 9; i++)
        in->jacobi33_kin2[i] = Jcobi[i];
}
    return 1;
}
