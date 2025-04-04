#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "eso_RL.h"
#if !RUN_WEBOTS || RUN_PI
#include "can.h"
#endif

float q_i_max = 0.1;
float max_imp_spd[3] = {1.5, 1.5, 1.5};     // m/s
float max_imp_spd_air[3] = {1.5, 1.5, 1.5}; // m/s 空中的IMP位置移动

#if TSET_F_IMP && TEST_FF_MODE
float pos_force_enable[3] = {1, 1, 0}; // 力控标志位  位置  反馈  前馈
#elif TSET_F_FF && TEST_FF_MODE
float pos_force_enable[3] = {1, 0, 1}; // 力控标志位  位置  反馈  前馈
#else
float pos_force_enable[3] = {1, 1, 1}; // 力控标志位  位置  反馈  前馈
#endif

// Mess compass Kin1------------------------------------------------
float kin1_tau_limit[3] = {1.5, 1.5, 1.5};
float kin12_tau_limit[3] = {1.5, 1.5, 1.5};
float kin2_tau_limit[3] = {1.5, 1.5, 1.5};

char ground_reg[4] = {0};
char touch_reg[4] = {0};
char ff_flag_reg[4] = {0};
float kp_imp_spd = 1; // 0.25;
char ground_test[4] = {1, 1, 1, 1};
float imp_spd_test[3] = {0, 0, 0};
int nan_err1[10] = {0, 0, 0};

float min_q1[4] = {0};
float min_q2[4] = {0};
float min_q3[4] = {0};

float max_q1[4] = {0};
float max_q2[4] = {0};
float max_q3[4] = {0};

void force_control_and_dis_stand(float dt) // 底层==>位力混控制  站立步态！！
{
    int i = 0;
    float q_err[3] = {0};
    Vect3 Vect3_zero;
    Vect3_zero.x = Vect3_zero.y = Vect3_zero.z = 0;
    Vect3 force_err_n = Vect3_zero;
    Vect3 force_imp_spd_n = Vect3_zero;
    Vect3 force_imp_spd_n_yaw = Vect3_zero;
    Vect3 force_imp_spd_h = Vect3_zero;
    Vect3 epos_h_next = Vect3_zero;
    float pos_taod[3] = {0, 0};
    float fb_force_taod[3] = {0, 0};
    float ff_force_taod[3] = {0, 0};
    float mess_force_taod1[3] = {0, 0};
    float mess_force_taod2[3] = {0, 0};
    float mess_force_taod12[3] = {0, 0};
    float dt_scale = 1; // 0.005/(dt+0.00000001);<---------------------------------微分尺度系数

    set_motor_t(i); // CAN底层输出赋值
}

void force_control_and_dis_rl(float dt) // 底层==>位力混控制  强化学习！！
{
    int i = 0;
    float q_err[3] = {0};
    Vect3 Vect3_zero;
    Vect3_zero.x = Vect3_zero.y = Vect3_zero.z = 0;
    Vect3 force_err_n = Vect3_zero;
    Vect3 force_imp_spd_n = Vect3_zero;
    Vect3 force_imp_spd_n_yaw = Vect3_zero;
    Vect3 force_imp_spd_h = Vect3_zero;
    Vect3 epos_h_next = Vect3_zero;
    float pos_taod[3] = {0, 0};
    float fb_force_taod[3] = {0, 0};
    float ff_force_taod[3] = {0, 0};
    float mess_force_taod1[3] = {0, 0};
    float mess_force_taod2[3] = {0, 0};
    float mess_force_taod12[3] = {0, 0};
    float dt_scale = 1; // 0.005/(dt+0.00000001);<---------------------------------微分尺度系数

     set_motor_t(i); // CAN底层输出赋值
}

void force_control_and_dis_rl_eso(float dt) // 底层==>位力混控制  强化学习 加干扰观测器！！
{
    int i = 0;
    int j = 0;
    for (i = 0; i < 4; i++)
    { 
        for (j = 0; i < 3; j++)
        {
            if (joint_eso[i][j].Is_initialized == 0)
            {
                ESO_Init(&joint_eso[i][j], robotwb.Leg[i].sita[j]*2*PI/360.0, robotwb.Leg[i].sita_d[j]*2*PI/360.0);
                joint_eso[i][j].Is_initialized = 1;
            }
            ESO_update(&joint_eso[i][j],robotwb.Leg[i].sita[j],robotwb.Leg[i].taom[j],dt);
            compute_torque_comp(&joint_eso[i][j]);
            robotwb.Leg[i].taod[j] = joint_eso[i][j].torque_comp;
        }
        set_motor_t(i); // CAN底层输出赋值
    }
}
