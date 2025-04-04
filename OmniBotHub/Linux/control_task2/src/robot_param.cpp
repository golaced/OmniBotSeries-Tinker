#include "locomotion_header.h"
#include "base_struct.h"
#include "can.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include <iostream>
using namespace std;
YAML::Node config_robot=YAML::LoadFile("/home/odroid/Tinker/Param/param_robot.yaml");
YAML::Node config_gait=YAML::LoadFile("/home/odroid/Tinker/Param/param_gait.yaml");

void robot_param_read(void)
{
    int i=0;
    printf("--Load Robot Yaml--\n");

    vmc_all.param.soft_weight=1;
    gait_ww.auto_switch=config_gait["sys_param"]["auto_gait_switch"].as<float>();
    gait_ww.auto_gait_time=config_gait["sys_param"]["auto_gait_time"].as<float>();
    gait_ww.auto_zmp_st_check=config_gait["sys_param"]["auto_zmp_st_check"].as<float>();
    gait_ww.auto_mess_est=config_gait["sys_param"]["auto_mess_est"].as<float>();

    MAX_SPD_X= config_gait["vmc_param"]["max_spd_x"].as<float>();
    MAX_SPD_Y= config_gait["vmc_param"]["max_spd_y"].as<float>();
    MAX_SPD_RAD= config_gait["vmc_param"]["max_spd_rotate"].as<float>();

    //----------------------------底层伺服参数--------------------------------
    leg_motor_all.stiff_init=config_gait["imp_param"]["stiff_init"].as<float>();
    leg_motor_all.stiff_stand=config_gait["imp_param"]["stiff_stand"].as<float>();

    leg_motor_all.kp[0]=config_gait["imp_param"]["kp_q00"].as<float>();
    leg_motor_all.kp[1]=config_gait["imp_param"]["kp_q01"].as<float>();
    leg_motor_all.kp[2]=config_gait["imp_param"]["kp_q02"].as<float>();
    leg_motor_all.kp[3]=config_gait["imp_param"]["kp_q03"].as<float>();
    leg_motor_all.kp[4]=config_gait["imp_param"]["kp_q04"].as<float>();
    leg_motor_all.kp[5]=config_gait["imp_param"]["kp_q05"].as<float>();
    leg_motor_all.kp[6]=config_gait["imp_param"]["kp_q06"].as<float>();

    leg_motor_all.kp[7]=config_gait["imp_param"]["kp_q10"].as<float>();
    leg_motor_all.kp[8]=config_gait["imp_param"]["kp_q11"].as<float>();
    leg_motor_all.kp[9]=config_gait["imp_param"]["kp_q12"].as<float>();
    leg_motor_all.kp[10]=config_gait["imp_param"]["kp_q13"].as<float>();
    leg_motor_all.kp[11]=config_gait["imp_param"]["kp_q14"].as<float>();
    leg_motor_all.kp[12]=config_gait["imp_param"]["kp_q15"].as<float>();
    leg_motor_all.kp[13]=config_gait["imp_param"]["kp_q16"].as<float>();
    printf("param kp loading...\n");
    leg_motor_all.kd[0]=config_gait["imp_param"]["kd_q00"].as<float>();
    leg_motor_all.kd[1]=config_gait["imp_param"]["kd_q01"].as<float>();
    leg_motor_all.kd[2]=config_gait["imp_param"]["kd_q02"].as<float>();
    leg_motor_all.kd[3]=config_gait["imp_param"]["kd_q03"].as<float>();
    leg_motor_all.kd[4]=config_gait["imp_param"]["kd_q04"].as<float>();
    leg_motor_all.kd[5]=config_gait["imp_param"]["kd_q05"].as<float>();
    leg_motor_all.kd[6]=config_gait["imp_param"]["kd_q06"].as<float>();

    leg_motor_all.kd[7]=config_gait["imp_param"]["kd_q10"].as<float>();
    leg_motor_all.kd[8]=config_gait["imp_param"]["kd_q11"].as<float>();
    leg_motor_all.kd[9]=config_gait["imp_param"]["kd_q12"].as<float>();
    leg_motor_all.kd[10]=config_gait["imp_param"]["kd_q13"].as<float>();
    leg_motor_all.kd[11]=config_gait["imp_param"]["kd_q14"].as<float>();
    leg_motor_all.kd[12]=config_gait["imp_param"]["kd_q15"].as<float>();
    leg_motor_all.kd[13]=config_gait["imp_param"]["kd_q16"].as<float>();
    printf("param kd loading...\n");
    leg_motor_all.stiff[0]=config_gait["imp_param"]["stiff_q00"].as<float>();
    leg_motor_all.stiff[1]=config_gait["imp_param"]["stiff_q01"].as<float>();
    leg_motor_all.stiff[2]=config_gait["imp_param"]["stiff_q02"].as<float>();
    leg_motor_all.stiff[3]=config_gait["imp_param"]["stiff_q03"].as<float>();
    leg_motor_all.stiff[4]=config_gait["imp_param"]["stiff_q04"].as<float>();
    leg_motor_all.stiff[5]=config_gait["imp_param"]["stiff_q05"].as<float>();
    leg_motor_all.stiff[6]=config_gait["imp_param"]["stiff_q06"].as<float>();

    leg_motor_all.stiff[7]=config_gait["imp_param"]["stiff_q10"].as<float>();
    leg_motor_all.stiff[8]=config_gait["imp_param"]["stiff_q11"].as<float>();
    leg_motor_all.stiff[9]=config_gait["imp_param"]["stiff_q12"].as<float>();
    leg_motor_all.stiff[10]=config_gait["imp_param"]["stiff_q13"].as<float>();
    leg_motor_all.stiff[11]=config_gait["imp_param"]["stiff_q14"].as<float>();
    leg_motor_all.stiff[12]=config_gait["imp_param"]["stiff_q15"].as<float>();
    leg_motor_all.stiff[13]=config_gait["imp_param"]["stiff_q16"].as<float>();
    printf("param stiff loading...\n");
    leg_motor_all.q_init[0]=config_robot["kin_param"]["init_q00"].as<float>();
    leg_motor_all.q_init[1]=config_robot["kin_param"]["init_q01"].as<float>();
    leg_motor_all.q_init[2]=config_robot["kin_param"]["init_q02"].as<float>();
    leg_motor_all.q_init[3]=config_robot["kin_param"]["init_q03"].as<float>();
    leg_motor_all.q_init[4]=config_robot["kin_param"]["init_q04"].as<float>();
    leg_motor_all.q_init[5]=config_robot["kin_param"]["init_q05"].as<float>();
    leg_motor_all.q_init[6]=config_robot["kin_param"]["init_q06"].as<float>();

    leg_motor_all.q_init[7]=config_robot["kin_param"]["init_q10"].as<float>();
    leg_motor_all.q_init[8]=config_robot["kin_param"]["init_q11"].as<float>();
    leg_motor_all.q_init[9]=config_robot["kin_param"]["init_q12"].as<float>();
    leg_motor_all.q_init[10]=config_robot["kin_param"]["init_q13"].as<float>();
    leg_motor_all.q_init[11]=config_robot["kin_param"]["init_q14"].as<float>();
    leg_motor_all.q_init[12]=config_robot["kin_param"]["init_q15"].as<float>();
    leg_motor_all.q_init[13]=config_robot["kin_param"]["init_q16"].as<float>();
    //for kick
    printf("param init loading...\n");//kick ball
    leg_motor_all.q_init1[0]=config_robot["kin_param"]["init_q00"].as<float>();
    leg_motor_all.q_init1[1]=config_robot["kin_param"]["init_q01"].as<float>()*2;
    leg_motor_all.q_init1[2]=config_robot["kin_param"]["init_q02"].as<float>();
    leg_motor_all.q_init1[3]=config_robot["kin_param"]["init_q03"].as<float>();
    leg_motor_all.q_init1[4]=config_robot["kin_param"]["init_q04"].as<float>();
    leg_motor_all.q_init1[5]=config_robot["kin_param"]["init_q05"].as<float>();
    leg_motor_all.q_init1[6]=config_robot["kin_param"]["init_q06"].as<float>();

    leg_motor_all.q_init1[7]=config_robot["kin_param"]["init_q10"].as<float>();
    leg_motor_all.q_init1[8]=config_robot["kin_param"]["init_q11"].as<float>();
    leg_motor_all.q_init1[9]=config_robot["kin_param"]["init_q12"].as<float>()*0.2;
    leg_motor_all.q_init1[10]=config_robot["kin_param"]["init_q13"].as<float>()*1.5;
    leg_motor_all.q_init1[11]=config_robot["kin_param"]["init_q14"].as<float>()*1.35;
    leg_motor_all.q_init1[12]=config_robot["kin_param"]["init_q15"].as<float>();
    leg_motor_all.q_init1[13]=config_robot["kin_param"]["init_q16"].as<float>();
    printf("param init1 loading...\n");
    leg_motor_all.q_reset[0]=config_robot["kin_param"]["reset_q00"].as<float>();
    leg_motor_all.q_reset[1]=config_robot["kin_param"]["reset_q01"].as<float>();
    leg_motor_all.q_reset[2]=config_robot["kin_param"]["reset_q02"].as<float>();
    leg_motor_all.q_reset[3]=config_robot["kin_param"]["reset_q03"].as<float>();
    leg_motor_all.q_reset[4]=config_robot["kin_param"]["reset_q04"].as<float>();
    leg_motor_all.q_reset[5]=config_robot["kin_param"]["reset_q05"].as<float>();
    leg_motor_all.q_reset[6]=config_robot["kin_param"]["reset_q06"].as<float>();

    leg_motor_all.q_reset[7]=config_robot["kin_param"]["reset_q10"].as<float>();
    leg_motor_all.q_reset[8]=config_robot["kin_param"]["reset_q11"].as<float>();
    leg_motor_all.q_reset[9]=config_robot["kin_param"]["reset_q12"].as<float>();
    leg_motor_all.q_reset[10]=config_robot["kin_param"]["reset_q13"].as<float>();
    leg_motor_all.q_reset[11]=config_robot["kin_param"]["reset_q14"].as<float>();
    leg_motor_all.q_reset[12]=config_robot["kin_param"]["reset_q15"].as<float>();
    leg_motor_all.q_reset[13]=config_robot["kin_param"]["reset_q16"].as<float>();
    printf("param reset loading...\n");
    leg_motor_all.q_max[0]=config_robot["kin_param"]["max_q00"].as<float>();
    leg_motor_all.q_max[1]=config_robot["kin_param"]["max_q01"].as<float>();
    leg_motor_all.q_max[2]=config_robot["kin_param"]["max_q02"].as<float>();
    leg_motor_all.q_max[3]=config_robot["kin_param"]["max_q03"].as<float>();
    leg_motor_all.q_max[4]=config_robot["kin_param"]["max_q04"].as<float>();
    leg_motor_all.q_max[5]=config_robot["kin_param"]["max_q05"].as<float>();
    leg_motor_all.q_max[6]=config_robot["kin_param"]["max_q06"].as<float>();

    leg_motor_all.q_max[7]=config_robot["kin_param"]["max_q10"].as<float>();
    leg_motor_all.q_max[8]=config_robot["kin_param"]["max_q11"].as<float>();
    leg_motor_all.q_max[9]=config_robot["kin_param"]["max_q12"].as<float>();
    leg_motor_all.q_max[10]=config_robot["kin_param"]["max_q13"].as<float>();
    leg_motor_all.q_max[11]=config_robot["kin_param"]["max_q14"].as<float>();
    leg_motor_all.q_max[12]=config_robot["kin_param"]["max_q15"].as<float>();
    leg_motor_all.q_max[13]=config_robot["kin_param"]["max_q16"].as<float>();
    printf("param qmax loading...\n");
    leg_motor_all.q_min[0]=config_robot["kin_param"]["min_q00"].as<float>();
    leg_motor_all.q_min[1]=config_robot["kin_param"]["min_q01"].as<float>();
    leg_motor_all.q_min[2]=config_robot["kin_param"]["min_q02"].as<float>();
    leg_motor_all.q_min[3]=config_robot["kin_param"]["min_q03"].as<float>();
    leg_motor_all.q_min[4]=config_robot["kin_param"]["min_q04"].as<float>();
    leg_motor_all.q_min[5]=config_robot["kin_param"]["min_q05"].as<float>();
    leg_motor_all.q_min[6]=config_robot["kin_param"]["min_q06"].as<float>();

    leg_motor_all.q_min[7]=config_robot["kin_param"]["min_q10"].as<float>();
    leg_motor_all.q_min[8]=config_robot["kin_param"]["min_q11"].as<float>();
    leg_motor_all.q_min[9]=config_robot["kin_param"]["min_q12"].as<float>();
    leg_motor_all.q_min[10]=config_robot["kin_param"]["min_q13"].as<float>();
    leg_motor_all.q_min[11]=config_robot["kin_param"]["min_q14"].as<float>();
    leg_motor_all.q_min[12]=config_robot["kin_param"]["min_q15"].as<float>();
    leg_motor_all.q_min[13]=config_robot["kin_param"]["min_q16"].as<float>();
    printf("param qmin loading...\n");
//---------------------------------servo
    leg_motor_all.q_set_servo_init[0]=config_robot["servo_param"]["init_q00"].as<float>();
    leg_motor_all.q_set_servo_init[1]=config_robot["servo_param"]["init_q01"].as<float>();
    leg_motor_all.q_set_servo_init[2]=config_robot["servo_param"]["init_q02"].as<float>();
    leg_motor_all.q_set_servo_init[3]=config_robot["servo_param"]["init_q03"].as<float>();
    leg_motor_all.q_set_servo_init[4]=config_robot["servo_param"]["init_q04"].as<float>();
    leg_motor_all.q_set_servo_init[5]=config_robot["servo_param"]["init_q05"].as<float>();
    leg_motor_all.q_set_servo_init[6]=config_robot["servo_param"]["init_q06"].as<float>();

    leg_motor_all.q_set_servo_init[7]=config_robot["servo_param"]["init_q10"].as<float>();
    leg_motor_all.q_set_servo_init[8]=config_robot["servo_param"]["init_q11"].as<float>();
    leg_motor_all.q_set_servo_init[9]=config_robot["servo_param"]["init_q12"].as<float>();
    leg_motor_all.q_set_servo_init[10]=config_robot["servo_param"]["init_q13"].as<float>();
    leg_motor_all.q_set_servo_init[11]=config_robot["servo_param"]["init_q14"].as<float>();
    leg_motor_all.q_set_servo_init[12]=config_robot["servo_param"]["init_q15"].as<float>();
    leg_motor_all.q_set_servo_init[13]=config_robot["servo_param"]["init_q16"].as<float>();

    leg_motor_all.q_set_servo_off[0]=config_robot["servo_param"]["reset_q00"].as<float>();
    leg_motor_all.q_set_servo_off[1]=config_robot["servo_param"]["reset_q01"].as<float>();
    leg_motor_all.q_set_servo_off[2]=config_robot["servo_param"]["reset_q02"].as<float>();
    leg_motor_all.q_set_servo_off[3]=config_robot["servo_param"]["reset_q03"].as<float>();
    leg_motor_all.q_set_servo_off[4]=config_robot["servo_param"]["reset_q04"].as<float>();
    leg_motor_all.q_set_servo_off[5]=config_robot["servo_param"]["reset_q05"].as<float>();
    leg_motor_all.q_set_servo_off[6]=config_robot["servo_param"]["reset_q06"].as<float>();

    leg_motor_all.q_set_servo_off[7]=config_robot["servo_param"]["reset_q10"].as<float>();
    leg_motor_all.q_set_servo_off[8]=config_robot["servo_param"]["reset_q11"].as<float>();
    leg_motor_all.q_set_servo_off[9]=config_robot["servo_param"]["reset_q12"].as<float>();
    leg_motor_all.q_set_servo_off[10]=config_robot["servo_param"]["reset_q13"].as<float>();
    leg_motor_all.q_set_servo_off[11]=config_robot["servo_param"]["reset_q14"].as<float>();
    leg_motor_all.q_set_servo_off[12]=config_robot["servo_param"]["reset_q15"].as<float>();
    leg_motor_all.q_set_servo_off[13]=config_robot["servo_param"]["reset_q16"].as<float>();

    leg_motor_all.q_servo_max[0]=config_robot["servo_param"]["max_q00"].as<float>();
    leg_motor_all.q_servo_max[1]=config_robot["servo_param"]["max_q01"].as<float>();
    leg_motor_all.q_servo_max[2]=config_robot["servo_param"]["max_q02"].as<float>();
    leg_motor_all.q_servo_max[3]=config_robot["servo_param"]["max_q03"].as<float>();
    leg_motor_all.q_servo_max[4]=config_robot["servo_param"]["max_q04"].as<float>();
    leg_motor_all.q_servo_max[5]=config_robot["servo_param"]["max_q05"].as<float>();
    leg_motor_all.q_servo_max[6]=config_robot["servo_param"]["max_q06"].as<float>();

    leg_motor_all.q_servo_max[7]=config_robot["servo_param"]["max_q10"].as<float>();
    leg_motor_all.q_servo_max[8]=config_robot["servo_param"]["max_q11"].as<float>();
    leg_motor_all.q_servo_max[9]=config_robot["servo_param"]["max_q12"].as<float>();
    leg_motor_all.q_servo_max[10]=config_robot["servo_param"]["max_q13"].as<float>();
    leg_motor_all.q_servo_max[11]=config_robot["servo_param"]["max_q14"].as<float>();
    leg_motor_all.q_servo_max[12]=config_robot["servo_param"]["max_q15"].as<float>();
    leg_motor_all.q_servo_max[13]=config_robot["servo_param"]["max_q16"].as<float>();

    leg_motor_all.q_servo_min[0]=config_robot["servo_param"]["min_q00"].as<float>();
    leg_motor_all.q_servo_min[1]=config_robot["servo_param"]["min_q01"].as<float>();
    leg_motor_all.q_servo_min[2]=config_robot["servo_param"]["min_q02"].as<float>();
    leg_motor_all.q_servo_min[3]=config_robot["servo_param"]["min_q03"].as<float>();
    leg_motor_all.q_servo_min[4]=config_robot["servo_param"]["min_q04"].as<float>();
    leg_motor_all.q_servo_min[5]=config_robot["servo_param"]["min_q05"].as<float>();
    leg_motor_all.q_servo_min[6]=config_robot["servo_param"]["min_q06"].as<float>();

    leg_motor_all.q_servo_min[7]=config_robot["servo_param"]["min_q10"].as<float>();
    leg_motor_all.q_servo_min[8]=config_robot["servo_param"]["min_q11"].as<float>();
    leg_motor_all.q_servo_min[9]=config_robot["servo_param"]["min_q12"].as<float>();
    leg_motor_all.q_servo_min[10]=config_robot["servo_param"]["min_q13"].as<float>();
    leg_motor_all.q_servo_min[11]=config_robot["servo_param"]["min_q14"].as<float>();
    leg_motor_all.q_servo_min[12]=config_robot["servo_param"]["min_q15"].as<float>();
    leg_motor_all.q_servo_min[13]=config_robot["servo_param"]["min_q16"].as<float>();
    printf("param servo loading...\n");

    for(int i=0;i<14;i++){
        leg_motor_all.q_set_servo[i]= leg_motor_all.q_set_servo_init[i];
        leg_motor_all.q_set_servo_out[i]=leg_motor_all.q_set_servo[i]+leg_motor_all.q_set_servo_off[i];
    }

    vmc_all.net_run_dt=config_gait["rl_gait"]["net_run_dt"].as<float>();
    vmc_all.action_scale=config_gait["rl_gait"]["action_scale"].as<float>();

    vmc_all.rl_commond_off[4]=config_gait["rl_gait"]["en_vel_off"].as<float>();
    vmc_all.rl_commond_off[0]=config_gait["rl_gait"]["vel_x_off"].as<float>();
    vmc_all.rl_commond_off[1]=config_gait["rl_gait"]["vel_y_off"].as<float>();

    vmc_all.default_action[0]=config_gait["rl_gait"]["def_act0"].as<float>();
    vmc_all.default_action[1]=config_gait["rl_gait"]["def_act1"].as<float>();
    vmc_all.default_action[2]=config_gait["rl_gait"]["def_act2"].as<float>();
    vmc_all.default_action[3]=config_gait["rl_gait"]["def_act3"].as<float>();
    vmc_all.default_action[4]=config_gait["rl_gait"]["def_act4"].as<float>();
    vmc_all.default_action[5]=config_gait["rl_gait"]["def_act5"].as<float>();
    vmc_all.default_action[6]=config_gait["rl_gait"]["def_act6"].as<float>();

    vmc_all.default_action[7]=config_gait["rl_gait"]["def_act7"].as<float>();
    vmc_all.default_action[8]=config_gait["rl_gait"]["def_act8"].as<float>();
    vmc_all.default_action[9]=config_gait["rl_gait"]["def_act9"].as<float>();
    vmc_all.default_action[10]=config_gait["rl_gait"]["def_act10"].as<float>();
    vmc_all.default_action[11]=config_gait["rl_gait"]["def_act11"].as<float>();
    vmc_all.default_action[12]=config_gait["rl_gait"]["def_act12"].as<float>();
    vmc_all.default_action[13]=config_gait["rl_gait"]["def_act13"].as<float>();
    printf("param rl-default loading...\n");

    //---------------------VMC 全局参数
    vmc_all.att_measure_bias[PITr]=config_gait["vmc_param"]["att_bias_pit"].as<float>();
    vmc_all.att_measure_bias[ROLr]=config_gait["vmc_param"]["att_bias_rol"].as<float>();
    vmc_all.att_measure_bias_flip[PITr]=config_gait["vmc_param"]["att_bias_pit_f"].as<float>();
    vmc_all.att_measure_bias_flip[ROLr]=config_gait["vmc_param"]["att_bias_rol_f"].as<float>();
    printf("IMU att fix bias is:P=%f R=%f Y=%f\n",vmc_all.att_measure_bias[PITr],vmc_all.att_measure_bias[ROLr],vmc_all.att_measure_bias[YAWr]);
    printf("IMU att_f fix bias is:P=%f R=%f Y=%f\n",vmc_all.att_measure_bias_flip[PITr],vmc_all.att_measure_bias_flip[ROLr],vmc_all.att_measure_bias_flip[YAWr]);

}
