#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "can.h"
#include "spi_node.h"
#include "eso.h"
#if !RUN_WEBOTS
#include "beep.h"
#endif

Gait_Mode gait_ww;
POS_FORCE_PARM pos_force_p;
VMC_ROBOT_PARM vmc_robot_p;
VMC vmc[4];
VMC_ALL vmc_all;
ESO att_rate_eso[3];
_OCU ocu;
_SDK sdk;
_SLIP slip_mode;
robotTypeDef robotwb;

float MIN_Z=-0.1;
float MAX_Z=-0.19;
float MIN_Y=-0.1;
float MAX_Y=-0.1;
float MIN_X=-0.15;
float MAX_X=0.15;
float MAX_SPD=0;

float MAX_SPD_X=0.4;
float MAX_SPD_Y=0.15;
float MAX_SPD_RAD=35;
char state_pass=1;
char test_pos_st=0;//测试标志位
float test_st_exp[5]={15,10,0.04,0.03,1.5};//P R X Z SPD
char stand_force_enable_flag[5]={0};

float Www =0;//宽
float Hw = 0;//长
float L1w= 0;//1连杆
float L2w =0;//2连杆
float L3w =0;//3连杆
float Mw  =0;//总重量
float I_VMC[3]  = {0};// inert
float Q_SIDE_BIAS=0;
float I_2_Nm = 0;//力矩系数
float soft_weight[2]={0.8,0.8};

float  MESS_KIN1=0;//   0.63 //kg
float  MESS_KIN12=0;//   0.35 //kg
float  MESS_KIN2=0;//     0.63 //0.46+0.335 //kg
float  L_MESS_KIN1=0;// 1  0.045
float  L_MESS_KIN12=0;//  0.08
float  L_MESS_KIN2=0;//   0.06

void vmc_param_init(void)
{
    int i,j;
    float scale_x=1;
    float scale_z=20;
    float pos_gain_scale_st=1;
    float pos_gain_scale_sw=1;
    printf("Robot::vmc_param_init\n");
    Www = config_robot["robot_param"]["W"].as<float>();//         0.19	//宽
    Hw = config_robot["robot_param"]["H"].as<float>();//           0.365		//长
    L1w= config_robot["robot_param"]["L1"].as<float>();//           0.075  //1连杆
    L2w = config_robot["robot_param"]["L2"].as<float>();//          0.144		//2连杆
    L3w = config_robot["robot_param"]["L3"].as<float>();//          0.027	//3连杆
    Mw  = config_robot["robot_param"]["Mess"].as<float>();//          4.8  //kg     总重量

    MESS_KIN1=config_robot["dyn_param"]["MESS_KIN1"].as<float>();//   0.63 //kg
    MESS_KIN12=config_robot["dyn_param"]["MESS_KIN12"].as<float>();//   0.35 //kg
    MESS_KIN2=config_robot["dyn_param"]["MESS_KIN2"].as<float>();//     0.63 //0.46+0.335 //kg
    L_MESS_KIN1=config_robot["dyn_param"]["L_MESS_KIN1"].as<float>();// 1  0.045
    L_MESS_KIN12=config_robot["dyn_param"]["L_MESS_KIN12"].as<float>();//  0.08
    L_MESS_KIN2=config_robot["dyn_param"]["L_MESS_KIN2"].as<float>();//   0.06

    vmc_all.param.Ix_Body=config_robot["robot_param"]["Ix_Body"].as<float>();
    vmc_all.param.Iy_Body=config_robot["robot_param"]["Iy_Body"].as<float>();
    vmc_all.param.Iz_Body=config_robot["robot_param"]["Iz_Body"].as<float>();
    robotwb.mess_off[Xr]=0;
    robotwb.mess_off[Yr]=0;
    robotwb.mess_payload=0;

    for(i=0;i<4;i++){
        vmc[i].param.id=i;
    }
    //------------------------机械参数-------------------------
    vmc_all.l1 =  config_robot["robot_param"]["L1"].as<float>();
    vmc_all.l2 =  config_robot["robot_param"]["L2"].as<float>();
    vmc_all.l3 =  config_robot["robot_param"]["L3"].as<float>();
    vmc_all.H =   config_robot["robot_param"]["H"].as<float>();
    vmc_all.W =   config_robot["robot_param"]["W"].as<float>();
    vmc_all.param.leg_dof = config_robot["kin_param"]["dof_num"].as<float>();
    vmc_all.param.leg_type = LOOP;
    printf("Robot::Leg Dof is [%d]\n",vmc_all.param.leg_dof);

    vmc_all.gait_mode = IDLE;
    vmc_all.mess = config_robot["robot_param"]["Mess"].as<float>();//;

    vmc_all.use_ground_sensor_new = EN_GROUND_CHECK;
    vmc_all.tar_att_bias[PITr] = vmc_all.tar_att_bias[ROLr] = 0;
    vmc_all.param.param_vmc_default.stance_time[1] = vmc_all.param.param_vmc_default.stance_time[0];

    memcpy(&vmc_all.param.param_vmc, &vmc_all.param.param_vmc_default, sizeof(vmc_all.param.param_vmc_default));

    //--------------------------------------------------------
    vmc_all.gait_mode=IDLE;

    if(vmc_all.param.leg_type==LOOP||vmc_all.param.leg_type==PARALE_LOOP){
        MIN_Z = -sind(10)*vmc_all.l1 - sind(20)*vmc_all.l2;//0.04;
        MAX_Z = -sind(75)*vmc_all.l1 - sind(75)*vmc_all.l2;//0.09;

        MIN_Y = -sind(55)*(vmc_all.l1 + vmc_all.l2)*0.8;//0.06;
        MAX_Y = sind(55)*(vmc_all.l1 + vmc_all.l2)*0.8;
        MIN_X = -sind(65)*vmc_all.l2;
        MAX_X = sind(65)*vmc_all.l2;
    }else{//并联
        MIN_Z=-(cosd(40)*vmc_all.l2-cosd(60)*vmc_all.l1);
        MAX_Z=-(sind(66)*vmc_all.l1+sind(70)*vmc_all.l2);

        MIN_X=-vmc_all.l1*1.15;
        MAX_X=vmc_all.l1*1.15;
        MIN_Y=-vmc_all.l1*1.15;
        MAX_Y=vmc_all.l1*1.15;
    }

    float temp[10];
    temp[0]=cosd(45)*vmc_all.l1+cosd(45)*vmc_all.l2;
    temp[1]=cosd(65)*vmc_all.l1+cosd(65)*vmc_all.l2;

    vmc_all.param.MAX_PIT=LIMIT(atan2(temp[0]-temp[1],vmc_all.H)*57.3*1,-25,25);
    vmc_all.param.MAX_ROL=LIMIT(atan2(temp[0]-temp[1],vmc_all.W)*57.3*0.5,-25,25);

    temp[0]=cosd(45)*vmc_all.l1;
    temp[1]=sind(45)*vmc_all.l1;
    vmc_all.param.MAX_YAW=LIMIT(90-atan2(vmc_all.H+2*temp[1],2*temp[0])*57.3,-45,45)*0.7;
    printf("Param Setting::MAX_PIT=%f MAX_ROL=%f MAX_YAW=%f\n",vmc_all.param.MAX_PIT,vmc_all.param.MAX_ROL, vmc_all.param.MAX_YAW);
    att_rate_eso[ROLr].b0=att_rate_eso[PITr].b0=att_rate_eso[YAWr].b0=1;
    vmc_all.param.end_sample_dt=0.0025;

    vmc[FL1].flag_rl=vmc[BL1].flag_rl=1;
    vmc[FL2].flag_rl=vmc[BL2].flag_rl=-1;
    vmc[FL1].flag_fb=vmc[FL2].flag_fb=1;
    vmc[BL1].flag_fb=vmc[BL2].flag_fb=-1;

    if(vmc_all.param.leg_type==LOOP||vmc_all.param.leg_type==PARALE_LOOP)
    {vmc_all.sita_test[1]=90;vmc_all.sita_test[0]=90;}//
    else
    {vmc_all.sita_test[1]=180;vmc_all.sita_test[0]=0;}//

    if(vmc_all.param.leg_type==LOOP||vmc_all.param.leg_type==PARALE_LOOP)
        MAX_SPD=LIMIT(MAX_X/(0.4+1e-6)*1.5,0.01,MAX_FSPD);
    else
        MAX_SPD=LIMIT(vmc_all.l1*2/(vmc_all.gait_time[0]+1e-6),0.01,MAX_FSPD);

    for(i=0;i<4;i++){
        vmc[i].l1=vmc_all.l1;vmc[i].l2=vmc_all.l2;vmc[i].l3=vmc_all.l3;vmc[i].l4=vmc_all.l4;
        vmc[i].sita1=180;vmc[i].sita2=0;
        vmc[i].ground=0;
        vmc[i].param.trig_state=99;
    }
    //----------------------------------------------------------------------------------------
    for(i=0;i<4;i++)
    {
        robotwb.Leg[i].id=i;
        robotwb.Leg[i].limit_sita[0]=45;
        robotwb.Leg[i].limit_sita[1]=10;
        robotwb.Leg[i].limit_sita[2]=35;
        robotwb.Leg[i].limit_tao[0]=5;
        robotwb.Leg[i].limit_tao[1]=5;
        robotwb.Leg[i].limit_tao[2]=5;
        robotwb.Leg[i].is_ground=0;
        robotwb.Leg[i].delta_ht=0.03;
        robotwb.Leg[i].ground_state=0;
        robotwb.Leg[i].time_trig=0;
        robotwb.Leg[i].cnt_ss=0;
        robotwb.Leg[i].is_touch_est=0;
        vmc[i].param.trap_sw_t_fix=vmc[i].param.is_trap=0;
    }

    robotwb.gait_time=0.5;
    robotwb.stance_time=robotwb.gait_time/2;

    robotwb.Leg[0].flag_fb=robotwb.Leg[2].flag_fb=1;
    robotwb.Leg[1].flag_fb=robotwb.Leg[3].flag_fb=-1;
    robotwb.Leg[0].flag_rl=robotwb.Leg[1].flag_rl=1;
    robotwb.Leg[2].flag_rl=robotwb.Leg[3].flag_rl=-1;

    robotwb.MIN_Z=-(cosdw(40)*L2w-cosdw(60)*L1w);
    robotwb.MAX_Z=-(cosdw(25)*L1w+cosdw(25)*L2w);

    robotwb.MIN_X=-L1w*1.5;
    robotwb.MAX_X=L1w*1.5;

    robotwb.vect3_zero.x=robotwb.vect3_zero.y=robotwb.vect3_zero.z=0;

    gait_ww.state_gait=0;
    sdk.sdk_mode=0;
//参数
    robot_param_read();

    vmc_all.gait_time[0]=vmc_all.gait_time[1]=vmc_all.param.param_vmc_default.stance_time[0];//???????
    vmc_all.gait_alfa=0.5;
    vmc_all.stance_time=vmc_all.gait_time[1]*vmc_all.gait_alfa;
    vmc_all.gait_delay_time=vmc_all.stance_time*0;//?????????
}

void reset_tar_pos(char sel){//now set is degree
    int i;
    for(i=0;i<14;i++){//复位状态 for smooth
       leg_motor_all.q_set[i]=leg_motor_all.q_now[i];
    }
}

void vmc_reset(void)
{
     int i=0;
     vmc_all.param.soft_weight=1;
     vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
     slip_mode.slip_F[Xr]=slip_mode.slip_F[Yr]=slip_mode.slip_F[Zr]=0;
     slip_mode.slip_timer=0;
     for(i=0;i<4;i++)
    {
        vmc[i].spd.x=vmc[i].spd.y=vmc[i].spd.z=0;
        vmc[i].spd_b.x=vmc[i].spd_b.y=vmc[i].spd_b.z=0;
        vmc[i].spd_n.x=vmc[i].spd_n.y=vmc[i].spd_n.z=0;
        vmc[i].param.trig_state=vmc[i].ground=0;
    }
    vmc_all.ground_num=0;
    vmc_all.att_trig[0]=vmc_all.att_ctrl[0]=vmc_all.att[0];
    vmc_all.att_trig[1]=vmc_all.att_ctrl[1]=vmc_all.att[1];
    vmc_all.att_trig[2]=vmc_all.att_ctrl[2]=vmc_all.att[2];
    vmc_all.att_vm_b[PITr]=vmc_all.att_vm_b[ROLr]=vmc_all.att_vm_b[YAWr]=0;
    vmc_all.att_rate_vm[PITr]=vmc_all.att_rate_vm[ROLr]=vmc_all.att_rate_vm[YAWr]=0;
    vmc_all.att_rate_ctrl[PITr]=vmc_all.att_rate_ctrl[ROLr]=vmc_all.att_rate_ctrl[YAWr]=0;

    vmc_all.pos.z=0;
    vmc_all.pos_n.z=0;
    vmc_all.body_spd[Xr]=vmc_all.body_spd[Yr]=vmc_all.body_spd[Zr]=0;
    vmc_all.spd_n.x=vmc_all.spd_n.y=vmc_all.spd_n.z=0;
    vmc_all.param.encoder_spd[Ls]=vmc_all.param.encoder_spd[Rs]=0;
    vmc_all.body_spd[YAWrr]=vmc_all.att_ctrl[YAWr];
    vmc_all.param.cog_off_use[3]=0;
}

char safe_check(float dt)
{
    char i=0;
    static float timer_imu[3]={0};
    static float timer_q_safe[4]={0};
    dt=LIMIT(dt,0,0.0055);
    #if !TEST_TROT_SW
    //printf("safe pit:%f %f rol:%f %f\n",robotwb.exp_att.pitch,robotwb.now_att.pitch,robotwb.exp_att.roll,robotwb.now_att.roll);
    if(fabs(robotwb.exp_att.pitch-robotwb.now_att.pitch)>SAFE_PITCH){//&&stand_force_enable_flag[4]==1){
        //printf("rol %f %f\n",robotwb.exp_att.pitch,robotwb.now_att.pitch);
        timer_imu[0]+=dt;
    }
    else
        timer_imu[0]=0;

    if(fabs(robotwb.exp_att.roll-robotwb.now_att.roll)>SAFE_ROLL){//&&stand_force_enable_flag[4]==1){
        //printf("rol %f %f\n",robotwb.exp_att.roll,robotwb.now_att.roll);
        timer_imu[1]+=dt;
    }
    else
        timer_imu[1]=0;
    //printf("%f %f\n",robotwb.exp_att.roll,robotwb.now_att.roll);
    if((timer_imu[0]>SAFE_T||timer_imu[1]>SAFE_T)&&gait_ww.state_gait>=2){
        //printf("%f %f\n",timer_imu[0],timer_imu[1]);
        return 1;
    }
    #endif
#if 0
    for(i=0;i<4;i++){//∠角度保护1
        if(fabs(To_180_degreesw(vmc[i].sita1-vmc[i].sita2))<5&&stand_force_enable_flag[4]==1)
            timer_q_safe[i]+=dt;
        else
            timer_q_safe[i]=0;

        if(timer_q_safe[i]>2){
            timer_q_safe[i]=0;
        #if RUN_PI
            printf("ERR:: Q Closest Protect!!\n");
        #endif
            return 1;
        }
    }
#endif
 //Hardware check
    return 0;
}


#include <iostream>
#include <fstream>
#include <vector>
int isFirstWrite=0;
void appendFloatToFile(const std::vector<float>& floatNumbers, const std::string& filename) {
    // 以追加模式打开文件
    std::ofstream outFile(filename, std::ios::app);
    // 检查文件是否成功打开
    if (!outFile) {
        std::cerr << "无法打开文件进行写入" << std::endl;
        return;
    }

    // 将浮点数写入文件
    for (float num : floatNumbers) {
        outFile << num << " ";
    }
    outFile << std::endl;
    // 关闭文件
    outFile.close();

    //std::cout << "浮点数已成功追加写入文件" << std::endl;
}

int  readNextLine(const std::string& filename, int& currentLine, float floatNumbers[12]) {
    std::ifstream inFile(filename);

    // 检查文件是否成功打开
    if (!inFile) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return -1;
    }

    std::string line;
    int lineCount = 0;

    // 跳过前面的行
    while (lineCount < currentLine && std::getline(inFile, line)) {
        ++lineCount;
    }

    // 读取当前行
    if (std::getline(inFile, line)) {
        currentLine++;
        std::istringstream lineStream(line);
        std::string data;
        std::vector<float> rowData;

        while (lineStream >> data) {
            try {
                float value = std::stof(data);
                rowData.push_back(value);
            } catch (const std::invalid_argument& e) {
                std::cerr << "转换错误: 无效的浮点数 '" << data << "'" << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "转换错误: 浮点数超出范围 '" << data << "'" << std::endl;
            }
        }

        // 将数据分配给变量
        if (rowData.size() >= 14-1) {
            for(int i=0;i<14;i++)
                floatNumbers[i]= rowData[i];
            // 输出读取到的数据
//            std::cout << "a: " << a << ", b: " << b << ", c: " << c << ", d: " << d << ", e: " << e << std::endl;
        }
    } else {
        std::cerr << "没有更多的行可以读取" << std::endl;
        // 关闭文件
        inFile.close();
        return -1;
    }

    // 关闭文件
    inFile.close();
    return  1;
}

void servo_controller(float dt){
    static float timer_grasp=0;
    static int ocu_a_reg=0,ocu_ud_reg=0,ocu_key_reg=0,ocu_x_reg,ocu_back_reg=0,ocu_key_b_reg,ocu_key_x_reg;
    static int init=0;
    static int cnt_p=0;
    static END_POS arm_att_b_grasp_app_reg;
    static float put_dis=0;
    static float timer_put=0;
    static int lsm_record=0;
    static float pose_check[3]={0.225,-0.29,0.18};
    static int currentLine = 0;
    float dis_z=0,dis=0;
    static float timer_lsm=0;
    if(!init){
       init=1;
       lsm_record=0;
       isFirstWrite=0;
    }

    if((ocu.key_ud==-1&&!ocu_ud_reg)){
       printf("Stop record\n");
       lsm_record=0;
       timer_grasp=0;
       isFirstWrite=0;
       timer_lsm=0;
       leg_motor_all.servo_en=0;
    }

    std::vector<float> floatNumbers1 = {0,0,0 ,0,0,0 ,0,0,0 ,0,0,0 ,0,0};
    float floatNumbers[14];

    switch(lsm_record){
    case 0:
        if(ocu.key_a==1&&gait_ww.state_gait==0){
            timer_lsm+=dt;
        }else
            timer_lsm=0;
        if(timer_lsm>3){
            printf("Start record\n");
            lsm_record++;
        }
    break;
    case 1:
        // 持续写入浮点数到文件
        for(int i=0;i<14;i++)
            floatNumbers1[i]=leg_motor_all.q_servo[i];
         appendFloatToFile(floatNumbers1, "/home/odroid/Tinker/Motion/wave.txt");
        //printf("q_record=%.2f %.2f %.2f\n",spi_rx.q_servo[0],spi_rx.q_servo[1],spi_rx.q_servo[2]);
        if(ocu.key_b==1&&!ocu_key_b_reg){
            timer_grasp=0;
            lsm_record=0;
            timer_lsm=0;
            isFirstWrite=0;
            printf("Record motion finish\n");
        }
    break;
    }

#if !HEAD_USE_DC//Tinker
    if(leg_motor_all.servo_en==1){
        leg_motor_all.q_set_servo[0]=-35;

        DigitalLPF(ocu.rc_att_w[PITr]*40+20,&leg_motor_all.q_set_servo[1],0.9,dt);
        DigitalLPF(ocu.rc_att_w[ROLr]*55,&leg_motor_all.q_set_servo[2],0.8,dt);
    }
#endif
    switch(leg_motor_all.servo_replay_mode){
        case 0:
            if(ocu.key_st){//start grasping
                leg_motor_all.servo_replay_mode++;
                leg_motor_all.servo_en=1;
                currentLine=0;
                printf("Start Replay\n");
            }
        break;
        case 1://replay
           //printf("currentLine=%d\n",currentLine);
           if(readNextLine("/home/odroid/Tinker/Motion/wave.txt", currentLine, floatNumbers)==-1)
           {
               leg_motor_all.servo_replay_mode++;
               printf("Relay Done\n");
           }else{
               for(int i=0;i<14;i++)
                    if(floatNumbers[i]!=0)
                        leg_motor_all.q_set_servo[i]=floatNumbers[i];
           }
        break;
        case 2:
            leg_motor_all.servo_replay_mode=0;
        break;
        default:
            leg_motor_all.servo_replay_mode=0;
        break;
    }
    ocu_x_reg=ocu.key_x;
    ocu_key_reg=ocu.key_st;
    ocu_back_reg=ocu.key_back;
    ocu_a_reg=ocu.key_a;
    ocu_key_b_reg=ocu.key_b;
    ocu_ud_reg=ocu.key_ud;
}
//--------------------------------------------------主步态状态机------------------------------------------------
float qd_init[2]={  88, 88};//初始化角速度
float qd_reset[2]={-70,-70};
char sit_down_flag=0;
float tau_block_init=10.68;//3.2

float sdk_input_check=0;
int get_new_sdk_input=0;
void locomotion_sfm(float dt)
{
    int i,j;
    char robot_protect=0;
    static int test_state=0;
    static char temp_q_rst_flag=0,temp_err_rst_flag=0;
    char temp_flag[14]={0};
    static char q_init_done[14]={0};
    static float q_init_done_timer[14]={0};
    static int init=0;
    static char ocu_key_ud_reg;
    static float timer[10]={0};
    static char flip_flag=0;
    static float q_init_rotate_w[4][3]={1};
    static int motor_power_on=0;
    float tau_rotate[14]={0};

  if(!init){init=1;
        #if RUN_WEBOTS
        printf("VMC Param Initing!!!!!!!\n");
        #endif
        vmc_param_init();
  }

  if((fabs(sdk.cmd_vx)>0.01||fabs(sdk.cmd_vy)>0.01||fabs(sdk.cmd_vyaw)>0.01)&&sdk.sdk_mode==1){
      sdk_input_check+=dt;
      if(sdk_input_check>1)
      {
          get_new_sdk_input=1;
          sdk_input_check=0;
          printf("SDK::Input Updated!\n");
      }
  }else
      sdk_input_check=get_new_sdk_input=0;


  switch (gait_ww.state_gait)
  {
        case 0:
            switch(temp_q_rst_flag)//------cal motor
            {
                case 0:
                    motor_power_on=0;
                    //-------------------标定角度 按键Y
                    if(ocu.key_st)timer[1]+=dt;else timer[1]=0;
                    if(timer[1]>6)
                    {
                    #if RUN_WEBOTS
                        printf("Motor Zero Position delaying!!.....\n");
                    #endif
                    temp_q_rst_flag++;

                    robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                    timer[1]=0;
                    }
                break;
                case 1:
                    timer[1]+=dt;
                    if(timer[1]>4)
                    {
                    #if RUN_WEBOTS
                        printf("Motor Zero Position Save!!.....\n");
                    #endif
                    temp_q_rst_flag++;

                    leg_motor_all.reset_q=2;
                    robotwb.beep_state=BEEP_BLDC_GAIT_SWITCH;
                    timer[1]=0;
                    }
                break;
                case 2://
                    if(!ocu.key_st)timer[1]+=dt;

                    if(timer[1]>0.5)
                    {
                        temp_q_rst_flag=0;
                        leg_motor_all.reset_q=0;
                        timer[1]=0;
                    }
                break;
            }	//----

            //-------------------复位故障
            switch(temp_err_rst_flag)//按键B
            {
            case 0:
                if(ocu.key_b)timer[2]+=dt;else timer[2]=0;

                if(timer[2]>1)
                {
                    #if RUN_WEBOTS
                    printf("Reset Motor Error.....\n");
                    #endif
                    temp_err_rst_flag++;
                    robotwb.beep_state=BEEP_BLDC_RESET_ERR;
                    leg_motor_all.motor_en=0;
                    timer[2]=0;
                    leg_motor_all.reset_err=1;
                }
            break;
            case 1:
                if(!ocu.key_b)timer[2]+=dt;

                if(timer[2]>0.5)
                {
                    temp_err_rst_flag=0;
                    leg_motor_all.reset_err=0;//reset_err_flag=0;
                    timer[2]=0;
                }
            break;
            }//-----------

            if(ocu.key_x||get_new_sdk_input)timer[0]+=dt;else timer[0]=0;
            if(timer[0]>1.0)
            {
                #if RUN_WEBOTS
                printf("Motor Initing.....\n");
                #endif
                leg_motor_all.motor_en=1;//开始能
                leg_motor_all.servo_en=1;
                motor_power_on=1;
                timer[0]=timer[1]=0;
                vmc_all.param.soft_weight=1;
                vmc_all.gait_mode=IDLE;vmc_all.power_state=2;

                gait_ww.state_gait++;
                sit_down_flag=0;//进站立标志位
                robotwb.beep_state=BEEP_BLDC_ZERO_INIT;
                reset_tar_pos(0);
                for(i=0;i<14;i++){//复位状态 for smooth
                   leg_motor_all.stiff[i]=leg_motor_all.stiff_init;
                }
            }

    break;

    case 1:
      timer[0]+=dt;
      if(timer[0]>0.1){
          for(i=0;i<14;i++){
              leg_motor_all.q_set[i]=move_joint_to_pos_all(leg_motor_all.q_set[i],leg_motor_all.q_init[i],5+timer[0]*40,dt);//doghome
          }
          for(i=0;i<14;i++){
              leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,dt);//doghome
          }
      }else{
          for(i=0;i<14;i++){//复位状态 for smooth
             leg_motor_all.q_set[i]=leg_motor_all.q_now[i];
          }
      }

      if(timer[0]>3.5&&state_pass&&1){//可强制结束 Lock now angle
              #if RUN_WEBOTS
              printf("Motor Initing Done All!!\n");
              #endif
              gait_ww.state_gait++;

              reset_tar_pos(0);
              for(i=0;i<14;i++){//复位状态 for smooth
                 leg_motor_all.stiff[i]=leg_motor_all.stiff_init;
              }
      }
  break;

  case 2://步态线程------------------------<<
    gait_switch(dt);//步态切换状态机
    switch(vmc_all.gait_mode){
        case STAND_RC:
            if(test_pos_st)//姿态测试
            {
                timer[3] += test_st_exp[4] / 2.0*dt;

                vmc_all.tar_pos.x = cosdw(timer[3] * 180)*test_st_exp[2] + vmc_robot_p.stand_off.x;
                //vmc_all.tar_pos.z = (fabs(MAX_Z)-fabs(MIN_Z))*0.8 - sindw(timer[3] * 180)*test_st_exp[3];

                vmc_all.tar_att[PITr] = sindw(timer[3] * 180) *  test_st_exp[0];
                vmc_all.tar_att[ROLr] =  cosdw(timer[3] * 180) *  test_st_exp[1];
            }else{
                vmc_all.tar_pos.x=ocu.rc_spd_w[Xr]*0.05+vmc_robot_p.stand_off.x;
                timer[3]=0;
            }
            Gait_Stand_Update(dt);
        break;
        case G_RL:
            Gait_RL_Update(dt);
        break;
        case RECOVER:
            Gait_Recovery_Update(dt);
        break;
    default:
    break;
    }
  break;
  }

#if !RUN_WEBOTS||RUN_PI

    robot_protect=safe_check(dt);
    //printf("robot_protect=%d\n",robot_protect);
    if(robot_protect==1&&gait_ww.state_gait>=2){
        vmc_all.gait_mode=SOFT;
        vmc_all.param.robot_mode=M_SOFT;
        vmc_all.falling[0]=1;
        vmc_all.falling[1]=0;//timer
        ocu.cmd_robot_state=99;

        leg_motor_all.servo_en=0;
        leg_motor_all.motor_en=0;
        for(i=0;i<14;i++){//复位状态 for smooth
           leg_motor_all.q_set[i]=leg_motor_all.q_now[i];
        }
        gait_ww.state_gait=0;//reset main state
        printf("Soft-Mode Interupted::Falling!!! robot_protect=%d\n",robot_protect);
    }
    if(robot_protect||(ocu.key_ud<=-1&&ocu_key_ud_reg==0)||(ocu.sbus_power_off&&gait_ww.state_gait<=2)){//Disable Power 断使能
       // vmc_all.tar_spd.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=0;
       // vmc_all.param.tar_spd_use_rc.x=vmc_all.param.tar_spd_use_rc.y=vmc_all.param.tar_spd_use_rc.z=0;
        vmc_all.power_state=ocu.cmd_robot_state=vmc_all.leg_power=0;
        vmc_all.gait_mode=IDLE;
        vmc_all.param.robot_mode=M_SAFE;
        leg_motor_all.motor_en=0;
        leg_motor_all.servo_en=0;
        #if RUN_WEBOTS
        if(gait_ww.state_gait!=0)
            printf("Safety Power Off !!!!!!!\n");
        #endif
        //doghome
        for(i=0;i<4;i++){
#if !RUN_WEBOTS||RUN_PI
            reset_current_cmd(i);
#endif
            leg_motor[i].motor_en=0;
        }
        for(i=0;i<14;i++){//复位状态 for smooth
           leg_motor_all.q_set[i]=leg_motor_all.q_now[i];
        }
        gait_ww.state_gait=0;//reset main state
        //reset sdk
        sdk_input_check=get_new_sdk_input=0;
        if(ocu.key_ud==-1)
            sdk.sdk_mode=0;
    }
    ocu_key_ud_reg=ocu.key_ud;
#endif

}


float att_trig_reset_dead=2.56;
float soft_param[2]={0.75,0.6};
float z_init_spd=2;//站立初始化速度
float init_end_z=0;
char gait_switch(float dt)//moshi Remote 主步态切换状态机--------------------------------------------------------
{
    char i;
    static float timer_webot_auto = 0;
    static char param_change=0;
    static char sdk_mode_reg=0;
    static char stand_switch_trig=0;
    static float t,t_rst[3],timer[5],t_fall,t_fall_self_right=0;
    static char state,rst_state,soft_start_state,fall_state,t_um;
    static float sbus_mode_sw_cnt=0;
    static float rolling_check_timer=0;
    static float couch_z=0;
    char auto_switch=0;
    float cog_off,att_off;
    float end_dis[4];
    float att_use[3],err[2];
    float tar_acc_cog[3]={0};
    static float tar_spd_reg[3]={0};
    static int swing_height_sel=0;
    static float timer_trot_protect=0;
    static float timer_auto_switch=0;
    static float auto_sit_down_t=0;
    static char  auto_sit_down=0;
    float z_err=0;

    float check_tau_knee=leg_motor_all.t_now[10];
    float check_tau_ankle=leg_motor_all.t_now[11];
    static int kick_lsm=0;
    static float kick_timer=0;

    dt=LIMIT(dt,0,0.0055);
    att_use[ROLr]=vmc_all.att_ctrl[ROLr];
    err[ROLr]=vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr];

    tar_acc_cog[Xr]=(vmc_all.tar_spd.x-tar_spd_reg[Xr])/dt;
    tar_acc_cog[Yr]=(vmc_all.tar_spd.y-tar_spd_reg[Yr])/dt;
    tar_acc_cog[Zr]=(vmc_all.tar_spd.z-tar_spd_reg[Zr])/dt;

    tar_spd_reg[Xr]=vmc_all.tar_spd.x;
    tar_spd_reg[Yr]=vmc_all.tar_spd.y;
    tar_spd_reg[Zr]=vmc_all.tar_spd.z;
//---------------------------OCU 模式锟叫伙拷-------------------------------
    if(((ocu.connect&&ocu.mode>=2)||sdk.sdk_mode==1||ocu.esp32_connect)||(RUN_WEBOTS&&!RUN_PI)){//锟斤拷通锟街憋拷

            switch(ocu.cmd_robot_state)//moshi
            {
            case 0://锟斤拷全模式
                vmc_all.param.robot_mode=M_SAFE;
                vmc_reset();//
                ocu.cmd_robot_state=1;
                vmc_all.gait_force_fast=0;
                t_fall_self_right=0;
                param_change=0;
                vmc_all.leg_power=vmc_all.unmove=1;//锟较碉拷
                vmc_all.param.stand_switch_flag[0]=vmc_all.param.stand_switch_flag[1]=1;//锟较碉拷锟斤拷锟斤拷锟?					vmc_all.param.stand_switch_cnt[0]=vmc_all.param.stand_switch_cnt[1]=0;
                vmc_all.param.stand_trot_switch_flag=0;
                vmc_all.reset_trot_flag=0;
                vmc_all.t_reset_trot_flag=0;
                stand_switch_trig=0;
                rolling_check_timer=0;
                swing_height_sel=0;
                printf("Gait Machine Active!\n");
            break;
            case 1://
                vmc_all.unmove=1;
                for(i=0;i<4;i++){
                    vmc[i].ground=0;
                }

                if(1||(RUN_WEBOTS&&!RUN_PI)||(ocu.key_ll&&ocu.key_ll_reg==0)||(ocu.key_rr&&ocu.key_rr_reg==0)||(get_new_sdk_input==1&&sdk.sdk_mode==1)){//键切换模式 doghome
                    vmc_all.power_state=2;
                    ocu.cmd_robot_state=2;

                    reset_tar_pos(0);//
                    vmc_all.param.stand_switch_flag[0]=0;//
                    vmc_all.param.stand_switch_flag[1]=0;//

                    vmc_all.param.robot_mode=M_STAND_RC;vmc_all.gait_mode=STAND_RC;
                    Gait_Stand_Active();

                    auto_sit_down_t=0;
                    auto_sit_down=0;
                    init_end_z=vmc_all.pos_n.z;
                    couch_z=vmc_all.tar_pos.z=robotwb.exp_pos_n.z=init_end_z;
                    printf("ST height=%f\n",init_end_z);
                }
            break;
            case 2://Stand mode-------------------------------
                //printf("vmc_all.param.soft_weight=%f\n",vmc_all.param.soft_weight);
                cog_off=vmc_all.param.cog_off_use[0];//
                att_off=vmc_all.param.cog_off_use[1];//
                if((init_end_z>0.6*fabs(MAX_Z)||auto_sit_down==1)&&ocu.key_x==0)
                    auto_sit_down_t+=dt;
                if(ocu.key_ll&&ocu.key_ll_reg==0||get_new_sdk_input==1){//下蹬腿
                    if(get_new_sdk_input)
                        get_new_sdk_input=0;

                    if(stand_force_enable_flag[4]==0)//空中竟可能下探
                        init_end_z=fabs(MAX_Z)*config_gait["vmc_param"]["stand_z"].as<float>();
                    else														 //着地步态高度
                        init_end_z=fabs(MAX_Z)*config_gait["vmc_param"]["stand_z"].as<float>();
                    sit_down_flag=1;//下蹬时才触发ST着地判断
                    auto_sit_down=auto_sit_down_t=0;
                }

                if((ocu.key_rr&&ocu.key_rr_reg==0&&vmc_all.reset_trot_flag==0)||auto_sit_down_t>20){
                    sit_down_flag=0;
                    if(ocu.key_rr)
                        auto_sit_down=auto_sit_down_t=0;

                    init_end_z= couch_z/2;//fabs(MIN_Z)*1.1;

                    if(auto_sit_down_t>20&&!auto_sit_down){
                        auto_sit_down=1;
                        auto_sit_down_t=0;
                        printf("Warning:No RC Auto Sit Down!\n");
                    }
                }

                if(auto_sit_down==1&&auto_sit_down_t>5)
                {
                    vmc_all.gait_mode=SOFT;
                    vmc_all.param.robot_mode=M_SOFT;
                    ocu.cmd_robot_state=99;
                    auto_sit_down=auto_sit_down_t=0;
                    printf("Warning:Auto Power Off!\n");
                }

                if(fabs(vmc_all.tar_att[PITr])>1||fabs(vmc_all.tar_att[ROLr])>1)
                    auto_sit_down=auto_sit_down_t=0;

                if(gait_ww.switch_timer[0]>0.15){
                    auto_switch=1;
                    vmc_all.tar_spd.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=0;//reset FLT reselt
                    gait_ww.switch_timer[0]=0;
                }else
                    auto_switch=0;

                vmc_all.param.tar_spd_use_rc.x=vmc_all.param.tar_spd_use_rc.y=vmc_all.param.tar_spd_use_rc.z=0;
    #if AUTO_SWITCH
                if(gait_ww.auto_switch==1){
                    if(fabs(vmc_all.tar_spd.x)>0.02||fabs(vmc_all.tar_spd.y)>0.02||fabs(vmc_all.tar_spd.z)>0.1)
                        timer_auto_switch+=dt;
                    else
                        timer_auto_switch=0;
                }
    #endif
                if (((ocu.key_x == 1 && ocu.key_x_reg == 0)||timer_auto_switch>0.15)&&(vmc_all.rl_connect==1||RL_USE_TVM))//Key-X -> RL
                {
                     ocu.cmd_robot_state = 18;
                     Gait_RL_Active(1);
                     auto_sit_down=auto_sit_down_t=0;
                     kick_lsm=0;
                     kick_timer=0;
                     timer_auto_switch=0;
                }
            break;

            case 18://RL Locomotion-------------------------------
                vmc_all.rl_commond_rl_rst[0]+=(0-vmc_all.rl_commond_rl_rst[0])*dt*3.0;
                vmc_all.net_run_dt+=(config_gait["rl_gait"]["net_run_dt"].as<float>()-vmc_all.net_run_dt)*dt*1.5;
                //printf("vmc_all.rl_commond_rl_rst[0]=%f vmc_all.net_run_dt=%f\n",vmc_all.rl_commond_rl_rst[0],vmc_all.net_run_dt);
     #if AUTO_SWITCH
                if(gait_ww.auto_switch==1){
                    if(vmc_all.rl_mode_used==1){//tort
                        if(fabs(vmc_all.tar_spd.x)<0.02&&fabs(vmc_all.tar_spd.y)<0.02&&fabs(vmc_all.tar_spd.z)<0.1)
                            timer_auto_switch+=dt;
                        else
                            timer_auto_switch=0;
                    }else if(vmc_all.rl_mode_used==2){//stand
                        if(fabs(vmc_all.tar_spd.x)>0.02||fabs(vmc_all.tar_spd.y)>0.02||fabs(vmc_all.tar_spd.z)>0.1)
                            timer_auto_switch+=dt;
                        else
                            timer_auto_switch=0;
                    }
                }
     #endif
                if (ocu.key_ll)//
                    vmc_all.tar_pos.z += dt * 0.05;
                if (ocu.key_rr)
                    vmc_all.tar_pos.z -= dt * 0.05;

                if (sdk.sdk_mode == 1) {
                    if (sdk.cmd_z != 0)
                        vmc_all.tar_pos.z = sdk.cmd_z;
                    else
                        vmc_all.tar_pos.z += dt * sdk.cmd_vz;
                }

                vmc_all.tar_pos.z = LIMIT(vmc_all.tar_pos.z, fabs(MIN_Z)*1.2, fabs(MAX_Z)*0.9);

                if (((ocu.key_b == 1 && ocu.key_b_reg == 0) ||
                    (timer_auto_switch > 0.35 && vmc_all.rl_mode_used==1) ||
                    (sdk.sdk_mode == 1 && sdk.gait_mode == STAND_RC)) && vmc_all.rl_mode_used==1)
                {
#if 0
                    stand_switch_trig = 0;
                    ocu.cmd_robot_state = 2;
                    vmc_all.param.robot_mode = M_STAND_RC;	vmc_all.gait_mode = STAND_RC;
                    Gait_Stand_Active();//

                    for(i=0;i<14;i++){
                        leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,dt);//doghome
                    }
#else
                    stand_switch_trig = 0;
                    timer_auto_switch=0;
                    Gait_RL_Active(2);//RL stand
#endif
                }

                if (((ocu.key_x == 1 && ocu.key_x_reg == 0) ||
                    (timer_auto_switch > 0.15 && vmc_all.rl_mode_used==2)||
                    (sdk.sdk_mode == 1 && sdk.gait_mode == G_RL)) && vmc_all.rl_mode_used==2)
                {
                    timer_auto_switch=0;
                    Gait_RL_Active(1);//RL trot
                }

                check_tau_knee=leg_motor_all.t_now[10];//right
                check_tau_ankle=leg_motor_all.t_now[11];
               // printf("check_tau_knee=%f check_tau_ankle=%f\n",check_tau_knee,check_tau_ankle);
                if((ocu.key_a != 0 && ocu.key_a_reg == 0)){
                    kick_lsm++;
                    kick_timer=0;
                }
                else if(kick_lsm==1){//check stand
                    if(fabs(check_tau_knee)<0.15)//&&fabs(check_tau_ankle)<0.15)//lift ground
                    {
                        kick_lsm++;
                        kick_timer=0;
                    }
                }else if(kick_lsm==2){//delay
                    kick_timer+=dt;
                    if(kick_timer>0.05)
                    {
                        kick_lsm++;
                        kick_timer=0;
                    }
                }else if(kick_lsm==3)//kick ball
                {
                    printf("kick_ball\n");
                    for(i=0;i<14;i++){
                        leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,dt);//doghome
                    }
                    ocu.cmd_robot_state=20;
                    vmc_all.param.robot_mode = M_KICK;	vmc_all.gait_mode = G_KICK;
                    timer[0]=0;
                    kick_lsm=0;
                    kick_timer=0;
                }

                if (fabs(vmc_all.att_ctrl[ROLr]) > SAFE_ROLL)
                    t_fall_self_right += dt;
                else
                    t_fall_self_right = 0;

                if (t_fall_self_right > 0.05 && 0) {
                    Gait_Recovery_Active();
                    ocu.cmd_robot_state = 12;
                }
               break;
            case 20://kick ball move
                 timer[0]+=dt;

                 for(i=0;i<7;i++){
                     leg_motor_all.q_set[i]=
                             move_joint_to_pos_all(leg_motor_all.q_set[i],leg_motor_all.q_init1[i],1500,dt);//doghome
                     leg_motor_all.stiff[i]=3.0;
                 }

                 for(i=7;i<14;i++){
                     leg_motor_all.q_set[i]=move_joint_to_pos_all(leg_motor_all.q_set[i],leg_motor_all.q_init1[i],1500,dt);
                     leg_motor_all.stiff[i]=3.0;
                 }

                 if (timer[0]>0.05)
                 {
                     printf("kick done move\n");
#if 0
                     stand_switch_trig = 0;
                     ocu.cmd_robot_state = 2;
                     vmc_all.param.robot_mode = M_STAND_RC;	vmc_all.gait_mode = STAND_RC;
                     Gait_Stand_Active();

                     for(i=0;i<14;i++){
                         leg_motor_all.q_set[i]=leg_motor_all.q_init[i];//doghome
                         leg_motor_all.stiff[i]=3.5;
                     }
                     for(i=0;i<14;i++){
                         leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,dt);//doghome
                     }
#else
                     for(i=0;i<14;i++){
                         leg_motor_all.stiff[i]=1;
                     }
                     ocu.cmd_robot_state = 18;
                     Gait_RL_Active(0);
                     auto_sit_down=auto_sit_down_t=0;
#endif
                 }
            break;
           //-------------------------
           case 12://自恢复
              if(ocu.key_lr==-1&&ocu.key_lr_reg==0)
              {
                  if(vmc_all.param.leg_dof==3){
                  vmc_all.param.robot_mode=M_RECOVER;//
                  vmc_all.gait_mode=RECOVER;
                  }
                  else{
                  vmc_reset();
                  ocu.cmd_robot_state=1;
                  t_fall_self_right=0;
                  vmc_all.leg_power=vmc_all.unmove=1;//
                  vmc_all.param.stand_switch_flag[0]=vmc_all.param.stand_switch_flag[1]=1;//
                  vmc_all.param.stand_trot_switch_flag=0;
                  stand_switch_trig=0;
                  }
              }
            break;
            //-------------------soft  Key-Y
            case 99:
                if(vmc_all.param.soft_weight<0.5)
                    vmc_all.param.soft_weight-=soft_weight[1]*dt;
                else
                    vmc_all.param.soft_weight-=soft_weight[0]*dt;

                if(vmc_all.param.soft_weight<0.02){
                    printf("Auto-Safe!!!\n");
                    ocu.key_ud=-2;
                }
                //printf("vmc_all.param.soft_weight=%f\n",vmc_all.param.soft_weight);
                vmc_all.param.soft_weight=limitw(vmc_all.param.soft_weight,0.0,0.75);
            break;
        }

        if(ocu.key_y==1&&ocu.key_y_reg==0){//low-level
            vmc_all.gait_mode=SOFT;
            vmc_all.param.robot_mode=M_SOFT;
            ocu.cmd_robot_state=99;
            printf("Soft-Mode Interupted!!!\n");
        }

        if(((ocu.key_ud==-1&&ocu.key_ud_reg==0))&&vmc_all.sita_test[4]==0){//Disable Power high-level
            vmc_all.tar_spd.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=0;
            vmc_all.param.tar_spd_use_rc.x=vmc_all.param.tar_spd_use_rc.y=vmc_all.param.tar_spd_use_rc.z=0;
            vmc_all.power_state=ocu.cmd_robot_state=vmc_all.leg_power=0;
            vmc_all.gait_mode=IDLE;
            vmc_all.param.robot_mode=M_SAFE;
            vmc_all.ground_per_trot=0;
            vmc_all.ground_per_trot_timer=0;
            sit_down_flag=0;//进站立标志位
            stand_force_enable_flag[0]=0;stand_force_enable_flag[1]=0;
            stand_force_enable_flag[2]=0;stand_force_enable_flag[3]=0;
            stand_force_enable_flag[4]=0;
            init_end_z=fabs(MAX_Z)*0.36;
            vmc_all.param.soft_weight=1;

            reset_tar_pos(0);
            gait_ww.state_gait=0;//reset main state
            printf("Safe-Mode Interupted!!!\n");
        }

//================================================
        sdk_mode_reg=sdk.sdk_mode;ocu.key_ud_reg=ocu.key_ud;ocu.key_lr_reg=ocu.key_lr;ocu.key_x_reg =ocu.key_x;ocu.key_y_reg =ocu.key_y;ocu.key_a_reg=ocu.key_a;
        ocu.key_b_reg=ocu.key_b;ocu.key_ll_reg=ocu.key_ll;ocu.key_rr_reg=ocu.key_rr;ocu.key_st_reg=ocu.key_st;ocu.key_back_reg=ocu.key_back;
    }
    return vmc_all.leg_power;
}

