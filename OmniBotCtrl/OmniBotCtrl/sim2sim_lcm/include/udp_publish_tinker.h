#include <iostream>
#include <torch/torch.h>
#include <torch/script.h>
#include <mutex>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include "Request.hpp"
#include "Response.hpp"

#include "enumClass.h"
#include "mathTools.h"
#include "mathTypes.h" 
#include <iostream>
#include "stdio.h"

struct _msg_request
{
    float trigger;
    float command[4];
    float eu_ang[3];
    float omega[3];
    float acc[3];
    float q[10];
    float dq[10];
    float tau[10];
    float init_pos[10];
};

struct _msg_response
{
    float q_exp[10];
    float dq_exp[10];
    float tau_exp[10];
};

class RL_Tinymal_UDP {
public:
    std::string model_path;
    int init_policy();
    int load_policy();
    torch::Tensor model_infer(torch::Tensor policy_input);
    void handleMessage(_msg_request request);//获取机器人反馈

    //gamepad
    float smooth = 0.03;
    float dead_zone = 0.01;

    float cmd_x = 0.;
    float cmd_y = 0.;
    float cmd_rate = 0.;

    std::vector<float> action;
    std::vector<float> action_temp;
    std::vector<float> prev_action;

    torch::Tensor action_buf;
    torch::Tensor obs_buf;
    torch::Tensor last_action;

    // default values
    int action_refresh=0;
    int history_length = 10;
    //float init_pos[10] = {0.0,-0.07,0.628,-1.16,0.565,  0.0,0.07,0.628,-1.16,0.565};//important
    float init_pos[10] = {0.0,-0.07,0.57,-1.12,0.56,  0.0,0.07,0.57,-1.12,0.56};//important
    float eu_ang_scale= 1;
    float omega_scale=  0.25;
    float pos_scale =   1.0;
    float vel_scale =   0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;
    torch::jit::script::Module model;
    torch::DeviceType device;
private:

   
};

