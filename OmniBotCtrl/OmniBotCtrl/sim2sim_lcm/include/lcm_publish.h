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
class RL_Tinymal {
public:
    std::string model_path;
    int init_policy();
    int load_policy();
    torch::Tensor model_infer(torch::Tensor policy_input);
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const  my_lcm::Request *request);//获取机器人反馈

    float Kp = 3.0;
    float Kd = 0.15;
    float infer_dt = 0.02;//50Hz

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
    float init_pos[12] = {-0.16,0.68,1.3 ,0.16,0.68,1.3, -0.16,0.68,1.3, 0.16,0.68,1.3};
    float eu_ang_scale= 1;
    float omega_scale=  0.25;
    float pos_scale =   1.0;
    float vel_scale =   0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;
    float action_scale[12] = {0.25,0.25,0.25, 0.25,0.25,0.25, 0.25,0.25,0.25, 0.25,0.25,0.25};
    float action_delta_max = 1.0;
    float action_delta_min = -1.0;

    torch::jit::script::Module model;
    torch::DeviceType device;
private:

   
};

