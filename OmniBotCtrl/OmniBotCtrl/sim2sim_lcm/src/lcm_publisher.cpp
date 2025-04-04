
#include "lcm_publish.h"
#include <iostream>
#include <valarray>

using namespace std;
using namespace torch::indexing; // 确保使用了正确的命名空间
RL_Tinymal tinymal_rl;

void RL_Tinymal::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const  my_lcm::Request *request)//获取机器人反馈
{              
    // #单次观测
    // obs[0, 0] = omega[0] *cfg.normalization.obs_scales.ang_vel
    // obs[0, 1] = omega[1] *cfg.normalization.obs_scales.ang_vel
    // obs[0, 2] = omega[2] *cfg.normalization.obs_scales.ang_vel
    // obs[0, 3] = eu_ang[0] *cfg.normalization.obs_scales.quat
    // obs[0, 4] = eu_ang[1] *cfg.normalization.obs_scales.quat
    // obs[0, 5] = eu_ang[2] *cfg.normalization.obs_scales.quat
    // obs[0, 6] = cmd.vx * cfg.normalization.obs_scales.lin_vel
    // obs[0, 7] = cmd.vy * cfg.normalization.obs_scales.lin_vel
    // obs[0, 8] = cmd.dyaw * cfg.normalization.obs_scales.ang_vel
    // obs[0, 9:21] = (q-default_dof_pos) * cfg.normalization.obs_scales.dof_pos #g关节角度顺序依据修改为样机
    // obs[0, 21:33] = dq * cfg.normalization.obs_scales.dof_vel
    // obs[0, 33:45] = last_actions#上次控制指令

    // 将 data 转为 tensor 类型，输入到模型
    std::vector<float> obs;
    //---------------Push data into obsbuf--------------------
    obs.push_back(request->omega[0]*omega_scale);
    obs.push_back(request->omega[1]*omega_scale);
    obs.push_back(request->omega[2]*omega_scale);

    obs.push_back(request->eu_ang[0]*eu_ang_scale);
    obs.push_back(request->eu_ang[1]*eu_ang_scale);
    obs.push_back(request->eu_ang[2]*eu_ang_scale);

    // cmd
    float max = 1.0;
    float min = -1.0;

    cmd_x = cmd_x * (1 - smooth) + (std::fabs(request->command[0]) < dead_zone ? 0.0 : request->command[0]) * smooth;
    cmd_y = cmd_y * (1 - smooth) + (std::fabs(request->command[1]) < dead_zone ? 0.0 : request->command[1]) * smooth;
    cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request->command[2]) < dead_zone ? 0.0 : request->command[2]) * smooth;

    obs.push_back(cmd_x*lin_vel);//控制指令x
    obs.push_back(cmd_y*lin_vel);//控制指令y
    obs.push_back(cmd_rate*ang_vel);//控制指令yaw rate

    // pos q joint
    for (int i = 0; i < 12; ++i)
    {
        float pos = (request->q[i]  - init_pos[i])* pos_scale;
        obs.push_back(pos);
    }
    // vel q joint
    for (int i = 0; i < 12; ++i)
    {
        float vel = request->dq[i] * vel_scale;
        obs.push_back(vel);
    }
    // last action
    for (int i = 0; i < 12; ++i)
    {
        obs.push_back(action_temp[i]);// 
    }
    // std::cout<<("----------------obs---------------")<<std::endl;
    // cout<<obs<<endl;
    // std::cout<<("--------------------------------")<<std::endl;

    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    torch::Tensor obs_tensor = torch::from_blob(obs.data(),{1,45},options).to(device);
    //----------------------------------------------------------------
    auto obs_buf_batch = obs_buf.unsqueeze(0);

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_tensor.to(torch::kHalf));
    inputs.push_back(obs_buf_batch.to(torch::kHalf));
    
    //---------------------------网络推理----------------------------- Execute the model and turn its output into a tensor
    //cout<<"obs_tensor1:"<<endl<<obs_tensor<<endl;
    //std::cout<<("*****************")<<std::endl;
    //cout<<"obs_buf_batch:"<<endl<<obs_buf_batch<<endl;
    torch::Tensor action_tensor = model.forward(inputs).toTensor();
    action_buf = torch::cat({action_buf.index({ Slice(1,None),Slice()}),action_tensor},0);
    //cout<<"[action out]:"<<endl<<action_tensor<<endl;
    bool has_nan = false;
    for (float val : obs) {
        //cout << val << " ";
        if (std::isnan(val)) {
            has_nan = true;
        }
    }
    if (has_nan) {
        cout << "NaN detected in obs. Press any key to continue..." << endl;
        getchar(); // 等待键盘输入
    }

    //-----------------------------网络输出滤波--------------------------------
    torch::Tensor action_blend_tensor = 0.8*action_tensor + 0.2*last_action;
    last_action = action_tensor.clone();
 
    this->obs_buf = torch::cat({this->obs_buf.index({Slice(1, None), Slice()}), obs_tensor}, 0); // 历史观测移位
    // //obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);//历史观测移位
    // //----------------------------------------------------------------
    torch::Tensor action_raw = action_blend_tensor.squeeze(0);
    // move to cpu
    action_raw = action_raw.to(torch::kFloat32);
    action_raw = action_raw.to(torch::kCPU);
    // // assess the result
    auto action_getter = action_raw.accessor <float,1>();//bug
    for (int j = 0; j < 12; j++)
    {
        action[j] = action_getter[j]; 
        action_temp[j] = action_getter[j];//原始值
    }
     
    action_refresh=1;
}


int RL_Tinymal::load_policy()
{   
    std::cout << model_path << std::endl;
    // load model from check point
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device= torch::kCPU;
    if (torch::cuda::is_available()&&1){
        device = torch::kCUDA;
    }
    std::cout<<"device:"<<device<<endl;
    model = torch::jit::load(model_path);
    std::cout << "load model is successed!" << std::endl;
    model.to(device);
    std::cout << "LibTorch Version: " << TORCH_VERSION_MAJOR << "." 
              << TORCH_VERSION_MINOR << "." 
              << TORCH_VERSION_PATCH << std::endl;
    model.to(torch::kHalf);
    std::cout << "load model to device!" << std::endl;
    model.eval();
}
 
int RL_Tinymal::init_policy(){
 // load policy
    std::cout << "RL model thread start"<<endl;
    cout <<"cuda_is_available:"<< torch::cuda::is_available() << endl;
    cout <<"cudnn_is_available:"<< torch::cuda::cudnn_is_available() << endl;
    
    model_path = "/home/pi/Downloads/LocomotionWithNP3O-master/model_jitt.pt";//载入jit模型
    load_policy();

 // initialize record
    action_buf = torch::zeros({history_length,12},device);
    obs_buf = torch::zeros({history_length,45}, device);//历史观测
    last_action = torch::zeros({1,12},device);

    action_buf.to(torch::kHalf);
    obs_buf.to(torch::kHalf);
    last_action.to(torch::kHalf);

    for (int j = 0; j < 12; j++)
    {
        action_temp.push_back(0.0);
	    action.push_back(init_pos[j]);
        prev_action.push_back(init_pos[j]);
    }
    //hot start
    for (int i = 0; i < history_length; i++)//为历史观测初始化
    {
        // 将 data 转为 tensor 类型，输入到模型
        std::vector<float> obs;
        //---------------Push data into obsbuf--------------------
        obs.push_back(0);//request->omega[0]*omega_scale);
        obs.push_back(0);//request->omega[1]*omega_scale);
        obs.push_back(0);//request->omega[2]*omega_scale);

        obs.push_back(0);//request->eu_ang[0]*eu_ang_scale);
        obs.push_back(0);//request->eu_ang[1]*eu_ang_scale);
        obs.push_back(0);//request->eu_ang[2]*eu_ang_scale);

        // cmd
        obs.push_back(0);//控制指令x
        obs.push_back(0);//控制指令y
        obs.push_back(0);//控制指令yaw rate

        // pos q joint
        for (int i = 0; i < 12; ++i)
        {
            float pos = 0;//(request->q[i]  - init_pos[i])* pos_scale;
            obs.push_back(pos);
            action[i]=init_pos[i];
        }
        // vel q joint
        for (int i = 0; i < 12; ++i)
        {
            float vel = 0;//request->dq[i] * vel_scale;
            obs.push_back(vel);
        }
        // last action
        for (int i = 0; i < 12; ++i)
        {
            obs.push_back(0);//历史  self.cfg.env.history_len, self.num_dofs
        }
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor obs_tensor = torch::from_blob(obs.data(),{1,45},options).to(device);
    }

    for (int i = 0; i < 200; i++)
    {
        //tinymal_rl.model_infer();
    }
}

int main(int argc, char** argv) {
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "LCM initialization failed" << std::endl;
        return 1;
    }
    tinymal_rl.init_policy();
 
    lcm.subscribe("LCM_OBS", &RL_Tinymal::handleMessage, &tinymal_rl);
    while (1){
        my_lcm::Response msg;
        for(int i=0;i<12;i++)
            msg.q_exp[i]=tinymal_rl.action[i];
        
        lcm.publish("LCM_ACTION", &msg);
        //std::cout << "Message robot state published!" << send_cnd++<<std::endl;
        lcm.handle();
    }
 
    return 0;
}

