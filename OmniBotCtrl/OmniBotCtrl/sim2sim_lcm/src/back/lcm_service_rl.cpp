#include "lcm_service.h"
#include "stdio.h"
#include <torch/torch.h>
using namespace torch::indexing; // 确保使用了正确的命名空间

using namespace std;

void Handler::handleMessage(const lcm::ReceiveBuffer* rbuf,//50Hz pubulish
                    const std::string& chan,
                    const my_lcm::Request* request) {
                        
    //cout<<"Get robot fb:"<<request->att[0]<<endl;                   
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
 
    cmd_x = cmd_x * (1 - smooth) + (std::fabs(request->command[0]) < dead_zone ? 0.0 : request->command[0]) * smooth;
    cmd_y = cmd_y * (1 - smooth) + (std::fabs(request->command[1]) < dead_zone ? 0.0 : request->command[1]) * smooth;
    cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request->command[2]) < dead_zone ? 0.0 : request->command[2]) * smooth;

    float max = 1.0;
    float min = -1.0;

    // double heading = atan2((double)forward(1,0), (double)forward(0,0));
    // double angle = (double)rot - heading;
    // angle = fmod(angle,2.0*M_PI);
    // if(angle > M_PI)
    // {
    //     angle = angle - 2.0*M_PI;
    // }
    // angle = angle*0.5;
    // angle = std::max(std::min((float)angle, max), min);
    // angle = angle * 0.25;

    obs.push_back(cmd_x);//控制指令x
    obs.push_back(cmd_y);//控制指令y
    obs.push_back(cmd_rate);//控制指令yaw rate

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
        obs.push_back(action_temp[i]);//历史  self.cfg.env.history_len, self.num_dofs
    }
    std::cout<<("$$$$$$$$$$$$$$$$$$$$$")<<std::endl;
    cout<<"obs:"<<obs<<endl;
    std::cout<<("$$$$$$$$$$$$$$$$$$$$$")<<std::endl;

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
    this->action_buf = torch::cat({action_buf.index({Slice(1,None),Slice()}),action_tensor},0);
    cout<<"action1:"<<endl<<action_tensor<<endl;
    bool has_nan = false;
    for (float val : obs) {
        cout << val << " ";
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
    //std::cout<<("=========2========")<<action_blend_tensor<<std::endl;
    this->obs_buf = torch::cat({this->obs_buf.index({Slice(1, None), Slice()}), obs_tensor}, 0); // 历史观测移位
    // //obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);//历史观测移位
    // //----------------------------------------------------------------
    float action_lcm[36]={0};
    torch::Tensor action_raw = action_blend_tensor.squeeze(0);
    // move to cpu
    action_raw = action_raw.to(torch::kFloat32);
    action_raw = action_raw.to(torch::kCPU);
    // // assess the result
    auto action_getter = action_raw.accessor <float,1>();//bug
    for (int j = 0; j < 12; j++)
    {
        action_lcm[j]=action[j] = action_getter[j] * action_scale[j] + init_pos[j];
        action_temp[j] = action_getter[j];//原始值
    }
     
    sendMessage(action_lcm);
}

int main(int argc, char** argv) {
    // load policy
    std::cout << "RL model thread start"<<endl;
    cout << torch::cuda::is_available() << endl;
    cout << torch::cuda::cudnn_is_available() << endl;
    cout << torch::cuda::device_count() << endl;
 
 
    Handler handlerObject;
    handlerObject.model_path = "/home/pi/Downloads/LocomotionWithNP3O-master/model_jitt.pt";//载入jit模型
    handlerObject.load_policy();

 // initialize record
    handlerObject.action_buf = torch::zeros({handlerObject.history_length,12},handlerObject.device);
    handlerObject.obs_buf = torch::zeros({handlerObject.history_length,45},handlerObject.device);//历史观测
    handlerObject.last_action = torch::zeros({1,12},handlerObject.device);

    handlerObject.action_buf.to(torch::kHalf);
    handlerObject.obs_buf.to(torch::kHalf);
    handlerObject.last_action.to(torch::kHalf);

    for (int j = 0; j < 12; j++)
    {
        handlerObject.action_temp.push_back(0.0);
	    handlerObject.action.push_back(handlerObject.init_pos[j]);
        handlerObject.prev_action.push_back(handlerObject.init_pos[j]);
    }
    //hot start
    for (int i = 0; i < handlerObject.history_length; i++)//为历史观测初始化
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
        torch::Tensor obs_tensor = torch::from_blob(obs.data(),{1,45},options).to(handlerObject.device);
        // append obs to obs buffer
        std::cout<<("=========1=======")<<std::endl;
        handlerObject.obs_buf = torch::cat({handlerObject.obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);//历史观测移位
    }

    for (int i = 0; i < 200; i++)
    {
        //handlerObject.model_infer();
    }

    handlerObject.subscribeRequest();
    handlerObject.run();
    return 0;
}
 

int Handler::load_policy()
{   
    std::cout << model_path << std::endl;
    // load model from check point
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device= torch::kCPU;
    if (torch::cuda::is_available()){
        device = torch::kCUDA;
    }
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
// int Handler::load_policy() {

//         std::cout << "Model path: " << model_path << std::endl;
//         // Check CUDA availability
//         std::cout << "cuda::is_available(): " << torch::cuda::is_available() << std::endl;
//         device = torch::kCPU;
//         if (torch::cuda::is_available()) {
//             device = torch::kCUDA;
//         }

//         // Load model from checkpoint
//         model = torch::jit::load(model_path, device);
//         std::cout << "Model loaded successfully!" << std::endl;

//         // Optionally convert model to half precision if supported
//         model.to(torch::kHalf);
//         std::cout << "Model moved to device!" << std::endl;

//         // Set model to evaluation mode
//         model.eval();
//         std::cout << "Model set to evaluation mode!" << std::endl;
// }


torch::Tensor Handler::model_infer(torch::Tensor ori_policy_input){
    // policy_input 是通过LCM传过来的，可能还需要处理才能喂给模型
    std::vector<torch::jit::IValue> policy_input;
    policy_input.push_back(ori_policy_input);
    //
    // Execute the model and turn its output into a tensor.






    torch::Tensor action_tensor = model.forward(policy_input).toTensor();

    return action_tensor;
    
}

void Handler::sendMessage(float action[36]){//exp_q exp_dq exp_tau
    // 把接收到的数据用网络处理后，以action的形式 返回去
    my_lcm::Response response;
    for(int i=0;i<12;i++)
        response.q_exp[i]=action[i];
 
    lcm.publish("RESPONSE_CHANNEL", &response);
    std::cout << "pub action succ" << std::endl;
}