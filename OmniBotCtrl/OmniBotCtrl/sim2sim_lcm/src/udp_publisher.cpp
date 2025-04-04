
#include "udp_publish.h"
#include <iostream>
#include <valarray>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <sys/shm.h>
#include <arpa/inet.h>
#include <time.h>
using namespace std;
using namespace torch::indexing; // 确保使用了正确的命名空间
RL_Tinymal_UDP tinymal_rl;
//RL
struct _msg_request msg_request;
struct _msg_response msg_response;
float limit(float input,float min,float max){
    if(input>max)
        return max;
    if(input<min)
        return min;
    return input;
}

void RL_Tinymal_UDP::handleMessage(_msg_request request)//获取机器人反馈
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
    #if 0
        cout<<"cmd:";
        cout<<request.command[0]<<" ";
        cout<<request.command[1]<<" ";
        cout<<request.command[2]<<" ";
        cout<<endl;
        cout<<"att:";
        cout<<request.eu_ang[0]<<" ";
        cout<<request.eu_ang[1]<<" ";
        cout<<request.eu_ang[2]<<" ";
        cout<<endl;
        cout<<"rate:";
        cout<<request.omega[0]<<" ";
        cout<<request.omega[1]<<" ";
        cout<<request.omega[2]<<" ";
        cout<<endl;
        cout<<"q:";
        for(int i=0;i<12;i++)
            cout<<request.q[i]<<" ";
        cout<<endl;
        cout<<"dq:";
        for(int i=0;i<12;i++)
            cout<<request.dq[i]<<" ";
        cout<<endl;    
        cout<<"trigger:"<<request.trigger<<" ";
        
    #endif
    // 将 data 转为 tensor 类型，输入到模型
    if(request.trigger==1){
        request.trigger=0;
        std::vector<float> obs;
        //---------------Push data into obsbuf--------------------
        obs.push_back(request.omega[0]*omega_scale);
        obs.push_back(request.omega[1]*omega_scale);
        obs.push_back(request.omega[2]*omega_scale);

        obs.push_back(request.eu_ang[0]*eu_ang_scale);
        obs.push_back(request.eu_ang[1]*eu_ang_scale);
        obs.push_back(request.eu_ang[2]*eu_ang_scale);

        // cmd
        float max = 1.0;
        float min = -1.0;

        cmd_x = cmd_x * (1 - smooth) + (std::fabs(request.command[0]) < dead_zone ? 0.0 : request.command[0]) * smooth;
        cmd_y = cmd_y * (1 - smooth) + (std::fabs(request.command[1]) < dead_zone ? 0.0 : request.command[1]) * smooth;
        cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request.command[2]) < dead_zone ? 0.0 : request.command[2]) * smooth;

        obs.push_back(cmd_x*lin_vel);//控制指令x
        obs.push_back(cmd_y*lin_vel);//控制指令y
        obs.push_back(cmd_rate*ang_vel);//控制指令yaw rate

        // pos q joint
        for (int i = 0; i < 12; ++i)
        {
            float pos = (request.q[i]  - init_pos[i])* pos_scale;
            obs.push_back(pos);
        }
        // vel q joint
        for (int i = 0; i < 12; ++i)
        {
            float vel = request.dq[i] * vel_scale;
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
            action[j] = limit(action_getter[j],-5,5); 
            action_temp[j] = limit(action_getter[j],-5,5);//原始值
        }
        
        action_refresh=1;
    }
}


int RL_Tinymal_UDP::load_policy()
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
 
int RL_Tinymal_UDP::init_policy(){
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
    int sock_fd;
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        exit(1);
    }

    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    //string UDP_IP="192.168.1.186";// 
#if 0
    string UDP_IP="127.0.0.1";// 
    int SERV_PORT= 8888 ;// 
#else
    string UDP_IP="192.168.1.106";// 
    int SERV_PORT= 10000 ;// 
#endif
    addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str());//机器人是客户端 软件主动发送
    addr_serv.sin_port = htons(SERV_PORT);
    len = sizeof(addr_serv);

    int recv_num=0,send_num=0;
    int connect=0,loss_cnt=0;
    char send_buf[500]={0},recv_buf[500]={0};

    tinymal_rl.init_policy();
     for(int i=0;i<12;i++)
        msg_response.q_exp[i]=tinymal_rl.action[i];
    printf("Thread UDP RL\n");
    int cnt_p=0;
    while (1)
    {
        //send action
        if(tinymal_rl.action_refresh){
            tinymal_rl.action_refresh=0;
            for(int i=0;i<12;i++)
                //msg_response.q_exp[i]=i+cnt_p;//tinymal_rl.action[i];
                msg_response.q_exp[i]=tinymal_rl.action[i];
            std::cout.precision(2);
            #if 1
                cout<<endl;
                cout<<"act send:";
                for(int i=0;i<12;i++)
                cout<<msg_response.q_exp[i]<<" ";
                cout<<endl;
            #endif
            cnt_p++;
        }
        memcpy(send_buf,&msg_response,sizeof(msg_response));//send joint command in python script
        send_num = sendto(sock_fd, send_buf, sizeof(msg_response), MSG_WAITALL, (struct sockaddr *)&addr_serv, len);
 
        if(send_num < 0)
        {
            perror("Robot sendto error:");
            exit(1);
        }
        //get obs
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_WAITALL, (struct sockaddr *)&addr_serv, (socklen_t *)&len);
        if(recv_num >0)
        {
            memcpy(&msg_request,recv_buf,sizeof(msg_request));
            
            tinymal_rl.handleMessage(msg_request);
        }
        usleep(5*1000);
    }
    return 0;
}

