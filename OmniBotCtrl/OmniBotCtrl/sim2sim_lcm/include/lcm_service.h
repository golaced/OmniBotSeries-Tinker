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

class Handler {
public:

    Handler(){
        if (!lcm.good()) {
        std::cerr << "LCM initialization failed" << std::endl;
        exit(1);
        }
    }
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const my_lcm::Response* msg);
    void sendMessage();

    void subscribeRequest() {
        
        lcm.subscribe("LCM_ACTION", &Handler::handleMessage, this);
        
    };
    void run() {
        while (0 == lcm.handle());
    };

    std::vector<float> action;
    std::vector<float> action_temp;
    std::vector<float> prev_action;

    float init_pos[12] = {-0.16,0.68,1.1 ,0.16,0.68,1.1, -0.16,0.68,1.1, 0.16,0.68,1.1};
    lcm::LCM lcm;
private:

   
};
