#include "lcm_service.h"
#include "stdio.h"

using namespace std;

void Handler::handleMessage(const lcm::ReceiveBuffer* rbuf,//50Hz pubulish
                    const std::string& chan,
                    const my_lcm::Response* msg) {
                        
    
    sendMessage();
}

int main(int argc, char** argv) {
    // load policy
    std::cout << "robot-sim thread start"<<endl;
    Handler handlerObject;

    handlerObject.subscribeRequest();
    handlerObject.run();
    return 0;
}
 

void Handler::sendMessage(){//
    static int cnt_send=0;
    my_lcm::Request obs;
    #if 1

    obs.eu_ang[0]=0.1;
    obs.eu_ang[1]=0.1;
    obs.eu_ang[2]=0.1;

    obs.omega[0]=0.1;
    obs.omega[1]=0.1;
    obs.omega[2]=0.1;

    obs.command[0]=0;
    obs.command[1]=0;
    obs.command[2]=0;
    for(int i=0;i<12;i++)
    {
        obs.q[i]=init_pos[i];
        obs.dq[i]=0;
    }
    #endif

    lcm.publish("LCM_OBS", &obs);
    std::cout << "pub ob succ=" <<cnt_send++<< std::endl;
}