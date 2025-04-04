#include "sys_time.h"
#include "mem_node.h"
#include "base_struct.h"
#include "udp_ocu.h"
#include "comm.h"
#include "mem_node.h"
#include "can.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

//nav_tx.rc_spd_b[0]=			ocu.rc_spd_w[Xr];
//nav_tx.rc_spd_b[1]=			ocu.rc_spd_w[Yr];
//nav_tx.rc_rate_b[0]	=		ocu.rc_att_w[PITr];
//nav_tx.rc_rate_b[1]	=		ocu.rc_att_w[ROLr];
//nav_tx.rc_rate_b[2]	=		ocu.rate_yaw_w;
//nav_tx.key_st=			ocu.key_st;
//nav_tx.key_back=		ocu.key_back;
//nav_tx.key_lr=			ocu.key_lr;
//nav_tx.key_ud=			ocu.key_ud;
//nav_tx.key_x=			ocu.key_x;
//nav_tx.key_y=			ocu.key_y;
//nav_tx.key_b=			ocu.key_b;
//nav_tx.key_a=			ocu.key_a;
//nav_tx.key_ll=			ocu.key_ll;
//nav_tx.key_rr=			ocu.key_rr;

void simple_test(float dt){
    static int state=0;
    static float timer=0;
    switch(state){
        case 0:
            if(ocu.key_back){
                state++;
                printf("Simple API Start!\n");
            }
        break;
        case 1:
            if(nav_rx.gait_state==M_SAFE)//init leg
            {
                nav_tx.key_x=1;
                nav_tx.key_rr=1;
                timer=0;
                state++;
            }
        break;
        case 2:
            timer+=dt;
            if(timer>0.6){
               timer=0;
               state++;
               nav_tx.key_ll=1;
            }
        break;
        case 3://force gait enable
            timer+=dt;
            if(timer>0.6){
               timer=0;
               state++;
               nav_tx.key_ll=0;
            }
        break;
        //--
        case 4:
            if(nav_rx.gait_state!=M_SAFE)
             timer+=dt;
            if(timer>0.1){//stand
                timer=0;
                state++;
                nav_tx.key_x=0;
                nav_tx.key_rr=0;
                nav_tx.key_ll=1;
            }
        break;
        case 5:
             timer+=dt;

            if(timer>3)
             nav_tx.rc_rate_b[0]=0;//change pitch
            else if(timer>2)
             nav_tx.rc_rate_b[0]=1;
            else if(timer>1)
             nav_tx.rc_rate_b[0]=-1;

            if(timer>4){//sit down
                timer=0;
                state++;
                nav_tx.key_ll=0;
                nav_tx.key_rr=1;
                nav_tx.rc_rate_b[0]=0;
            }
        break;
        case 6:
             timer+=dt;
            if(timer>1){
                timer=0;
                state=0;
                nav_tx.key_rr=0;
                nav_tx.key_ud=-1;
                printf("Simple API Done!\n");
            }
        break;
    }

    if(state>0&&(ocu.key_ud==-1||ocu.key_y==1))
    {
        printf("Simple API Reset!\n");
        state=0;

        nav_tx.rc_spd_b[0]=0;//			ocu.rc_spd_w[Xr];
        nav_tx.rc_spd_b[1]=0;//			ocu.rc_spd_w[Yr];
        nav_tx.rc_rate_b[0]	=0;//		ocu.rc_att_w[PITr];
        nav_tx.rc_rate_b[1]	=0;//		ocu.rc_att_w[ROLr];
        nav_tx.rc_rate_b[2]	=0;//		ocu.rate_yaw_w;
        nav_tx.key_st=		0;//	ocu.key_st;
        nav_tx.key_back=	0;//	ocu.key_back;
        nav_tx.key_lr=		0;//	ocu.key_lr;
        nav_tx.key_ud=		0;//	ocu.key_ud;
        nav_tx.key_x=		0;//	ocu.key_x;
        nav_tx.key_y=		0;//	ocu.key_y;
        nav_tx.key_b=		0;//	ocu.key_b;
        nav_tx.key_a=		0;//	ocu.key_a;
        nav_tx.key_ll=		0;//	ocu.key_ll;
        nav_tx.key_rr=		0;//	ocu.key_rr;
    }
}
