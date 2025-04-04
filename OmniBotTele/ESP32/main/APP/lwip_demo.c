/**
 ****************************************************************************************************
 * @file        udp.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-26
 * @brief       lwip实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32S3 BOX开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "lwip_demo.h"
#include "robot.h"

/* 接收数据缓冲区 */
uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE]; 
/* 发送数据内容 */
char g_lwip_demo_sendbuf[] = "UDP Test DATA \r\n";
/* 数据发送标志位 */
uint8_t g_lwip_send_flag;
static struct sockaddr_in dest_addr;            /* 远端地址 */
struct sockaddr_in g_local_info;
socklen_t g_sock_fd;                            /* 定义一个Socket接口 */
extern QueueHandle_t g_display_queue;           /* 显示消息队列句柄 */
/**
 * @brief       lwip_demo实验入口
 * @param       无
 * @retval      无
 */
struct _msg_response_tinker//UDP的读取机器人反馈
{
    char gait_mode;
    char auto_mode;
    char head_mode;
    char step_mode;
    char heart_cnt;
    float att[3];
    float vel[3];
    float acc[3];
    float mag[3];
    float gyro[3];
    float baro;
    float pos[3];
    float q_leg[14];
    float q_arm[14];            
} _msg_response_tinker;

struct _msg_request_tinker//向机器人发送数据
{
    char Head1;
    char Head2;
    char gait_mode;
    char auto_mode;
    char head_mode;
    char step_mode;
    char cal_mode;
    char cal_sel;
    float att_cmd[3];
    float vel_cmd[3];
    float pos_cmd[3];    
    char key_rc[4];
    int power_cnt;      
} _msg_request_tinker;

void lwip_demo(char *ip_addr, uint16_t port)
{
    char *tbuf;
 
    /* 远端参数设置 */
    dest_addr.sin_addr.s_addr = inet_addr(ip_addr);             /* 目标地址 */
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);                 /* 目标端口 */
    g_local_info.sin_family = AF_INET;                          /* IPv4地址 */
    g_local_info.sin_port = htons(port);              /* 设置端口号 */
    g_local_info.sin_addr.s_addr = htons(INADDR_ANY);           /* 设置本地IP地址 */
    g_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);                 /* 建立一个新的socket连接 */
    
    tbuf = malloc(1024);                                         /* 申请内存 */
    sprintf((char *)tbuf, "建立UDP Port:%d", port);           /* 客户端端口号 */
    bind(g_sock_fd, (struct sockaddr *)&g_local_info, sizeof(g_local_info));
    printf("设置UDP ip_addr: %s port=%d\n", ip_addr,port);
    robot_lvgl.udp_connected=true;
}



void lwip_send_dir(void)//发送线程
{
        //发送
        static int heart_cnt;
        static int cntp=0;
        if (1)  /* 有数据要发送 */
        {
            _msg_request_tinker.Head1=0xFA;
            _msg_request_tinker.Head2=0xFB;
            _msg_request_tinker.gait_mode=robot_cmd.gait_mode;
            _msg_request_tinker.auto_mode=robot_cmd.auto_mode;
            _msg_request_tinker.head_mode=robot_cmd.head_mode;
            _msg_request_tinker.step_mode=robot_cmd.step_mode;
            for(int i=0;i<3;i++){
                _msg_request_tinker.att_cmd[i]=robot_cmd.att[i];
                _msg_request_tinker.vel_cmd[i]=robot_cmd.vel[i];
                _msg_request_tinker.pos_cmd[i]=robot_cmd.pos[i];
            }
            for(int i=0;i<4;i++){
                _msg_request_tinker.key_rc[i]=robot_cmd.key_rc[i];
            }
            #if 0
            if(cntp++>20){
                cntp=0;
                printf("robot_cmd vel:%.1f %.1f %.1f\n",robot_cmd.vel[0],robot_cmd.vel[1],robot_cmd.vel[2]);
                printf("robot_cmd att:%.1f %.1f %.1f\n",robot_cmd.att[0],robot_cmd.att[1],robot_cmd.att[2]);
            }
            #endif
            _msg_request_tinker.cal_mode=robot_cmd.cal_mode;
            _msg_request_tinker.cal_sel=robot_cmd.cal_sel;
            memcpy(g_lwip_demo_sendbuf,&_msg_request_tinker,sizeof(_msg_request_tinker));
            sendto(g_sock_fd, g_lwip_demo_sendbuf, sizeof(_msg_request_tinker), MSG_DONTWAIT, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
        //printf("tx:%s\r\n",g_lwip_demo_sendbuf);
        #if 1
         //阻塞接收
        memset(g_lwip_demo_recvbuf, 0, sizeof(g_lwip_demo_recvbuf));
        recv(g_sock_fd, (void *)g_lwip_demo_recvbuf, sizeof(g_lwip_demo_recvbuf), MSG_DONTWAIT);
        memcpy(&_msg_response_tinker,g_lwip_demo_recvbuf,sizeof(_msg_response_tinker));

        if(1)//_msg_response_tinker.att[0]!=0&&_msg_response_tinker.att[1]!=0&&_msg_response_tinker.att[2]!=0)//null data check
        {   //printf("rx:%s\r\n",g_lwip_demo_recvbuf);
            robot_fb.gait_mode = _msg_response_tinker.gait_mode;
            robot_fb.auto_mode = _msg_response_tinker.auto_mode;
            robot_fb.head_mode = _msg_response_tinker.head_mode;
            robot_fb.step_mode = _msg_response_tinker.step_mode;
            for(int i=0;i<3;i++){
                robot_fb.att[i] =_msg_response_tinker.att[i];
                robot_fb.vel[i] =_msg_response_tinker.vel[i];
                robot_fb.pos[i] =_msg_response_tinker.pos[i];
                robot_fb.acc[i] =_msg_response_tinker.acc[i];
                robot_fb.gyro[i] =_msg_response_tinker.gyro[i];
                robot_fb.mag[i] =_msg_response_tinker.mag[i];
                robot_fb.baro =_msg_response_tinker.baro;
            }
            for(int i=0;i<14;i++){
                robot_fb.q_leg[i] =_msg_response_tinker.q_leg[i];
                robot_fb.q_arm[i] =_msg_response_tinker.q_arm[i];
            }
            if(_msg_response_tinker.heart_cnt!=heart_cnt){
                robot_fb.connect = 1;
                robot_fb.loss_cnt = 0;
            }
            heart_cnt=_msg_response_tinker.heart_cnt;
        }
        //printf("rx:%s\r\n",g_lwip_demo_recvbuf);

        #endif
}
