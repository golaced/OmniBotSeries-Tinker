#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "spi.h"
#include "sys_time.h"
#include <pthread.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include "imu_cmd.h"
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <iostream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

YAML::Node config_hardware=YAML::LoadFile("/home/odroid/Tinker/Param/param_hardware.yaml");

// ps -ef | grep hardware_task
// kill －9 324
_MEMS mems;

#define SPI_TEST 0 //Debug

#define USE_USB 0
#define USE_SERIAL 0

#define EN_SPI_BIG 1
#define SEND_DIV_SPI 0
#define CAN_LINK_COMM_VER1 0
#define CAN_LINK_COMM_VER2 1//3 BLDC Param DIV

#if EN_SPI_BIG
#if CAN_LINK_COMM_VER1
    #define SPI_SEND_MAX  85
#else
    #if SEND_DIV_SPI
        #define SPI_SEND_MAX  100//equal to stm32 spi send cnt all
    #else
        #define SPI_SEND_MAX  162//equal to stm32 spi send cnt all
    #endif
#endif
#else
#define SPI_SEND_MAX  40//40
#endif
#define EN_MULTI_THREAD 1
#define NO_THREAD 0
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MEM_SPI 0001
#define MEM_SIZE 2048
#define EN_DBG_PRINT 0

#if NO_THREAD&&!EN_MULTI_THREAD
static uint32_t speed = 150000*4;//3Mhz 死机
#define DELAY_SPI 250//us
#else
static uint32_t speed = 20000000;//odroid 20M  upbord 2M  raspberrypi 5M
#define DELAY_SPI 250//us
#endif
float spi_loss_cnt = 0;
int spi_connect = 0;
float mems_usb_loss_cnt = 0;
int mems_usb_connect = 0;

using namespace std;
std::string local_ip;
float local_ip1=0;
float local_ip2=0;

_SPI_RX spi_rx;
_SPI_TX spi_tx;
_OCU ocu;
uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt_show=0;
int spi_tx_cnt = 0;
int spi_rx_cnt = 0;

uint8_t usb_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t usb_rx_buf[SPI_BUF_SIZE] = {0};
int usb_tx_cnt = 0;
int usb_rx_cnt = 0;

uint8_t tx[SPI_BUF_SIZE] = {};
uint8_t rx[ARRAY_SIZE(tx)] = {};

std::string getLocalIP() {
    struct ifaddrs *ifaddr, *ifa;
    std::string ip;
    if (getifaddrs(&ifaddr) == -1) return "";

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
            continue;
        sockaddr_in* sa = (sockaddr_in*)ifa->ifa_addr;
        char ipStr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sa->sin_addr, ipStr, INET_ADDRSTRLEN);
        if (std::string(ipStr) != "127.0.0.1") {
            ip = ipStr;
            break;
        }
    }
    freeifaddrs(ifaddr);

// 使用split函数将IP地址字符串分割成各个部分
       std::istringstream iss(ip);
       std::vector<std::string> ip_parts;
       std::string part;

       while (std::getline(iss, part, '.')) {
           ip_parts.push_back(part);
       }

       // 检查是否至少有两个部分
       if (ip_parts.size() >= 2) {
           // 提取最后两位
           std::string part3 = ip_parts[ip_parts.size() - 2]; // 倒数第二位
           std::string part4 = ip_parts[ip_parts.size() - 1]; // 最后一位

           // 将它们转换为float类型
           float float_part3 = std::stof(part3);
           float float_part4 = std::stof(part4);
           local_ip1=float_part3;
           local_ip2=float_part4;
           // 输出结果
//           std::cout << "倒数第二位: " << float_part3 << std::endl;
//           std::cout << "最后一位: " << float_part4 << std::endl;
       } else {
           std::cerr << "IP地址格式不正确，部分不足" << std::endl;
       }
    return ip;
}

static void setDataInt_spi(int i)
{
    spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
    spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f)
{
    int i = *(int *)&f;
    spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
    spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi_int(float f,float size)
{
    int16_t _temp;
    _temp=f*size;
    spi_tx_buf[spi_tx_cnt++] = BYTE1(_temp);
    spi_tx_buf[spi_tx_cnt++] = BYTE0(_temp);
}

static float floatFromData_spi(unsigned char *data, int *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));

    *anal_cnt += 4;
    return *(float *)&i;
}

static float floatFromData_spi_int(unsigned char *data, int *anal_cnt,float size)
{
    float temp=0;
    temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
    *anal_cnt += 2;
    return temp;
}

static char charFromData_spi(unsigned char *data, int *anal_cnt)
{
    int temp = *anal_cnt;
    *anal_cnt += 1;
    return *(data + temp);
}

static int intFromData_spi(unsigned char *data, int *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));
    *anal_cnt += 4;
    return i;
}

float To_180_degrees(float x)
{
    return (x>180?(x-360):(x<-180?(x+360):x));
}

void can_board_send(char sel)//发送到单片机
{
    int i;
    static float t_temp = 0;
    char id = 0;
    char sum_t = 0, _cnt = 0;
    static char bldc_id_sel=0;
    spi_tx_cnt = 0;

    spi_tx_buf[spi_tx_cnt++] = 0xFE;
    spi_tx_buf[spi_tx_cnt++] = 0xFC;
    spi_tx_buf[spi_tx_cnt++] = sel;
    spi_tx_buf[spi_tx_cnt++] = 0;

    switch (sel)
    {
    case 45://14 BLDC type
#if SPI_TEST//TEST
        for (int id = 0; id < 14; id++)
        {
            spi_tx.q_set[id]=i+0.0;
            spi_tx.tau_ff[id]=i+0.1;
            spi_tx.q_reset[id]=i+0.2;
            spi_tx.kp[id]=0.5;
            spi_tx.kd[id]=0.11;
            spi_tx.stiff[id]=1.0;
        }
#endif
        spi_tx_buf[spi_tx_cnt++] = spi_tx.en_motor*100+spi_tx.reser_q*10+spi_tx.reset_err;
        spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE*100+mems.Gyro_CALIBRATE*10+mems.Mag_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  spi_tx.beep_state;
#if 0
        printf("en=%d %f %f kp=%f kd=%f\n", spi_tx.en_motor,spi_tx.q_set[0],spi_tx.q_set[1],spi_tx.kp[0],spi_tx.kd[0]);

#endif
        for (int id = 0; id < 14; id++)
        {
            setDataFloat_spi_int(spi_tx.q_set[id],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.kp[id],CAN_GAIN_DIV_P_M);
            setDataFloat_spi_int(spi_tx.kd[id],CAN_GAIN_DIV_D_M);
        }
    break;
    case 55://14 BLDC type
        spi_tx_buf[spi_tx_cnt++] =  spi_tx.en_servo;
        spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE*100+mems.Gyro_CALIBRATE*10+mems.Mag_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  spi_tx.beep_state;

        for (int id = 0; id < 14; id++)
        {
            setDataFloat_spi_int(spi_tx.q_set_servo[id],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.stiff[id],CAN_F_DIV);
            spi_tx_buf[spi_tx_cnt++] = spi_tx.reser_q_div[id];
        }
        //printf("bldc:%d %d %d\n",spi_tx.reser_q_div[0],spi_tx.reser_q_div[1],spi_tx.reser_q_div[2]);
    break;
    case 56://14 servo type
        spi_tx_buf[spi_tx_cnt++] =  spi_tx.en_servo;
        spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE*100+mems.Gyro_CALIBRATE*10+mems.Mag_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  spi_tx.beep_state;

        for (int id = 0; id < 14; id++)
        {
            setDataFloat_spi_int(spi_tx.tau_ff_servo[id],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_reset_servo[id],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.stiff_servo[id],CAN_F_DIV);
            spi_tx_buf[spi_tx_cnt++] = spi_tx.reser_q_div_servo[id];
        }
        //printf("%d %d %d\n",spi_tx.reser_q_div[0],spi_tx.reser_q_div[1],spi_tx.reser_q_div[2]);
    break;
    case 50://发送OCU配置
        //--------------------Param From OCU-------------
        setDataFloat_spi_int( mems.imu_pos.x,1000);
        setDataFloat_spi_int( mems.imu_pos.y,1000);
        setDataFloat_spi_int( mems.imu_pos.z,1000);
        setDataFloat_spi_int( mems.imu_att.x,CAN_POS_DIV);
        setDataFloat_spi_int( mems.imu_att.y,CAN_POS_DIV);
        setDataFloat_spi_int( mems.imu_att.z,CAN_POS_DIV);
        setDataFloat_spi_int( mems.gps_pos.x,1000);
        setDataFloat_spi_int( mems.gps_pos.y,1000);
        setDataFloat_spi_int( mems.gps_pos.z,1000);
        spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  mems.Gyro_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  mems.Mag_CALIBRATE;

    break;
    case 51://发送System配置
        spi_tx_buf[spi_tx_cnt++] =  local_ip1;
        spi_tx_buf[spi_tx_cnt++] =  local_ip2;
        for (int id = 0; id < 14; id++)
        {
            setDataFloat_spi_int(spi_tx.kp_servo[id],CAN_GAIN_DIV_P_M);
            setDataFloat_spi_int(spi_tx.kd_servo[id],CAN_GAIN_DIV_D_M);
        }
    break;

    default:
    break;
    }

    spi_tx_buf[3] = (spi_tx_cnt)-4;
    for (i = 0; i < spi_tx_cnt; i++)
        sum_t += spi_tx_buf[i];
    spi_tx_buf[spi_tx_cnt++] = sum_t;

    if(spi_tx_cnt>SPI_SEND_MAX)
       printf("spi_tx_cnt=%d over flow!!!\n",spi_tx_cnt);
    spi_tx_cnt_show=spi_tx_cnt;
}

int slave_rx(uint8_t *data_buf, int num)//接收解码--------------from stm32
{
    static int cnt_p = 0;
    static int cnt_err_sum=0;
    uint8_t id;
    uint8_t sum = 0;
    uint8_t i;
    uint8_t temp;
    int anal_cnt = 4;
    for (i = 0; i < (num - 1); i++)
        sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1))){
        printf("spi sum err=%d sum_cal=0x%X sum=0x%X !!\n",cnt_err_sum++,sum,*(data_buf + num - 1));
        return 0;
    }
    if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFB)){
        printf("spi head err!!\n");
        return 0;
    }
    if (*(data_buf + 2) == 26) //------------------14 BLDC type
    {
        spi_loss_cnt = 0;
        if (spi_connect==0)
        {
            printf("Hardware::Hardware SPI-STM32 Link3-Sbus Tinker14!!!=%d!!!\n",spi_connect);
            spi_connect = 1;
        }

        spi_rx.att[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        //printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
        spi_rx.att_rate[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att_rate[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att_rate[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        spi_rx.acc_b[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_b[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_b[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        for (int i = 0; i < 14; i++)
        {
            spi_rx.q[i] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.dq[i] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_DPOS_DIV);
            spi_rx.tau[i] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);

            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i] = temp%10;
        }
#if SPI_TEST
    printf("att :%f %f %f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
    printf("rate:%f %f %f\n",spi_rx.att_rate[0],spi_rx.att_rate[1],spi_rx.att_rate[2]);
    printf("q0:%.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.q[0], spi_rx.q[1], spi_rx.q[2], spi_rx.q[3], spi_rx.q[4], spi_rx.q[5],spi_rx.q[6]);
    printf("q1:%.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",  spi_rx.q[7], spi_rx.q[8], spi_rx.q[9], spi_rx.q[10], spi_rx.q[11],spi_rx.q[12]);

    printf("t0:%.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.tau[0], spi_rx.tau[1], spi_rx.tau[2], spi_rx.tau[3], spi_rx.tau[4], spi_rx.tau[5], spi_rx.tau[6]);
    printf("t1:%.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.tau[7], spi_rx.tau[8], spi_rx.tau[9], spi_rx.tau[10], spi_rx.tau[11], spi_rx.tau[12]);
#endif
    }else if (*(data_buf + 2) == 36) //------------------14 Sevo type
    {
        spi_loss_cnt = 0;
        if (spi_connect==0)
        {
            printf("Hardware::Hardware SPI-STM32 Link3-Sbus Tinker14!!!=%d!!!\n",spi_connect);
            spi_connect = 1;
        }

        ocu.connect=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_st=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_back=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_lr=charFromData_spi(spi_rx_buf, &anal_cnt);//1

        ocu.key_ud=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_x=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_a=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_b=charFromData_spi(spi_rx_buf, &anal_cnt);//1

        ocu.key_y=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_ll=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.key_rr=charFromData_spi(spi_rx_buf, &anal_cnt);//1
        ocu.rc_spd_w[0]=floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);//2
        ocu.rc_spd_w[1]=floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);//2
        ocu.rc_att_w[0]=floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);//2
        ocu.rc_att_w[1]=floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);//2
        ocu.rate_yaw_w=floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);//2
        //36  11+10 =21
        for (int i = 0; i < 15; i++)
            temp=charFromData_spi(spi_rx_buf, &anal_cnt);//1

        for (int i = 0; i < 14; i++)
        {
            spi_rx.q_servo[i] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.dq_servo[i] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_DPOS_DIV);
            spi_rx.tau_servo[i] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);

            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect_servo[i] = temp;
        }
#if 0
    printf("att :%f %f %f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
    printf("rate:%f %f %f\n",spi_rx.att_rate[0],spi_rx.att_rate[1],spi_rx.att_rate[2]);
    printf("q0:%.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.q_servo[0], spi_rx.q_servo[1], spi_rx.q_servo[2], spi_rx.q_servo[3], spi_rx.q_servo[4], spi_rx.q_servo[5]);
    printf("q1:%.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.q_servo[6], spi_rx.q_servo[7], spi_rx.q_servo[8], spi_rx.q_servo[9], spi_rx.q_servo[10], spi_rx.q_servo[11]);

    printf("t0:%.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.tau[0], spi_rx.tau[1], spi_rx.tau[2], spi_rx.tau[3], spi_rx.tau[4], spi_rx.tau[5]);
    printf("t1:%.2f %.2f %.2f %.2f %.2f %.2f\n", spi_rx.tau[6], spi_rx.tau[7], spi_rx.tau[8], spi_rx.tau[9], spi_rx.tau[10], spi_rx.tau[11]);
#endif

    }

    return 1;
}

void transfer(int fd, int sel)//发送
{
    static uint8_t state, rx_cnt;
    static uint8_t _data_len2 = 0, _data_cnt2 = 0;
    int ret;
    uint8_t data = 0;

    can_board_send(sel);
    //printf("before1\n");
    ret=SPIDataRW(0,spi_tx_buf,rx,SPI_SEND_MAX); //向总线中写入7个数据
    //printf("after1\n");//printf("ret=%d cnt=%d\n",ret,spi_tx_cnt);
    #if 0
    for(int i=0;i<ret;i++)
        printf("0x%X ",rx[i]);

    printf("ret=%d \n",ret);
    #endif
    if (ret < 1){
       printf("SPI Reopen!\n");
       SPISetup(0,speed);//printf("can't send spi message\n");
    }
    else
    {
        //printf("ret=%d\n",ret);
        for (int i = 0; i < SPI_SEND_MAX; i++)
        {
            data = rx[i];
            if (state == 0 && data == 0xFF)
            {
                state = 1;
                spi_rx_buf[0] = data;
            }
            else if (state == 1 && data == 0xFB)
            {
                state = 2;
                spi_rx_buf[1] = data;
            }
            else if (state == 2 && data > 0 && data < 0XF1)
            {
                state = 3;
                spi_rx_buf[2] = data;
            }
            else if (state == 3 && data < SPI_BUF_SIZE)
            {
                state = 4;
                spi_rx_buf[3] = data;
                _data_len2 = data;
                _data_cnt2 = 0;
            }
            else if (state == 4 && _data_len2 > 0)
            {
                _data_len2--;
                spi_rx_buf[4 + _data_cnt2++] = data;
                if (_data_len2 == 0)
                    state = 5;
            }
            else if (state == 5)
            {
                state = 0;
                spi_rx_buf[4 + _data_cnt2] = data;
                spi_rx_cnt = 4;
                slave_rx(spi_rx_buf, _data_cnt2 + 5);
            }
            else
                state = 0;
            //printf("%02x ",rx[i]);
        }
        //printf("\n");
    }
    //printf("after2\n");
}

//----------------------------------------------------------------------------
int mem_connect=0;
float mem_loss_cnt=0;
struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi;

unsigned char mem_write_buf[MEM_SIZE];
unsigned char mem_read_buf[MEM_SIZE];
int mem_write_cnt=0;
static void setDataFloat_mem(float f)
{
    int i = *(int *)&f;
    mem_write_buf[mem_write_cnt++] = ((i << 24) >> 24);
    mem_write_buf[mem_write_cnt++] = ((i << 16) >> 24);
    mem_write_buf[mem_write_cnt++] = ((i << 8) >> 24);
    mem_write_buf[mem_write_cnt++] = (i >> 24);
}

static void setDataChar_mem(char f)
{
    mem_write_buf[mem_write_cnt++] = (f);
}

void memory_write(void)//写入内存 to control
{
    static float temp=0;
    mem_write_cnt=0;
    setDataFloat_mem( spi_rx.att[0]);
    setDataFloat_mem( spi_rx.att[1]);
    setDataFloat_mem( spi_rx.att[2]);
    setDataFloat_mem( spi_rx.att_rate[0]);
    setDataFloat_mem( spi_rx.att_rate[1]);
    setDataFloat_mem( spi_rx.att_rate[2]);
    setDataFloat_mem( spi_rx.acc_b[0]);
    setDataFloat_mem( spi_rx.acc_b[1]);
    setDataFloat_mem( spi_rx.acc_b[2]);
    setDataFloat_mem( spi_rx.acc_n[0]);
    setDataFloat_mem( spi_rx.acc_n[1]);
    setDataFloat_mem( spi_rx.acc_n[2]);
    //-----------MEMS USB

    //printf("moco:%f %f %f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
    //printf("moco:%f %f %f\n",spi_rx.acc_b[0],spi_rx.acc_b[1],spi_rx.acc_b[2]);

//    spi_rx.att_usb[1]= AHRSData_Packet.Roll*57.3;
//    spi_rx.att_usb[0]= AHRSData_Packet.Pitch*57.3;
//    spi_rx.att_usb[2]= To_180_degrees(AHRSData_Packet.Heading*57.3);
    spi_rx.att_usb[1]=-imu_o.att[0];
    spi_rx.att_usb[0]=imu_o.att[1];
    spi_rx.att_usb[2]=-imu_o.att[2];

#if 0
    spi_rx.att_rate_usb[0]=-AHRSData_Packet.RollSpeed*57.3;
    spi_rx.att_rate_usb[1]= AHRSData_Packet.PitchSpeed*57.3;
    spi_rx.att_rate_usb[2]=-AHRSData_Packet.HeadingSpeed*57.3;
#else
//    spi_rx.att_rate_usb[1]= IMUData_Packet.gyroscope_x*57.3;
//    spi_rx.att_rate_usb[0]= IMUData_Packet.gyroscope_y*57.3;
//    spi_rx.att_rate_usb[2]=-IMUData_Packet.gyroscope_z*57.3;
    spi_rx.att_rate_usb[1]=-imu_o.gyro[0];
    spi_rx.att_rate_usb[0]=imu_o.gyro[1];
    spi_rx.att_rate_usb[2]=imu_o.gyro[2];
    static int cnt_p=0;
    if(cnt_p++>200&&0){cnt_p=0;
        printf("Att::%f %f %f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
        printf("AttU:%f %f %f\n",spi_rx.att_usb[0],spi_rx.att_usb[1],spi_rx.att_usb[2]);
        printf("Rate::%f %f %f\n",spi_rx.att_rate_usb[0],spi_rx.att_rate_usb[1],spi_rx.att_rate_usb[2]);
        printf("RateU:%f %f %f\n",spi_rx.att_rate[0],spi_rx.att_rate[1],spi_rx.att_rate[2]);
    }
#endif
    spi_rx.acc_b_usb[0]=-IMUData_Packet.accelerometer_x/9.81;
    spi_rx.acc_b_usb[1]= IMUData_Packet.accelerometer_y/9.81;
    spi_rx.acc_b_usb[2]=-IMUData_Packet.accelerometer_z/9.81;
    //printf("use::%f %f %f\n",spi_rx.att_usb[0],spi_rx.att_usb[1],spi_rx.att_usb[2]);
    //printf("use::%f %f %f\n",spi_rx.acc_b_usb[0],spi_rx.acc_b_usb[1],spi_rx.acc_b_usb[2]);

    setDataChar_mem ( mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[0]-spi_rx.att_usb_bias[0])*mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[1]-spi_rx.att_usb_bias[1])*mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[2]-spi_rx.att_usb_bias[2])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[0]-spi_rx.att_rate_usb_bias[0])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[1]-spi_rx.att_rate_usb_bias[1])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[2]-spi_rx.att_rate_usb_bias[2])*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[0]*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[1]*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[2]*mems_usb_connect);

    for (int i = 0; i < 14; i++)
    {
        setDataFloat_mem(spi_rx.q[i]);
        setDataFloat_mem(spi_rx.dq[i]);
        setDataFloat_mem(spi_rx.tau[i]);
        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i]*10+spi_rx.ready[i]);
    }

    for (int i = 0; i < 14; i++)
    {
        setDataFloat_mem(spi_rx.q_servo[i]);
        setDataFloat_mem(spi_rx.tau_servo[i]);
        setDataChar_mem(spi_rx.connect_servo[i]);
    }
    setDataFloat_mem(spi_rx.bat_v);

    setDataChar_mem(ocu.connect);
    setDataChar_mem(ocu.key_st);
    setDataChar_mem(ocu.key_back);
    setDataChar_mem(ocu.key_lr);
    setDataChar_mem(ocu.key_ud);
    setDataChar_mem(ocu.key_x);
    setDataChar_mem(ocu.key_a);
    setDataChar_mem(ocu.key_b);
    setDataChar_mem(ocu.key_y);
    setDataChar_mem(ocu.key_ll);
    setDataChar_mem(ocu.key_rr);
    setDataFloat_mem(ocu.rc_spd_w[0]);
    setDataFloat_mem(ocu.rc_spd_w[1]);
    setDataFloat_mem(ocu.rc_att_w[0]);
    setDataFloat_mem(ocu.rc_att_w[1]);
    setDataFloat_mem(ocu.rate_yaw_w);
    setDataChar_mem(spi_connect);

}

void memory_read(void){//读取内存 from control
    int mem_read_cnt=MEM_SIZE/2;
    float test1,test2;
    spi_tx.en_motor= mem_read_buf[mem_read_cnt++]*mem_connect;
    spi_tx.en_servo= mem_read_buf[mem_read_cnt++]*mem_connect;
    spi_tx.reser_q= mem_read_buf[mem_read_cnt++];
    spi_tx.reset_err= mem_read_buf[mem_read_cnt++];

    mems.Acc_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    mems.Gyro_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    mems.Mag_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.beep_state=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    for (int i = 0; i < 14; i++)
    {
        spi_tx.q_set[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.q_reset[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.tau_ff[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_tx.kp[i]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.kd[i]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.stiff[i]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.reser_q_div[i]= mem_read_buf[mem_read_cnt++];
    }

    for (int i = 0; i < 14; i++)
    {
        spi_tx.q_set_servo[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.q_reset_servo[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.tau_ff_servo[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_tx.kp_servo[i]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.kd_servo[i]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.stiff_servo[i]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_tx.reser_q_div_servo[i]= mem_read_buf[mem_read_cnt++];
    }
#if 0//debug
    printf("%f %f %f %f\n",spi_tx.kp[0],spi_tx.ki[0],spi_tx.kd[0],spi_tx.stiff[0]);
    printf("%f %f %f %f\n",spi_tx.q_reset[0],spi_tx.q_reset[1],spi_tx.q_reset[2],spi_tx.q_reset[3]);

#endif
}


pthread_mutex_t lock;

void* Thread_Mem(void*)//内存管理线程
{
    static int cnt = 0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);

    while (1)
    {
        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
            mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            pthread_mutex_lock(&lock);
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pthread_mutex_unlock(&lock);
            pshm_rx->flag = 1;
        }else{
            mem_loss_cnt+=sys_dt;
            mem_init_cnt=0;
        }

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 14; i++){
                spi_tx.q_set[i] = spi_rx.q[i];
                spi_tx.tau_ff[i] = 0;
                spi_tx.kp[i]= spi_tx.ki[i]= spi_tx.kd[i]= spi_tx.en_motor= spi_tx.en_servo=0;
            }     
            printf("Hardware::Memery Control Loss!!!\n");
        }
        usleep(500);
    }
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}


void* Thread_SPI(void*)//内存管理线程
{
    static float timer_spi1 = 0,timer_spi2=0;
    static int cnt = 0;
    static int timer_1s=0;
    static int timer_1m=0;
    static int timer_1h=0;
    static float timer_cnt=0;
    static int send_flag=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int fd = 0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();
    fd=SPISetup(0,speed); //初始化SPI通道0，并设置为最大速度500000hz
    if(fd==-1)
        printf("init spi failed!\n");
    int cnt_icon=0;
    while (1)
    {
        sys_dt = Get_Cycle_T(15);
        timer_cnt+=sys_dt;
        if(timer_cnt>1){
            timer_cnt=0;
            timer_1s++;
            if(timer_1s>60){timer_1s=0;
                timer_1m++;
            }
            if(timer_1m>60){timer_1m=0;
                timer_1h++;}
            cnt_icon++;
            printf("Hardware::SPI Still Online at hour-%d min-%d sec-%d spi_cnt=%d",timer_1h,timer_1m,timer_1s,spi_tx_cnt_show);
            for(int i=0;i<cnt_icon;i++)
                printf("*");
            printf("\n");
            if(cnt_icon>8)
                cnt_icon=0;
        }

        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware SPI-STM32 Loss!!!\n");
        }
        //-------SPI CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;
#if SEND_DIV_SPI
        if(send_flag==0){
            send_flag=1;
            transfer(fd, 46);//3 bldc div
        }else{
            send_flag=0;
            transfer(fd, 47);//3 bldc div
        }
#else
        if(timer_spi2>1){
            timer_spi2=0;
            transfer(fd, 51);
        }
        else if(timer_spi1>0.01&&1){
            timer_spi1=0;
            if(flag==0){
                flag=1;
                transfer(fd, 55);
            }else{
                flag=0;
                transfer(fd, 56);
            }
        }
        else
            transfer(fd, 45);//3 bldc div
#endif
        usleep(DELAY_SPI);
    }
    close(fd);
    return 0;
}


#if NO_THREAD&&!EN_MULTI_THREAD
int Thread_ALL(void)
#else
void* Thread_ALL(void*)
#endif
{
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    int fd = 0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();

    fd=SPISetup(0,speed); //初始化SPI通道0，并设置为最大速度500000hz
    if(fd==-1)
        printf("init spi failed!\n");
    //while(1);
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);
    while (1)
    {
        sys_dt = Get_Cycle_T(15);
        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware SPI-STM32 Loss!!!\n");
        }
        //-------SPI CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;
        if(timer_spi2>1){
            timer_spi2=0;
            transfer(fd, 51);
        }
        else if(timer_spi1>0.01&&1){
            timer_spi1=0;
            if(flag==0){
                flag=1;
                transfer(fd, 55);
            }else{
                flag=0;
                transfer(fd, 56);
            }

        }
            transfer(fd, 45);

        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
                mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pshm_rx->flag = 1;
        }else
            mem_loss_cnt+=sys_dt;

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 14; i++){
                spi_tx.q_set[i] = spi_rx.q[i];
                spi_tx.tau_ff[i] = 0;
                spi_tx.kp[i]= spi_tx.ki[i]= spi_tx.kd[i]= spi_tx.en_motor= spi_tx.en_servo=0;
            }

            printf("Hardware::Memery Control Loss!!!\n");
        }
        usleep(200);
    }
    close(fd);
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}


//===========================================================USB-STM32 Comm===========================
#include "serial.hpp"
#include <serial/serial.h>
#include <iostream>
uint8_t buff[512];

using namespace std;
#if USE_SERIAL
//serial::Serial m_serial("/dev/ttyACM0",2000000 , serial::Timeout::simpleTimeout(1000));
#endif
const char *dev  = "/dev/ttyACM0";

void* Thread_USB(void*)//not use now
{

    int i;
    static uint8_t state_usb=0, rx_cnt_usb=0;
    static uint8_t _data_len2_usb = 0, _data_cnt2_usb = 0;
    int ret=0;
    int rx_length=42;
    uint8_t data_usb = 0;
    int iResult = -1;
    int fd = -1,iCommPort,iBaudRate,iDataSize,iStopBit;
    char cParity;
    int iLen;
#if !USE_SERIAL
    iCommPort = 1;
    fd = open_port(iCommPort);
    if( fd<0 )
    {
    close_port(fd);
    printf("Hardware::USB open_port error !\n");
    //return 1;
    }
    iBaudRate = 2000000;
    iDataSize = 8;
    cParity = 'N';
    iStopBit = 1;
    iResult = set_port(fd,iBaudRate,iDataSize,cParity,iStopBit);
#else
    serial::Serial m_serial;
    //配置串口：serial_port_ = "/dev/ttyUSB0"
    m_serial.setPort( "/dev/ttyACM0");
    //配置波特率：baudrate_ = 115200
    m_serial.setBaudrate(2000000);
    m_serial.setBytesize(serial::eightbits);
    //设置奇偶校验位//serial::parity_even :偶校验  serial::parity_odd :奇校验 serial::parity_mark :校验位始终为1 serial::parity_space :校验位始终为0 serial::parity_none :无校验
    m_serial.setParity(serial::parity_none);
    //设置停止位 serial::stopbits_one ：1位 serial::stopbits_one_point_five ：1.5位 serial::stopbits_two ：2位
    m_serial.setStopbits(serial::stopbits_one);
    m_serial.open();
    cout << "Is the serial port open?"<< endl;
    if(m_serial.isOpen())
    cout << " Yes." << endl;
    else
    cout << " No." << endl;
#endif
//------------------------------------------memery-------------------------------------
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int memory_update=0;
    int flag = 0;
    int link_cnt=0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();
    //共享内存
#if 0
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);
#endif
    fd_set  rset;
    int  rv = -1 ;
    int nread=0;
    struct timeval timeout;
    timeout.tv_sec=0;
    timeout.tv_usec=1000;

    while (1)
    {

        sys_dt = Get_Cycle_T(20);
        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware USB-STM32 Loss!!!\n");
        }

#if USE_SERIAL
        nread = m_serial.available();
        // printf(" read_cefore=%d\n",nread);
        if (nread>0){
            printf(" \n read_cefore=%d\n",nread);
            m_serial.read(buff, nread);
            //m_serial.flush();
            //m_serial.flushInput();
        }
#else
        //iLen = read_port(fd,buff,rx_length);
        FD_ZERO(&rset);
        FD_SET(fd, &rset);
        rv = select(fd+1, &rset, NULL, NULL, &timeout);
        nread=0;
        if(rv < 0)
        {
            //printf("select() failed: %s\n", strerror(errno));
            //return 0;
        }else{
        nread = read(fd, buff, rx_length);
        //printf("nread=%d\n",nread);
        }
#endif
       // printf("nread=%d\n ",nread);
        for (int i = 0; i < nread; i++)
        {
            //data_usb = buff[i];
            printf("%02x ",buff[i]);
            // printf("%d ",buff[i]);
            if (state_usb == 0 && data_usb == 0xFF)
            {
                state_usb = 1;
                usb_rx_buf[0] = data_usb;
            }
            else if (state_usb == 1 && data_usb == 0xFB)
            {
                state_usb = 2;
                usb_rx_buf[1] = data_usb;
                //printf("rx_check1\n");
            }
            else if (state_usb == 2 && data_usb > 0 && data_usb < 0XF1)
            {
                state_usb = 3;
                usb_rx_buf[2] = data_usb;
                 //printf("rx_check2 data=%d\n",data_usb);
            }
            else if (state_usb == 3 && data_usb < SPI_BUF_SIZE)
            {
                state_usb = 4;
                usb_rx_buf[3] = data_usb;
                _data_len2_usb = data_usb;
                _data_cnt2_usb = 0;
                //printf("rx_check3 data=%d\n",data_usb);
            }
            else if (state_usb == 4 && _data_len2_usb > 0)
            {
                _data_len2_usb--;
                usb_rx_buf[4 + _data_cnt2_usb++] = data_usb;
                if (_data_len2_usb == 0)
                    state_usb = 5;
            }
            else if (state_usb == 5)
            {
                state_usb = 0;
                usb_rx_buf[4 + _data_cnt2_usb] = data_usb;
                usb_rx_cnt = 4;
                for(i=0;i<SPI_BUF_SIZE;i++)
                    spi_rx_buf[i]=usb_rx_buf[i];
                slave_rx(spi_rx_buf, _data_cnt2_usb + 5);
                //printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
                //printf("rx_check3\n");
            }
            else
                state_usb = 0;
        }

       // printf(" iLen=%d\n",iLen);
        //printf("\n");

        //-------USB CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;
        if (timer_spi2 > 1)
        {
            timer_spi2 = 0;
            can_board_send(50);
        }
        #if EN_SPI_BIG||0
            can_board_send(20);
        #else
        else if (timer_spi1 > 0.1)//10ms
        {
            timer_spi1 = 0;
            can_board_send(2);
        }
        else
            can_board_send(1); //1ms
        #endif
#if 1
        for(i=0;i<spi_tx_cnt;i++){
            usb_tx_buf[i]=spi_tx_buf[i];
            //printf("%02x ",usb_tx_buf[i]);
        }
       // printf("\n");
#else
        spi_tx_cnt=0;
        for(i=0;i<41;i++)
         spi_tx_buf[spi_tx_cnt++]=i;
#endif
#if !USE_SERIAL
        iLen = write_port(fd,usb_tx_buf,spi_tx_cnt);
#else
        m_serial.write(usb_tx_buf, spi_tx_cnt);
#endif
        //printf("tx=%d\n",iLen);
#if 0
        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
                mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pshm_rx->flag = 1;
        }else
            mem_loss_cnt+=sys_dt;

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 4; i++){
            spi_tx.q_set[i][0] = spi_rx.q[i][0];
            spi_tx.q_set[i][1] = spi_rx.q[i][1];
            spi_tx.tau_ff[i][0] = 0;
            spi_tx.tau_ff[i][1] = 0;
            }
            spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;

            printf("Hardware::Memery Control Loss!!!\n");
        }
#endif
       usleep(50);
    }
    close(fd);
   // shmdt(shm_rx);  //失败返回-1，假设成功
    //shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}


//===========================================================USB-MEMS==========================================================
IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;
char ttl_receive;
char Fd_data[64];
char Fd_rsimu[64];
char Fd_rsahrs[64];
int rs_imutype =0;
int rs_ahrstype =0;
extern int Time_count;
const char *dev_usb_mems  = "/dev/ttyUSB0";

/*************
实现16进制的can数据转换成浮点型数据
****************/
float DATA_Trans(char Data_1,char Data_2,char Data_3,char Data_4)
{
    long long transition_32;
    float tmp=0;
    int sign=0;
    int exponent=0;
    float mantissa=0;
    transition_32 = 0;
    transition_32 |=  Data_4<<24;
    transition_32 |=  Data_3<<16;
    transition_32 |=  Data_2<<8;
    transition_32 |=  Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
    //先右移操作，再按位与计算，出来结果是30到23位对应的e
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    //将22~0转化为10进制，得到对应的x系数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp=sign * mantissa * pow(2, exponent);
    return tmp;
}

long long timestamp(char Data_1,char Data_2,char Data_3,char Data_4)
{
    long transition_32;
    transition_32 = 0;
    transition_32 |=  Data_4<<24;
    transition_32 |=  Data_3<<16;
    transition_32 |=  Data_2<<8;
    transition_32 |=  Data_1;
    return transition_32;
}

void AHRSData2PC(void)
{
    printf("AHRS: The RollSpeed =  %f\r\n",AHRSData_Packet.RollSpeed);
    printf("AHRS: The PitchSpeed =  %f\r\n",AHRSData_Packet.PitchSpeed);
    printf("AHRS: The HeadingSpeed =  %f\r\n",AHRSData_Packet.HeadingSpeed);
    printf("AHRS: The Roll =  %f\r\n",AHRSData_Packet.Roll);
    printf("AHRS: The Pitch =  %f\r\n",AHRSData_Packet.Pitch);
    printf("AHRS: The Heading =  %f\r\n",AHRSData_Packet.Heading);
//    printf("AHRS: The Quaternion.Qw =  %f\r\n",AHRSData_Packet.Qw);
//    printf("AHRS: The Quaternion.Qx =  %f\r\n",AHRSData_Packet.Qx);
//    printf("AHRS: The Quaternion.Qy =  %f\r\n",AHRSData_Packet.Qy);
//    printf("AHRS: The Quaternion.Qz =  %f\r\n",AHRSData_Packet.Qz);
//    printf("AHRS: The Timestamp =  %d\r\n",AHRSData_Packet.Timestamp);
}

void IMUData2PC(void)
{
    //printf("Now start sending IMU data.\r\n");
    printf("IMU: The gyroscope_x =  %f\r\n",IMUData_Packet.gyroscope_x);
    printf("IMU:The gyroscope_y =  %f\r\n",IMUData_Packet.gyroscope_y);
    printf("IMU:The gyroscope_z =  %f\r\n",IMUData_Packet.gyroscope_z);
    printf("IMU:The accelerometer_x =  %f\r\n",IMUData_Packet.accelerometer_x);
    printf("IMU:The accelerometer_y =  %f\r\n",IMUData_Packet.accelerometer_y);
    printf("IMU:The accelerometer_z =  %f\r\n",IMUData_Packet.accelerometer_z);
//    printf("IMU:The magnetometer_x =  %f\r\n",IMUData_Packet.magnetometer_x);
//    printf("IMU:The magnetometer_y =  %f\r\n",IMUData_Packet.magnetometer_y);
//    printf("IMU:The magnetometer_z =  %f\r\n",IMUData_Packet.magnetometer_z);
//    printf("IMU:The Timestamp =  %d\r\n",IMUData_Packet.Timestamp);
    //printf("Now the data of IMU has been sent.\r\n");

}

char TTL_Hex2Dec(void)
{
    char i;
     if(rs_ahrstype==1)
    {
        if(Fd_rsahrs[1]==TYPE_AHRS&&Fd_rsahrs[2]==AHRS_LEN)
        {
        AHRSData_Packet.RollSpeed=DATA_Trans(Fd_rsahrs[7],Fd_rsahrs[8],Fd_rsahrs[9],Fd_rsahrs[10]);       //横滚角速度
        AHRSData_Packet.PitchSpeed=DATA_Trans(Fd_rsahrs[11],Fd_rsahrs[12],Fd_rsahrs[13],Fd_rsahrs[14]);   //俯仰角速度
        AHRSData_Packet.HeadingSpeed=DATA_Trans(Fd_rsahrs[15],Fd_rsahrs[16],Fd_rsahrs[17],Fd_rsahrs[18]); //偏航角速度

        AHRSData_Packet.Roll=DATA_Trans(Fd_rsahrs[19],Fd_rsahrs[20],Fd_rsahrs[21],Fd_rsahrs[22]);      //横滚角
        AHRSData_Packet.Pitch=DATA_Trans(Fd_rsahrs[23],Fd_rsahrs[24],Fd_rsahrs[25],Fd_rsahrs[26]);     //俯仰角
        AHRSData_Packet.Heading=DATA_Trans(Fd_rsahrs[27],Fd_rsahrs[28],Fd_rsahrs[29],Fd_rsahrs[30]);	 //偏航角

        AHRSData_Packet.Qw=DATA_Trans(Fd_rsahrs[31],Fd_rsahrs[32],Fd_rsahrs[33],Fd_rsahrs[34]);  //四元数
        AHRSData_Packet.Qx=DATA_Trans(Fd_rsahrs[35],Fd_rsahrs[36],Fd_rsahrs[37],Fd_rsahrs[38]);
        AHRSData_Packet.Qy=DATA_Trans(Fd_rsahrs[39],Fd_rsahrs[40],Fd_rsahrs[41],Fd_rsahrs[42]);
        AHRSData_Packet.Qz=DATA_Trans(Fd_rsahrs[43],Fd_rsahrs[44],Fd_rsahrs[45],Fd_rsahrs[46]);
        AHRSData_Packet.Timestamp=timestamp(Fd_rsahrs[47],Fd_rsahrs[48],Fd_rsahrs[49],Fd_rsahrs[50]);   //时间戳
        //AHRSData2PC();
        }
    rs_ahrstype=0;
    }

    if(rs_imutype==1)
    {
        if(Fd_rsimu[1]==TYPE_IMU&&Fd_rsimu[2]==IMU_LEN)
        {
        IMUData_Packet.gyroscope_x=DATA_Trans(Fd_rsimu[7],Fd_rsimu[8],Fd_rsimu[9],Fd_rsimu[10]);  //角速度
        IMUData_Packet.gyroscope_y=DATA_Trans(Fd_rsimu[11],Fd_rsimu[12],Fd_rsimu[13],Fd_rsimu[14]);
        IMUData_Packet.gyroscope_z=DATA_Trans(Fd_rsimu[15],Fd_rsimu[16],Fd_rsimu[17],Fd_rsimu[18]);

        IMUData_Packet.accelerometer_x=DATA_Trans(Fd_rsimu[19],Fd_rsimu[20],Fd_rsimu[21],Fd_rsimu[22]);  //线加速度
        IMUData_Packet.accelerometer_y=DATA_Trans(Fd_rsimu[23],Fd_rsimu[24],Fd_rsimu[25],Fd_rsimu[26]);
        IMUData_Packet.accelerometer_z=DATA_Trans(Fd_rsimu[27],Fd_rsimu[28],Fd_rsimu[29],Fd_rsimu[30]);

        IMUData_Packet.magnetometer_x=DATA_Trans(Fd_rsimu[31],Fd_rsimu[32],Fd_rsimu[33],Fd_rsimu[34]);  //磁力计数据
        IMUData_Packet.magnetometer_y=DATA_Trans(Fd_rsimu[35],Fd_rsimu[36],Fd_rsimu[37],Fd_rsimu[38]);
        IMUData_Packet.magnetometer_z=DATA_Trans(Fd_rsimu[39],Fd_rsimu[40],Fd_rsimu[41],Fd_rsimu[42]);

        IMUData_Packet.Timestamp=timestamp(Fd_rsimu[55],Fd_rsimu[56],Fd_rsimu[57],Fd_rsimu[58]);   //时间戳
        //IMUData2PC();
        }
        rs_imutype=0;
    }
    return 0;
}

void* Thread_USB_MEMS(void*)//not use now
{
    int i;
    static uint8_t state_usb=0, rx_cnt_usb=0;
    static uint8_t _data_len2_usb = 0, _data_cnt2_usb = 0;
    int ret=0;
    int rx_length=42;
    uint8_t data_usb = 0;
    int iResult = -1;
    int fd = -1,iCommPort,iBaudRate,iDataSize,iStopBit;
    char cParity;
    int iLen;

    serial::Serial m_serial;
    m_serial.setPort( "/dev/ttyUSB0");
    m_serial.setBaudrate(921600);
    m_serial.setBytesize(serial::eightbits);
    m_serial.setParity(serial::parity_none);
    m_serial.setStopbits(serial::stopbits_one);
    m_serial.open();
    while(m_serial.isOpen()==0)
    {
        m_serial.open();
        usleep(100000);
    }
//------------------------------------------memery-------------------------------------
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int memory_update=0;
    int flag = 0;
    int link_cnt=0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();

    fd_set  rset;
    int  rv = -1 ;
    int nread=0;
    struct timeval timeout;
    timeout.tv_sec=0;
    timeout.tv_usec=1000;

    static char Count=0;
    static char rs_count=0;
    static char last_rsnum=0;
    static char rsimu_flag=0;
    static char rsacc_flag=0;
    int att_cal=0,gyro_cal=0;
    //read bias
    spi_rx.att_usb_bias[0]=config_hardware["mems_param"]["att_bias"][0].as<float>();//
    spi_rx.att_usb_bias[1]=config_hardware["mems_param"]["att_bias"][1].as<float>();//
    spi_rx.att_usb_bias[2]=config_hardware["mems_param"]["att_bias"][2].as<float>();//
    spi_rx.att_rate_usb_bias[0]=config_hardware["mems_param"]["gyro_bias"][0].as<float>();//
    spi_rx.att_rate_usb_bias[1]=config_hardware["mems_param"]["gyro_bias"][1].as<float>();//
    spi_rx.att_rate_usb_bias[2]=config_hardware["mems_param"]["gyro_bias"][2].as<float>();//

    while (1)
    {
        sys_dt = Get_Cycle_T(20);
        mems_usb_loss_cnt += sys_dt;
        if (mems_usb_loss_cnt > 1.5&& mems_usb_connect==1)
        {
            mems_usb_loss_cnt = 0;
            mems_usb_connect = 0;
            printf("Hardware::Hardware USB-MEMS Loss!!!\n");
        }

        nread = m_serial.available();
        if (nread>0){
            //printf(" \n read_cefore=%d\n",nread);
            m_serial.read(buff, nread);
        }

        for (int i = 0; i < nread; i++)
        {
            //printf("%02x ",buff[i]);
            Fd_data[Count]=buff[i];  //串口数据填入数组
            if(((last_rsnum==FRAME_END)&&(buff[i] == FRAME_HEAD))||Count>0)
                {
                rs_count=1;
                Count++;

                mems_usb_loss_cnt=0;
                if(!mems_usb_connect){
                     printf("Hardware::Hardware USB-MEMS Connect-In!!!\n");
                     mems_usb_connect=1;
                }

                if((Fd_data[1]==TYPE_IMU)&&(Fd_data[2]==IMU_LEN))
                   { rsimu_flag=1;rsacc_flag=0;}
                else if((Fd_data[1]==TYPE_AHRS)&&(Fd_data[2]==AHRS_LEN))
                   { rsacc_flag=1;rsimu_flag=0;}
                }
                else
                    Count=0;
                last_rsnum=buff[i];

            if(rsimu_flag==1 && Count==IMU_RS) //将本帧数据保存至Fd_rsimu数组中
            {
                Count=0;
                rsimu_flag=0;
                rs_imutype=1;
                if(Fd_data[IMU_RS-1]==FRAME_END) //帧尾校验
                memcpy(Fd_rsimu, Fd_data, sizeof(Fd_data));
            }
            else if(rsacc_flag==1 && Count==AHRS_RS) //
            {
                Count=0;
                rsacc_flag=0;
                rs_ahrstype=1;
                if(Fd_data[AHRS_RS-1]==FRAME_END)
                memcpy(Fd_rsahrs, Fd_data, sizeof(Fd_data));
            }
      }

      TTL_Hex2Dec();

      //cal imu
      if(att_cal==0&&(mems.Acc_CALIBRATE))
          att_cal=1;
      //printf("%d %d\n",att_cal,mems.Acc_CALIBRATE);
      if(att_cal==1){
        att_cal=2;
        spi_rx.att_usb_bias[0]=spi_rx.att_usb[0];
        spi_rx.att_usb_bias[1]=spi_rx.att_usb[1];
        config_hardware["mems_param"]["att_bias"][0]=spi_rx.att_usb_bias[0];//
        config_hardware["mems_param"]["att_bias"][1]=spi_rx.att_usb_bias[1];//
        config_hardware["mems_param"]["att_bias"][2]=spi_rx.att_usb_bias[2];//
        std::ofstream fout_hard("/home/odroid/Tinker/Param/param_hardware.yaml");
        fout_hard<<config_hardware;
        fout_hard.close();
      }

      if(att_cal==2&&mems.Acc_CALIBRATE==0&&mems.Gyro_CALIBRATE==0){
          printf("Hardware::IMU Mems Att bias is [%f] [%f] [%f]\n",spi_rx.att_usb_bias[0],spi_rx.att_usb_bias[1],spi_rx.att_usb_bias[2]);
          att_cal=0;
      }

      if(gyro_cal==0&&(mems.Gyro_CALIBRATE))
          gyro_cal=1;

      if(gyro_cal==1){
        gyro_cal=2;
        spi_rx.att_rate_usb_bias[0]=spi_rx.att_rate_usb[0];
        spi_rx.att_rate_usb_bias[1]=spi_rx.att_rate_usb[1];
        spi_rx.att_rate_usb_bias[2]=spi_rx.att_rate_usb[2];
        config_hardware["mems_param"]["gyro_bias"][0]=spi_rx.att_rate_usb_bias[0];//
        config_hardware["mems_param"]["gyro_bias"][1]=spi_rx.att_rate_usb_bias[1];//
        config_hardware["mems_param"]["gyro_bias"][2]=spi_rx.att_rate_usb_bias[2];//
        std::ofstream fout_hard("/home/odroid/Tinker/Param/param_hardware.yaml");
        fout_hard<<config_hardware;
        fout_hard.close();
      }
      if(gyro_cal==2&&mems.Acc_CALIBRATE==0&&mems.Gyro_CALIBRATE==0){
          printf("Hardware::IMU Mems Gyro bias is [%f] [%f] [%f]\n",spi_rx.att_rate_usb_bias[0],spi_rx.att_rate_usb_bias[1],spi_rx.att_rate_usb_bias[2]);
          gyro_cal=0;
      }
      usleep(2*1000);
    }
    close(fd);

    return 0;
}

void* Thread_USB_MEMS1(void*)//not use now
{
    mems_usb_connect=1;
    test_uart();
    return 0;
}



int main(int argc, char *argv[])
{
    int use_usb_imu=config_hardware["mems_param"]["imu_usb_enable"].as<float>();//
    printf("use_usb_imu=%d\n",use_usb_imu);
    //get local IP
    local_ip=getLocalIP();

    cout<<"getLocalIP:"<<local_ip<<endl;

#if NO_THREAD&&!EN_MULTI_THREAD&&!USE_USB
    Thread_ALL();
#else
    pthread_t tida, tidb,tidc;
    pthread_mutex_init(&lock, NULL);
    #if EN_MULTI_THREAD&&!USE_USB
    pthread_create(&tida, NULL, Thread_Mem, NULL);
    pthread_create(&tidb, NULL, Thread_SPI, NULL);
    if(use_usb_imu)
        //pthread_create(&tidc, NULL, Thread_USB_MEMS, NULL);
        pthread_create(&tidc, NULL, Thread_USB_MEMS1, NULL);
    pthread_join(tida, NULL);
    pthread_join(tidb, NULL);
    if(use_usb_imu)
        pthread_join(tidc, NULL);
    #else
#if !USE_USB
    pthread_create(&tida, NULL, Thread_ALL, NULL);
    pthread_join(tida, NULL);
#else
    pthread_create(&tidb, NULL, Thread_Mem, NULL);
    pthread_create(&tidc, NULL, Thread_USB, NULL);
#endif
#if EN_MULTI_THREAD
    pthread_join(tidb, NULL);
    pthread_join(tidc, NULL);
#endif
    #endif
#endif
    return 1;
}
