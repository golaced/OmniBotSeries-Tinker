#ifndef __ROBOT_H
#define __ROBOT_H
#include <stdio.h>
#include <string.h>
#include "wifi_config.h"
#include "ui.h"
#include "ui_helpers.h"
#if 1
    extern lv_obj_t *label;
    extern const char *full_text;
    extern char display_text[64];  // 用于存放增量显示的文本
    extern int current_index;  // 当前显示的字符索引
    extern lv_obj_t *label;
    extern lv_obj_t *tab1;
    extern void increment_text(void);
#endif

#define LV_COLOR_RED        lv_color_hex(0xFF0000)  // 红色
#define LV_COLOR_GREEN      lv_color_hex(0x00FF00)  // 绿色
#define LV_COLOR_BLUE       lv_color_hex(0x0000FF)  // 蓝色
#define LV_COLOR_BLACK      lv_color_hex(0x000000)  // 黑色
#define LV_COLOR_WHITE      lv_color_hex(0xFFFFFF)  // 白色
#define LV_COLOR_GRAY       lv_color_hex(0x808080)  // 灰色
#define LV_COLOR_YELLOW     lv_color_hex(0xFFFF00)  // 黄色
#define LV_COLOR_PURPLE     lv_color_hex(0x800080)  // 紫色
#define LV_COLOR_CYAN       lv_color_hex(0x00FFFF)  // 青色
#define LV_COLOR_MAGENTA    lv_color_hex(0xFF00FF)  // 品红
#define LV_COLOR_ORANGE     lv_color_hex(0xFFA500)  // 橙色
#define LV_COLOR_LIME       lv_color_hex(0x00FF00)  // 亮绿色

typedef struct
{
    char connect;
    int loss_cnt;
    char gait_mode;
    char auto_mode;
    char head_mode;
    char step_mode;
    char heart_cnt;
    float att[3];
    float vel[3];
    float acc[3];
    float gyro[3];
    float mag[3];
    float baro;
    float pos[3];
    float q_leg[14];   
    float q_arm[14];       
    esp_ip4_addr_t ip;                                                                   
} _robot_fb;

extern _robot_fb robot_fb;

typedef struct
{
    char gait_mode;
    char auto_mode;
    char head_mode;
    char step_mode;
    char cal_mode;
    char cal_sel;
    float att[3];
    float vel[3];
    float pos[3];    
    int key_rc[4];
    int power_cnt;                                                                     
} _robot_cmd;

extern _robot_cmd robot_cmd;

typedef struct
{
    char wifi_connected;
    char udp_connected;
    char keyboard_input_sel;
    char cal_leg_trigger;
    char cal_leg_sel;
    char cal_arm_trigger;                                                                   
    char cal_arm_sel;       
    char cal_imu_trigger;                                                                   
    char cal_imu_sel;                                                              
    char ip_buf[20];
} _robot_lvgl;

extern _robot_lvgl robot_lvgl;

#if 0
    /* 链接wifi名称 */
    #define DEFAULT_SSID        "Tinker-2.4G-3"
    #define DEFAULT_PWD         "11111111"
    /* 需要自己设置远程IP地址 */
    #define IP_ADDR   "192.168.1.11" //目标机IP地址
    #define LWIP_DEMO_PORT               8080    /* 目标端口 */
    #define LWIP_DEMO_RX_BUFSIZE         1024    /* 最大接收数据长度 */
#else
    /* 链接wifi名称 */
    #define DEFAULT_SSID        "LBm"
    #define DEFAULT_PWD         "11111111"
    /* 需要自己设置远程IP地址 */
    #define IP_ADDR   "192.168.1.11" //目标机IP地址
    #define LWIP_DEMO_PORT               7070    /* 目标端口 */
    #define LWIP_DEMO_RX_BUFSIZE         1024    /* 最大接收数据长度 */
#endif

#endif
