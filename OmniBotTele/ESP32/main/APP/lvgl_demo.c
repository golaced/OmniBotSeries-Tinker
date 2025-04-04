/**
 ****************************************************************************************************
 * @file        lvgl_demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       LVGL SPI Flash读取XBF字库
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32S3 BOX 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "lvgl_demo.h"
#include "led.h"
#include "key.h"
#include "usart.h"
#include "lcd.h"
#include "touch.h"
#include "spi_sdcard.h"
#include "fonts.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "lwip_demo.h"
#include "lv_mainstart.h"
#include "ui.h"
#include "ui_helpers.h"
#include "robot.h"
#include "usart.h"
_robot_fb robot_fb;
_robot_cmd robot_cmd;
_robot_lvgl robot_lvgl;
joy adc_rc[2];
char key_rc[2];
char key_sel[4];
/* LV_DEMO_TASK 任务 配置
 * 包括: 任务优先级 堆栈大小 任务句柄 创建任务
 */
#define LV_DEMO_TASK_PRIO   2               /* 任务优先级 */
#define LV_DEMO_STK_SIZE    5 * 1024        /* 任务堆栈大小 */
TaskHandle_t LV_DEMOTask_Handler;           /* 任务句柄 */
void lv_demo_task(void *pvParameters);      /* 任务函数 */

/* LED_TASK 任务 配置
 * 包括: 任务优先级 堆栈大小 任务句柄 创建任务
 */
#define LED_TASK_PRIO       1               /* 任务优先级 */
#define LED_STK_SIZE        5 * 1024        /* 任务堆栈大小 */
TaskHandle_t LEDTask_Handler;               /* 任务句柄 */
void led_task(void *pvParameters);          /* 任务函数 */

/**
 * @brief       lvgl_demo入口函数
 * @param       无
 * @retval      无
 */
void lvgl_demo(void)
{
    uint8_t key = 0;

    lv_init();              /* 初始化LVGL图形库 */
    lv_port_disp_init();    /* lvgl显示接口初始化,放在lv_init()的后面 */
    lv_port_indev_init();   /* lvgl输入接口初始化,放在lv_init()的后面 */

    /* 为LVGL提供时基单元 */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 1 * 1000));
    
    // while (sd_spi_init())       /* 检测不到SD卡 */
    // {
    //     lcd_show_string(30, 130, 200, 32, 32, "SD Card Error!", RED);
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     lcd_show_string(30, 130, 200, 32, 32, "Please Check! ", RED);
    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }
    /* 创建LVGL任务 */
    xTaskCreatePinnedToCore((TaskFunction_t )lv_demo_task,          /* 任务函数 */
                            (const char*    )"lv_demo_task",        /* 任务名称 */
                            (uint16_t       )LV_DEMO_STK_SIZE,      /* 任务堆栈大小 */
                            (void*          )NULL,                  /* 传入给任务函数的参数 */
                            (UBaseType_t    )LV_DEMO_TASK_PRIO,     /* 任务优先级 */
                            (TaskHandle_t*  )&LV_DEMOTask_Handler,  /* 任务句柄 */
                            (BaseType_t     ) 0);                   /* 该任务哪个内核运行 */

    /* 创建LED测试任务 */
    xTaskCreatePinnedToCore((TaskFunction_t )led_task,              /* 任务函数 */
                            (const char*    )"led_task",            /* 任务名称 */
                            (uint16_t       )LED_STK_SIZE,          /* 任务堆栈大小 */
                            (void*          )NULL,                  /* 传入给任务函数的参数 */
                            (UBaseType_t    )LED_TASK_PRIO,         /* 任务优先级 */
                            (TaskHandle_t*  )&LEDTask_Handler,      /* 任务句柄 */
                            (BaseType_t     ) 0);                   /* 该任务哪个内核运行 */
    
    //lwip_demo();//udp demo 测试用例，需要注释掉上面的LVGL DEMO，否则会冲突     
}

/**
 * @brief       LVGL运行例程
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */

void update_label_text(lv_obj_t * ui_Label,float value) {
    char buf[32];  // 创建一个足够大的字符数组来存储浮点数文本
    snprintf(buf, sizeof(buf), "%.1f", value);  // 格式化浮点数并保留两位小数
    lv_label_set_text(ui_Label, buf);           // 更新标签文本
}


void update_label_ip(lv_obj_t * ui_Label,char* buf) {
    //char buf[32];  // 创建一个足够大的字符数组来存储浮点数文本
    //snprintf(buf, sizeof(buf), IPSTR, IP2STR(robot.ip_info));  // 格式化浮点数并保留两位小数
    lv_label_set_text(ui_Label, buf);           // 更新标签文本
}


void update_label_mode(lv_obj_t * ui_Label,char mode) {
    char buf[32];  // 创建一个足够大的字符数组来存储浮点数文本
    switch(mode){
        case 0:
            snprintf(buf, sizeof(buf), "%s", "IDLE");  // 格式化浮点数并保留两位小数
        break;
        case 1:
            snprintf(buf, sizeof(buf), "%s", "STAND");  // 格式化浮点数并保留两位小数
        break;
        case 2:
            snprintf(buf, sizeof(buf), "%s", "TROT");  // 格式化浮点数并保留两位小数
        break;
        default:
            snprintf(buf, sizeof(buf), "%s", "NULL");  // 格式化浮点数并保留两位小数    
        break;
    }
    lv_label_set_text(ui_Label, buf);           // 更新标签文本
}

void update_label_auto_mode(lv_obj_t * ui_Label,char mode) {
    char buf[32];  // 创建一个足够大的字符数组来存储浮点数文本
    switch(mode){
        case 0:
            snprintf(buf, sizeof(buf), "%s", "NULL");  // 格式化浮点数并保留两位小数
        break;
        case 1:
            snprintf(buf, sizeof(buf), "%s", "TRACK");  // 格式化浮点数并保留两位小数
        break;
        case 2:
            snprintf(buf, sizeof(buf), "%s", "SLAM");  // 格式化浮点数并保留两位小数
        break;
        default:
            snprintf(buf, sizeof(buf), "%s", "NULL");  // 格式化浮点数并保留两位小数    
        break;
    }
    lv_label_set_text(ui_Label, buf);           // 更新标签文本
}

char TxBuffer3[256];
char TxCounter3=0;
char count3=0; 
char Rx_Buf3[256];	
char RxBuffer3[255];
char RxState3 = 0;
char RxBufferNum3 = 0;
char RxBufferCnt3 = 0;
char RxLen3 = 0;
static char _data_len3 = 0,_data_cnt3 = 0;
void Data_ExtCAN_FB(char *data_buf,char num)
{   
    static int cnt_p=0;
	char sum = 0,sum_rx=0;
	char i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	sum_rx=*(data_buf+num-1);
	if(!(sum==*(data_buf+num-1)))		return;		
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		
    if(*(data_buf+2)==0x01)//FB1
    { 
        adc_rc[0].connect=1;
        adc_rc[0].loss_cnt=0;
        adc_rc[0].bat_percent=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
		adc_rc[0].x=((float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))-1000)/500.;
        adc_rc[0].y=((float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))-1000)/500.;
        adc_rc[1].x=((float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))-1000)/500.;
        adc_rc[1].y=((float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))-1000)/500.;
        key_rc[0]=*(data_buf+14);
        key_rc[1]=*(data_buf+15);
        key_sel[0]=*(data_buf+16);
        key_sel[1]=*(data_buf+17);
        key_sel[2]=*(data_buf+18);
        key_sel[3]=*(data_buf+19);
        #if 0
        if(cnt_p++>20){
            cnt_p=0;
            printf("rc=%.1f %.1f %.1f %.1f\n",adc_rc[0].x,adc_rc[0].y,adc_rc[1].x,adc_rc[1].y);
            printf("key=%d %d| %d %d %d %d\n",key_rc[0],key_rc[1],key_sel[0],key_sel[1],key_sel[2],key_sel[3]);
        }
        #endif
	}			
}
void USART1_IRQHandler(char com_data)//extcan
{ 
    if(RxState3==0&&com_data==0xAA)
    {
        RxState3=1;
        RxBuffer3[0]=com_data;
    }
    else if(RxState3==1&&com_data==0xAF)
    {
        RxState3=2;
        RxBuffer3[1]=com_data;
    }
    else if(RxState3==2&&com_data>0&&com_data<0XF1)
    {
        RxState3=3;
        RxBuffer3[2]=com_data;
    }
    else if(RxState3==3&&com_data<80)
    {
        RxState3 = 4;
        RxBuffer3[3]=com_data;
        _data_len3 = com_data;
        _data_cnt3 = 0;
    }
    else if(RxState3==4&&_data_len3>0)
    {
        _data_len3--;
        RxBuffer3[4+_data_cnt3++]=com_data;
        if(_data_len3==0)
            RxState3 = 5;
    }
    else if(RxState3==5)
    {
        RxState3 = 0;
        RxBuffer3[4+_data_cnt3]=com_data;
        Data_ExtCAN_FB(RxBuffer3,_data_cnt3+5);
    }
    else
        RxState3 = 0;
}

void lv_demo_task(void *pvParameters)//doghome
{
    const char* uart = "uart";
    esp_err_t ret;
    uint8_t len = 0;
    uint16_t times = 0;
    unsigned char data[RX_BUF_SIZE] = {0};
    int wifi_connected_flag = 0;
    pvParameters = pvParameters;
    float timer=0,timer_gait=0;
    //lv_mainstart();     /* 测试的demo */
    ui_init();
    usart_init(256000);    
    float timer_rst[10]={0};
    float dt=0.005;
    float timer_rc=0;
    while (1)
    {
        timer+=dt;
        if(robot_fb.loss_cnt++>100){
            robot_fb.connect = 0;   
        }
        if(adc_rc[0].loss_cnt++>100){
            adc_rc[0].connect = 0;   
        }
       
        adc_rc[0].loss_cnt=0;
        if(robot_lvgl.wifi_connected==1&&!wifi_connected_flag){
            wifi_connected_flag=1;
            printf("wifi_connected_flag\n");
            _ui_screen_change(&ui_ScreenloginGood, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, &ui_ScreenloginGood_screen_init);
        }
        if(robot_lvgl.wifi_connected==2&&!wifi_connected_flag){
            wifi_connected_flag=2;
            printf("wifi_fail_flag\n");
            _ui_screen_change(&ui_ScreenloginFail, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, &ui_ScreenloginFail_screen_init);
        }

        if(robot_cmd.cal_mode!=0){
            timer_rst[0]+=dt;
            if(timer_rst[0]>0.5){
                timer_rst[0]=0;
                robot_cmd.cal_mode=0;
                printf("robot_cmd.cal_mode rest\n");
            }
        }
        //按键
        uint8_t key = key_scan(0);      /* 获取键值 */
        //printf("key=%d\n",key);
        
        //uart串口
        timer_rc+=dt;
        if(timer_rc>0.01){
            timer_rc=0;
            uart_get_buffered_data_len(USART_UX, (size_t*) &len);                           /* 获取环形缓冲区数据长度 */
            //printf("len=%d\n",len); 
            if (len > 0)                                                                    /* 判断数据长度 */
            {
                memset(data, 0, RX_BUF_SIZE);                                               /* 对缓冲区清零 */
                uart_read_bytes(USART_UX, data, len, 30);                                  /* 读数据 */
                for(int i=0;i<len;i++)
                    USART1_IRQHandler(data[i]);
                #if 0
                    printf("Received %d bytes: ", len);
                    for (int i = 0; i < len; i++)
                    {
                        printf("0x%02x ", data[i]); // 以十六进制格式打印每个字节
                    }
                    printf("\n");  
                #endif
            }

            if(adc_rc[0].connect){
                robot_cmd.vel[0]=adc_rc[0].x*0.5;
                robot_cmd.vel[1]=adc_rc[0].y*0.5;
                robot_cmd.att[0]=adc_rc[1].x*0.5;
                robot_cmd.att[1]=adc_rc[1].y*0.5;
                robot_cmd.key_rc[0]=key_sel[3];
                robot_cmd.key_rc[1]=key_sel[2];
                robot_cmd.key_rc[2]=key_sel[1];
                robot_cmd.key_rc[3]=key_sel[0];
            }
        }

        //----------------LVGL
        lv_style_t style;
        lv_style_init(&style); // 初始化样式结构体 
        //修改按键背景颜色
        //printf("robot_fb.connec=%d loss=%d\n",robot_fb.connect,robot_fb.loss_cnt);
        #if 1
        if(robot_fb.connect==1){
            if(adc_rc[0].connect){
                lv_obj_set_style_bg_color(ui_ButtonLight, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight1, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight2, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight3, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight3, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");
                lv_obj_set_style_bg_color(ui_ButtonLight4, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");
                lv_obj_set_style_bg_color(ui_ButtonLight5, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight6, LV_COLOR_GREEN, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");//printf("WIFI connected\n");
            }
            else{
                lv_obj_set_style_bg_color(ui_ButtonLight, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight1, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight2, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight3, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight3, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");
                lv_obj_set_style_bg_color(ui_ButtonLight4, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");
                lv_obj_set_style_bg_color(ui_ButtonLight5, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_ButtonLight6, LV_COLOR_RED, LV_PART_MAIN | LV_STATE_DEFAULT);//p
            } //printf("WIFI connected\n");
        }else{
            lv_obj_set_style_bg_color(ui_ButtonLight, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(ui_ButtonLight1, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(ui_ButtonLight2, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(ui_ButtonLight3, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(ui_ButtonLight3, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");
            lv_obj_set_style_bg_color(ui_ButtonLight4, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);//printf("WIFI connected\n");
            lv_obj_set_style_bg_color(ui_ButtonLight5, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(ui_ButtonLight6, LV_COLOR_GRAY, LV_PART_MAIN | LV_STATE_DEFAULT);//pri
        }
        
        //腿部标定
        update_label_text(ui_LabelL01,robot_fb.att[0]);//更新数据doghome LVGL相关更新需要在本线程
        update_label_text(ui_LabelL02,robot_fb.att[1]);
        update_label_text(ui_LabelL03,robot_fb.att[2]);

        update_label_text(ui_LabelL11,robot_fb.vel[0]);
        update_label_text(ui_LabelL12,robot_fb.vel[1]);
        update_label_text(ui_LabelL13,robot_fb.vel[2]);

        update_label_text(ui_LabelL21,robot_fb.q_leg[0]);
        update_label_text(ui_LabelL22,robot_fb.q_leg[1]);
        update_label_text(ui_LabelL23,robot_fb.q_leg[2]);       
        update_label_text(ui_LabelL24,robot_fb.q_leg[3]);
        update_label_text(ui_LabelL25,robot_fb.q_leg[4]);
        update_label_text(ui_LabelL26,robot_fb.q_leg[5]);
        update_label_text(ui_LabelL27,robot_fb.q_leg[6]);

        update_label_text(ui_LabelL31,robot_fb.q_leg[7]);
        update_label_text(ui_LabelL32,robot_fb.q_leg[8]);
        update_label_text(ui_LabelL33,robot_fb.q_leg[9]);       
        update_label_text(ui_LabelL34,robot_fb.q_leg[10]);
        update_label_text(ui_LabelL35,robot_fb.q_leg[11]);
        update_label_text(ui_LabelL36,robot_fb.q_leg[12]);
        update_label_text(ui_LabelL37,robot_fb.q_leg[13]);

        update_label_ip(ui_LabelIP,robot_lvgl.ip_buf);
        update_label_mode(ui_LabelMShow,robot_fb.gait_mode);
        update_label_auto_mode(ui_LabelGPTshow,robot_fb.auto_mode);

        //机械臂标定
        update_label_text(ui_LabelA01,robot_fb.att[0]);//更新数据doghome LVGL相关更新需要在本线程
        update_label_text(ui_LabelA02,robot_fb.att[1]);
        update_label_text(ui_LabelA03,robot_fb.att[2]);

        update_label_text(ui_LabelA11,robot_fb.vel[0]);
        update_label_text(ui_LabelA12,robot_fb.vel[1]);
        update_label_text(ui_LabelA13,robot_fb.vel[2]);

        update_label_text(ui_LabelA21,robot_fb.q_arm[0]);
        update_label_text(ui_LabelA22,robot_fb.q_arm[1]);
        update_label_text(ui_LabelA23,robot_fb.q_arm[2]);       
        update_label_text(ui_LabelA24,robot_fb.q_arm[3]);
        update_label_text(ui_LabelA25,robot_fb.q_arm[4]);
        update_label_text(ui_LabelA26,robot_fb.q_arm[5]);
        update_label_text(ui_LabelA27,robot_fb.q_arm[6]);

        update_label_text(ui_LabelA31,robot_fb.q_arm[7]);
        update_label_text(ui_LabelA32,robot_fb.q_arm[8]);
        update_label_text(ui_LabelA33,robot_fb.q_arm[9]);       
        update_label_text(ui_LabelA34,robot_fb.q_arm[10]);
        update_label_text(ui_LabelA35,robot_fb.q_arm[11]);
        update_label_text(ui_LabelA36,robot_fb.q_arm[12]);
        update_label_text(ui_LabelA37,robot_fb.q_arm[13]);

        //传感器标定
        update_label_text(ui_LabelS01,robot_fb.att[0]);
        update_label_text(ui_LabelS02,robot_fb.att[1]);
        update_label_text(ui_LabelS03,robot_fb.att[2]);
        update_label_text(ui_LabelS11,robot_fb.acc[0]);
        update_label_text(ui_LabelS12,robot_fb.acc[1]);
        update_label_text(ui_LabelS13,robot_fb.acc[2]);
        update_label_text(ui_LabelS21,robot_fb.gyro[0]);
        update_label_text(ui_LabelS22,robot_fb.gyro[1]);
        update_label_text(ui_LabelS23,robot_fb.gyro[2]);
        update_label_text(ui_LabelS31,robot_fb.mag[0]);
        update_label_text(ui_LabelS32,robot_fb.mag[1]);
        update_label_text(ui_LabelS33,robot_fb.mag[2]);
        update_label_text(ui_LabelS41,robot_fb.baro);
      
        if(timer>0.1){
            timer=0;
            increment_text();//增量更新数据文本  GPT doghome
        }
        #endif 
        //----------------
        lv_timer_handler();             /* LVGL计时器 */
        vTaskDelay(pdMS_TO_TICKS(5));  /* 延时5毫秒 */
    }
}

/**
 * @brief       led_task, 程序运行指示灯
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void led_task(void *pvParameters)
{
    pvParameters = pvParameters;
    float timer=0;
    while(1)
    {
        if(robot_lvgl.udp_connected){
            lwip_send_dir();//启动后会阻塞
            LED_TOGGLE();
        }
        vTaskDelay(pdMS_TO_TICKS(10));  /* 延时1000毫秒 */
    }
}

/**
 * @brief       初始化并注册显示设备
 * @param       无
 * @retval      无
 */
void lv_port_disp_init(void)
{
    void *buf1 = NULL;
    void *buf2 = NULL;

    /* 初始化显示设备LCD */
    lcd_cfg_t lcd_config_info = {0};
    lcd_config_info.notify_flush_ready = NULL;
    lcd_init(lcd_config_info);          /* 初始化LCD */

    /*-----------------------------
     * 创建一个绘图缓冲区
     *----------------------------*/
    /**
     * LVGL 需要一个缓冲区用来绘制小部件
     * 随后，这个缓冲区的内容会通过显示设备的 'flush_cb'(显示设备刷新函数) 复制到显示设备上
     * 这个缓冲区的大小需要大于显示设备一行的大小
     *
     * 这里有3种缓冲配置:
     * 1. 单缓冲区:
     *      LVGL 会将显示设备的内容绘制到这里，并将他写入显示设备。
     *
     * 2. 双缓冲区:
     *      LVGL 会将显示设备的内容绘制到其中一个缓冲区，并将他写入显示设备。
     *      需要使用 DMA 将要显示在显示设备的内容写入缓冲区。
     *      当数据从第一个缓冲区发送时，它将使 LVGL 能够将屏幕的下一部分绘制到另一个缓冲区。
     *      这样使得渲染和刷新可以并行执行。
     *
     * 3. 全尺寸双缓冲区
     *      设置两个屏幕大小的全尺寸缓冲区，并且设置 disp_drv.full_refresh = 1。
     *      这样，LVGL将始终以 'flush_cb' 的形式提供整个渲染屏幕，您只需更改帧缓冲区的地址。
     */
    /* 使用双缓冲 */
    buf1 = heap_caps_malloc(lcd_dev.width * 60 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    buf2 = heap_caps_malloc(lcd_dev.width * 60 * sizeof(lv_color_t), MALLOC_CAP_DMA);

    /* 初始化显示缓冲区 */
    static lv_disp_draw_buf_t disp_buf;                                 /* 保存显示缓冲区信息的结构体 */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, lcd_dev.width * 60);   /* 初始化显示缓冲区 */
    
    /* 在LVGL中注册显示设备 */
    static lv_disp_drv_t disp_drv;      /* 显示设备的描述符(HAL要注册的显示驱动程序、与显示交互并处理与图形相关的结构体、回调函数) */
    lv_disp_drv_init(&disp_drv);        /* 初始化显示设备 */
    
    /* 设置显示设备的分辨率 
     * 这里为了适配正点原子的多款屏幕，采用了动态获取的方式，
     * 在实际项目中，通常所使用的屏幕大小是固定的，因此可以直接设置为屏幕的大小 
     */
    disp_drv.hor_res = lcd_dev.width;
    disp_drv.ver_res = lcd_dev.height;

    /* 用来将缓冲区的内容复制到显示设备 */
    disp_drv.flush_cb = lvgl_disp_flush_cb;

    /* 设置显示缓冲区 */
    disp_drv.draw_buf = &disp_buf;

    disp_drv.user_data = panel_handle;

    /* 注册显示设备 */
    lv_disp_drv_register(&disp_drv);
}

/**
 * @brief       初始化并注册输入设备
 * @param       无
 * @retval      无
 */
void lv_port_indev_init(void)
{
    /* 初始化触摸屏 */
    tp_dev.init();

    /* 初始化输入设备 */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);

    /* 配置输入设备类型 */
    indev_drv.type = LV_INDEV_TYPE_POINTER;

    /* 设置输入设备读取回调函数 */
    indev_drv.read_cb = touchpad_read;

    /* 在LVGL中注册驱动程序，并保存创建的输入设备对象 */
    lv_indev_t *indev_touchpad;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/**
* @brief    将内部缓冲区的内容刷新到显示屏上的特定区域
* @note     可以使用 DMA 或者任何硬件在后台加速执行这个操作
*           但是，需要在刷新完成后调用函数 'lv_disp_flush_ready()'
* @param    disp_drv : 显示设备
* @param    area : 要刷新的区域，包含了填充矩形的对角坐标
* @param    color_map : 颜色数组
* @retval   无
*/
static void lvgl_disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    /* 特定区域打点 */
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);

    /* 重要!!! 通知图形库，已经刷新完毕了 */
    lv_disp_flush_ready(drv);
}

/**
 * @brief       告诉LVGL运行时间
 * @param       arg : 传入参数(未用到)
 * @retval      无
 */
static void increase_lvgl_tick(void *arg)
{
    /* 告诉LVGL已经过了多少毫秒 */
    lv_tick_inc(1);
}

/**
 * @brief       获取触摸屏设备的状态
 * @param       无
 * @retval      返回触摸屏设备是否被按下
 */
static bool touchpad_is_pressed(void)
{
    tp_dev.scan(0);     /* 触摸按键扫描 */

    if (tp_dev.sta & TP_PRES_DOWN)
    {
        return true;
    }

    return false;
}


/**
 * @brief       在触摸屏被按下的时候读取 x、y 坐标
 * @param       x   : x坐标的指针
 * @param       y   : y坐标的指针
 * @retval      无
 */
static void touchpad_get_xy(lv_coord_t *x, lv_coord_t *y)
{
    (*x) = tp_dev.x[0];
    (*y) = tp_dev.y[0];
}

/**
 * @brief       图形库的触摸屏读取回调函数
 * @param       indev_drv   : 触摸屏设备
 * @param       data        : 输入设备数据结构体
 * @retval      无
 */
void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;

    /* 保存按下的坐标和状态 */
    if(touchpad_is_pressed())
    {
        touchpad_get_xy(&last_x, &last_y);  /* 在触摸屏被按下的时候读取 x、y 坐标 */
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    /* 设置最后按下的坐标 */
    data->point.x = last_x;
    data->point.y = last_y;
}