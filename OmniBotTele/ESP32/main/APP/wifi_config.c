/**
 ****************************************************************************************************
 * @file        wifi_config.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-26
 * @brief       网络连接配置
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

#include "wifi_config.h"
#include "robot.h"
/* 事件标志 */
static EventGroupHandle_t   wifi_event;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define MAX_WIFI_TRAY       2
network_connet_info network_connet;
static const char *TAG = "static_ip";
char lcd_buff[100] = {0};

/* WIFI默认配置 */
#define WIFICONFIG()   {                            \
    .sta = {                                        \
        .ssid = DEFAULT_SSID,                       \
        .password = DEFAULT_PWD,                    \
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,   \
    },                                              \
}

/* 存储12个WIFI名称 */
#define DEFAULT_SCAN_LIST_SIZE  6
static const char *TAG_S = "scan";
/**
 * @brief       身份认证模式
 * @param       authmode :身份验证模式
 * @retval      无
 */
static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_OWE:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_OWE");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_ENTERPRISE:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA3_ENT_192:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA3_ENT_192");
        break;
    case WIFI_AUTH_WPA3_EXT_PSK:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA3_EXT_PSK");
        break;
    case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE");
        break;
    default:
        ESP_LOGI(TAG_S, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}
/**
 * @brief       打印WIFI密码类型
 * @param       pairwise_cipher :密码类型
 * @param       group_cipher    :群密码类型
 * @retval      无
 */
static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher)
    {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        default:
            ESP_LOGI(TAG_S, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }

    switch (group_cipher)
    {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        default:
            ESP_LOGI(TAG_S, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }
}
  
void connet_display(uint8_t flag)
{
    // if((flag & 0x80) == 0x80)
    // {
    //     lcd_color_fill(0,90,320,240,WHITE);
    //     sprintf(lcd_buff, "ssid:%s",DEFAULT_SSID);
    //     lcd_show_string(0, 90, 240, 16, 16, lcd_buff, BLUE);
    //     sprintf(lcd_buff, "psw:%s",DEFAULT_PWD);
    //     lcd_show_string(0, 110, 240, 16, 16, lcd_buff, BLUE);
    //     lcd_show_string(0, 130, 200, 16, 16, "KEY0:Send data", MAGENTA);
    // }
    // else if ((flag & 0x04) == 0x04)
    // {
    //     lcd_show_string(0, 90, 240, 16, 16, "wifi connecting......", BLUE);
    // }
    // else if ((flag & 0x02) == 0x02)
    // {
    //     lcd_show_string(0, 90, 240, 16, 16, "wifi connecting fail", BLUE);
    // }
    // else if ((flag & 0x01) == 0x01)
    // {
    //     lcd_show_string(0, 150, 200, 16, 16, (char*)network_connet.ip_buf, MAGENTA);
    // }

    network_connet.connet_state &= 0x00;
}

/**
 * @brief       WIFI链接糊掉函数
 * @param       arg:传入网卡控制块
 * @param       event_base:WIFI事件
 * @param       event_id:事件ID
 * @param       event_data:事件数据
 * @retval      无
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static int s_retry_num = 0;

    /* 扫描到要连接的WIFI事件 */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        network_connet.connet_state |= 0x04;
        network_connet.fun(network_connet.connet_state);
        esp_wifi_connect();
    }
    /* 连接WIFI事件 */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        network_connet.connet_state |= 0x80;
        network_connet.fun(network_connet.connet_state);
    }
    /* 连接WIFI失败事件 */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        network_connet.connet_state |= 0x02;
        /* 尝试连接 */
        if (s_retry_num < MAX_WIFI_TRAY)
        {
            esp_wifi_connect();
            s_retry_num ++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(wifi_event, WIFI_FAIL_BIT);
            network_connet.fun(network_connet.connet_state);
        }

        ESP_LOGI(TAG,"connect to the AP fail");
    }
    /* 工作站从连接的AP获得IP */
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        network_connet.connet_state |= 0x01;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "static ip:" IPSTR, IP2STR(&event->ip_info.ip));
        
        s_retry_num = 0;
        sprintf(network_connet.ip_buf, "static ip:" IPSTR, IP2STR(&event->ip_info.ip));
        sprintf(robot_lvgl.ip_buf,IPSTR, IP2STR(&event->ip_info.ip));
        network_connet.fun(network_connet.connet_state);
        xEventGroupSetBits(wifi_event, WIFI_CONNECTED_BIT);

    }
}

/**
 * @brief       WIFI初始化
 * @param       无
 * @retval      无
 */
void copy_wifi_config(wifi_config_t *wifi_config, const wifi_config_t *input_wifi) {
    // 复制 SSID
    strncpy((char *)wifi_config->sta.ssid, (const char *)input_wifi->sta.ssid, sizeof(wifi_config->sta.ssid));
    wifi_config->sta.ssid[sizeof(wifi_config->sta.ssid) - 1] = '\0'; // 确保以空字符结尾

    // 复制密码
    strncpy((char *)wifi_config->sta.password, (const char *)input_wifi->sta.password, sizeof(wifi_config->sta.password));
    wifi_config->sta.password[sizeof(wifi_config->sta.password) - 1] = '\0'; // 确保以空字符结尾
}

void wifi_sta_init(wifi_config_t input_wifi)
{
   static esp_netif_t *sta_netif = NULL;
    network_connet.connet_state = 0x00;
    network_connet.fun = connet_display;

    wifi_event= xEventGroupCreate();    /* 创建一个事件标志组 */
    /* 网卡初始化 */
    ESP_ERROR_CHECK(esp_netif_init());
    /* 创建新的事件循环 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif= esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL) );
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));    
    wifi_config_t  wifi_config = WIFICONFIG();
    copy_wifi_config(&wifi_config, &input_wifi);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 等待链接成功后、ip生成 */
    EventBits_t bits = xEventGroupWaitBits( wifi_event,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);

    /* 判断连接事件 */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
        robot_lvgl.wifi_connected = 1;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
        robot_lvgl.wifi_connected = 2;
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        robot_lvgl.wifi_connected = false;
    }

    vEventGroupDelete(wifi_event);
}
