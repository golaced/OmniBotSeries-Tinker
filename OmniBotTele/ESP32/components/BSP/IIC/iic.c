/**
 ****************************************************************************************************
 * @file        iic.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       IIC驱动代码
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

#include "iic.h"


i2c_obj_t iic_master[I2C_NUM_MAX];  /* 为IIC0和IIC1分别定义IIC控制块结构体 */
const char * iic_name = "iic.c";

/**
 * @brief       初始化IIC
 * @param       iic_port：I2C编号(I2C_NUM_0 / I2C_NUM_1)
 * @retval      IIC控制块0 / IIC控制块1
 */
i2c_obj_t iic_init(uint8_t iic_port)
{
    uint8_t i;
    i2c_config_t iic_config_struct = {0};

    if (iic_port == I2C_NUM_0)
    {
        i = 0;
    }
    else
    {
        i = 1;
    }
    
    iic_master[i].port = iic_port;
    iic_master[i].init_flag = ESP_FAIL;

    if (iic_master[i].port == I2C_NUM_0)
    {
        iic_master[i].scl = IIC0_SCL_GPIO_PIN;
        iic_master[i].sda = IIC0_SDA_GPIO_PIN;
    }
    else
    {
        iic_master[i].scl = IIC1_SCL_GPIO_PIN;
        iic_master[i].sda = IIC1_SDA_GPIO_PIN;
    }

    iic_config_struct.mode = I2C_MODE_MASTER;                               /* 设置IIC模式-主机模式 */
    iic_config_struct.sda_io_num = iic_master[i].sda;                       /* 设置IIC_SDA引脚 */
    iic_config_struct.scl_io_num = iic_master[i].scl;                       /* 设置IIC_SCL引脚 */
    iic_config_struct.sda_pullup_en = GPIO_PULLUP_ENABLE;                   /* 配置IIC_SDA引脚上拉使能 */
    iic_config_struct.scl_pullup_en = GPIO_PULLUP_ENABLE;                   /* 配置IIC_SCL引脚上拉使能 */
    iic_config_struct.master.clk_speed = IIC_FREQ;                          /* 设置IIC通信速率 */
    i2c_param_config(iic_master[i].port, &iic_config_struct);               /* 设置IIC初始化参数 */

    /* 激活I2C控制器的驱动 */
    iic_master[i].init_flag = i2c_driver_install(iic_master[i].port,        /* 端口号 */
                                                 iic_config_struct.mode,    /* 主机模式 */
                                                 I2C_MASTER_RX_BUF_DISABLE, /* 从机模式下接收缓存大小(主机模式不使用) */
                                                 I2C_MASTER_TX_BUF_DISABLE, /* 从机模式下发送缓存大小(主机模式不使用) */     
                                                 0);                        /* 用于分配中断的标志(通常从机模式使用) */            

    if (iic_master[i].init_flag != ESP_OK)
    {
        while(1)
        {
            ESP_LOGI(iic_name,"%s , ret: %d", __func__, iic_master[i].init_flag);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    return iic_master[i];
}

/**
 * @brief       IIC读写数据
 * @param       self：设备控制块
 * @param       addr：设备地址
 * @param       n   ：数据大小
 * @param       bufs：要发送的数据或者是读取的存储区
 * @param       flags：读写标志位
 * @retval      无
 */
esp_err_t i2c_transfer(i2c_obj_t *self, uint16_t addr, size_t n, i2c_buf_t *bufs, unsigned int flags)
{
    int data_len = 0;
    esp_err_t ret = ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                                       /* 创建一个命令链接,将一系列待发送给从机的数据填充命令链接 */

    /* 根据器件通信时序去决定flags参数,进而选择如下代码不同的执行情况 */
    if (flags & I2C_FLAG_WRITE)
    {
        ESP_ERROR_CHECK(i2c_master_start(cmd));                                                         /* 启动位 */
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr << 1, ACK_CHECK_EN));                           /* 从机地址 + 写操作位 */
        ESP_ERROR_CHECK(i2c_master_write(cmd, bufs->buf, bufs->len, ACK_CHECK_EN));                     /* len个数据 */
        data_len += bufs->len; 
        --n;
        ++bufs;
    }

    ESP_ERROR_CHECK(i2c_master_start(cmd));                                                             /* 启动位 */
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr << 1 | (flags & I2C_FLAG_READ), ACK_CHECK_EN));     /* 从机地址 + 读/写操作位 */

    for (; n--; ++bufs)
    {
        if (flags & I2C_FLAG_READ)
        {
            ESP_ERROR_CHECK(i2c_master_read(cmd, bufs->buf, bufs->len, n == 0 ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK)); /* 读取数据 */
        }
        else
        {
            if (bufs->len != 0)
            {
                ESP_ERROR_CHECK(i2c_master_write(cmd, bufs->buf, bufs->len, ACK_CHECK_EN));             /* len个数据 */
            }
        }
        data_len += bufs->len;
    }

    if (flags & I2C_FLAG_STOP)
    {
        ESP_ERROR_CHECK(i2c_master_stop(cmd));                                                          /* 停止位 */
    }

    ret = i2c_master_cmd_begin(self->port, cmd, 100 * (1 + data_len) / portTICK_PERIOD_MS);             /* 触发I2C控制器执行命令链接,即命令发送 */
    i2c_cmd_link_delete(cmd);                                                                           /* 释放命令链接使用的资源 */

    return ret;
}
