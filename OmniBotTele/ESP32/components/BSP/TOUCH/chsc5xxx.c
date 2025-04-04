/**
 ****************************************************************************************************
 * @file        CHSC5XXX.c
 * @author      正点原子团队(正点原子)
 * @version     V1.1
 * @date        2023-12-1
 * @brief       2.4寸电容触摸屏-CHSC5xxx 驱动代码
 *
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

#include "chsc5xxx.h"


const char* touch_name = "touch.c";

/* 默认支持的触摸屏点数(5点触摸) */
uint8_t g_chsc_tnum = 5;
uint8_t temp[4];

/**
 * @brief       向chsc5xxx写入数据
 * @param       reg : 起始寄存器地址
 * @param       buf : 数据缓缓存区
 * @param       len : 写数据长度
 * @retval      esp_err_t ：0, 成功; 1, 失败;
 */
esp_err_t chsc5xxx_wr_reg(uint32_t reg, uint8_t *buf, unsigned int len)
{
    uint8_t data[4];        /* 存储32位地址和数据 */
    /* 将32位地址和数据分解为字节 */
    data[0] = (reg >> 24) & 0xFF; /* ADDR[31:24] */
    data[1] = (reg >> 16) & 0xFF; /* ADDR[23:16] */
    data[2] = (reg >> 8) & 0xFF;  /* ADDR[15:8] */
    data[3] = reg & 0xFF;         /* ADDR[7:0] */

    /* 创建I²C传输命令 */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    /* 发送启动信号 */
    i2c_master_start(cmd);
    /* 发送设备地址，写操作 */
    i2c_master_write_byte(cmd, (CHSC5432_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    /* 发送32位地址的各个部分 */
    i2c_master_write(cmd, data, 4, ACK_CHECK_EN); /* 发送4个字节的地址 */
    /* 发送数据 */
    i2c_master_write(cmd, buf, len, ACK_CHECK_EN);
    /* 发送停止信号 */
    i2c_master_stop(cmd);
    /* 执行命令 */
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    /* 删除命令链 */
    i2c_cmd_link_delete(cmd);
    //printf("chsc5xxx_wr_reg ret:%d\n",ret);
    return ret;
}

/**
 * @brief       从chsc5xxx读出数据
 * @param       reg : 起始寄存器地址
 * @param       buf : 数据缓缓存区
 * @param       len : 读数据长度
 * @retval      esp_err_t ：0, 成功; 1, 失败;
 */
esp_err_t chsc5xxx_rd_reg(uint32_t reg, uint8_t *buf, unsigned int len)
{
    uint8_t data[4];                /* 存储32位地址和数据 */
    /* 将32位地址和数据分解为字节 */
    data[0] = (reg >> 24) & 0xFF;   /* ADDR[31:24] */
    data[1] = (reg >> 16) & 0xFF;   /* ADDR[23:16] */
    data[2] = (reg >> 8) & 0xFF;    /* ADDR[15:8] */
    data[3] = reg & 0xFF;           /* ADDR[7:0] */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    /* 发送寄存器地址 */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CHSC5432_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK); /* 写入模式 */
    /* 发送32位地址的各个部分 */
    i2c_master_write(cmd, data, 4, ACK_CHECK_EN); /* 发送4个字节的地址 */
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY); 
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    /* 重新启动并读取数据 */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CHSC5432_ADDR << 1) | I2C_MASTER_READ, I2C_MASTER_ACK); /* 读取模式 */

    if (len > 1)
    {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }

    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    /* 执行命令 */
    ret |= i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);
    //printf("chsc5xxx_rd_reg ret:%d\n",ret);
    return ret;
}
extern int semi_touch_init();
/**
 * @brief       初始化chsc5xxx触摸屏
 * @param       无
 * @retval      0, 初始化成功; 1, 初始化失败;
 */
uint8_t chsc5xxx_init(void)
{   
    /* 复位触摸屏 */
    CT_RST(0);
    vTaskDelay(100);
    CT_RST(1);
    vTaskDelay(100);

    chsc5xxx_rd_reg(CHSC5XXX_PID_REG, temp, 4);             /* 从0x20000080地址读取ID */
    temp[3] = 0;
    //ESP_LOGI(touch_name, "CTP:0x%x", temp[0]);
    return 0;
}

/**
 * @brief       扫描触摸屏(采用查询方式)
 * @param       mode : 电容屏未用到次参数, 为了兼容电阻屏
 * @retval      当前触屏状态
 *   @arg       0, 触屏无触摸; 
 *   @arg       1, 触屏有触摸;
 */
uint8_t chsc5xxx_scan(uint8_t mode)
{
    uint8_t buf[28];
    uint8_t i = 0;
    uint8_t res = 0;
    uint16_t temp;
    uint16_t tempsta;
    static uint8_t t = 0;                            /* 控制查询间隔,从而降低CPU占用率 */
    t++;

    if ((t % 5) == 0 || t < 5)                       /* 空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率 */
    {
        chsc5xxx_rd_reg(CHSC5XXX_CTRL_REG, buf, 28); /* 官方建议一次读取28字节，然后在分析触摸坐标 */
        mode = 0x80 + buf[1];
        //ESP_LOGI(touch_name, "mode:%d", mode);

        /* 判断是否有触摸按下 */
        if ((mode & 0XF) && ((mode & 0XF) <= g_chsc_tnum))
        {
            temp = 0XFFFF << (mode & 0XF);
            tempsta = tp_dev.sta;
            tp_dev.sta = (~temp) | TP_PRES_DOWN | TP_CATH_PRES;
            tp_dev.x[g_chsc_tnum - 1] = tp_dev.x[0];
            tp_dev.y[g_chsc_tnum - 1] = tp_dev.y[0];
            /* 获取触摸坐标 */
            for (i = 0; i < g_chsc_tnum; i++)
            {
                if (tp_dev.sta & (1 << i))
                {
                    if (tp_dev.touchtype & 0X01)                        /* 横屏 */
                    {
                        tp_dev.x[i] = ((uint16_t)(buf[5 + i * 5] >> 4 ) << 8) + buf[3 + i * 5];
                        tp_dev.y[i] = lcd_dev.height - (((uint16_t)(buf[5 + i * 5] & 0x0F) << 8) + buf[2 + i * 5]);
                    }
                    else                                                /* 竖屏 */
                    {
                        tp_dev.x[i] = ((uint16_t)(buf[5 + i * 5] & 0x0F) << 8) + buf[2 + i * 5];
                        tp_dev.y[i] = ((uint16_t)((buf[5 + i * 5] & 0xF0) >> 4 ) << 8) + buf[3 + i * 5];
                    }

                    // ESP_LOGI(touch_name, "x[%d]:%d, y[%d]:%d", tp_dev.x[i], i, tp_dev.y[i]);
                }
            }
            res = 1;
            /* 触摸坐标超出屏幕范围 */
            if (tp_dev.x[0] > lcd_dev.width || tp_dev.y[0] > lcd_dev.height)
            {
                /* 触摸数量是否大于1~5 */
                if ((mode & 0XF) > 1)
                {
                    tp_dev.x[0] = tp_dev.x[1];
                    tp_dev.y[0] = tp_dev.y[1];
                    t = 0;
                }
                else    /* 超出范围且未检测到触摸点 */
                {
                    tp_dev.x[0] = tp_dev.x[g_chsc_tnum - 1];
                    tp_dev.y[0] = tp_dev.y[g_chsc_tnum - 1];
                    mode = 0X80;
                    tp_dev.sta = tempsta;
                }
            }
            else 
            {
                t = 0;
            }
        }
    }

    //printf("x[0]:%d,y[0]:%d\r\n", tp_dev.x[0], tp_dev.y[0]);
    
    /* 超出触摸范围 */
    if ((mode & 0X8F) == 0X80)
    {
        if (tp_dev.sta & TP_PRES_DOWN)
        {
            tp_dev.sta &= ~TP_PRES_DOWN;
        }
        else
        {
            tp_dev.x[0] = 0xffff;
            tp_dev.y[0] = 0xffff;
            tp_dev.sta &= 0XE000;
        }
    }

    if (t > 240)
    {
        t = 10;
    }

    return res;
}
