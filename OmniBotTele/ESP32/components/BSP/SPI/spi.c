/**
 ****************************************************************************************************
 * @file        spi.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       SPI驱动代码
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

#include "spi.h"


/**
 * @brief       初始化SPI
 * @param       无
 * @retval      无
 */
void spi2_init(void)
{
    esp_err_t ret = 0;
    spi_bus_config_t spi_bus_conf = {0};

    /* SPI总线配置 */
    spi_bus_conf.miso_io_num = SPI_MISO_GPIO_PIN;                               /* SPI_MISO引脚 */
    spi_bus_conf.mosi_io_num = SPI_MOSI_GPIO_PIN;                               /* SPI_MOSI引脚 */
    spi_bus_conf.sclk_io_num = SPI_CLK_GPIO_PIN;                                /* SPI_SCLK引脚 */
    spi_bus_conf.quadwp_io_num = -1;                                            /* SPI写保护信号引脚，该引脚未使能 */
    spi_bus_conf.quadhd_io_num = -1;                                            /* SPI保持信号引脚，该引脚未使能 */
    spi_bus_conf.max_transfer_sz = 320 * 240 * 2;                               /* 配置最大传输大小，以字节为单位 */
    
    /* 初始化SPI总线 */
    ret = spi_bus_initialize(SPI2_HOST, &spi_bus_conf, SPI_DMA_CH_AUTO);        /* SPI总线初始化 */
    ESP_ERROR_CHECK(ret);                                                       /* 校验参数值 */
}

/**
 * @brief       SPI发送命令
 * @param       handle : SPI句柄
 * @param       cmd    : 要发送命令
 * @retval      无
 */
void spi2_write_cmd(spi_device_handle_t handle, uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t = {0};

    t.length = 8;                                       /* 要传输的位数 一个字节 8位 */
    t.tx_buffer = &cmd;                                 /* 将命令填充进去 */
    ret = spi_device_polling_transmit(handle, &t);      /* 开始传输 */
    ESP_ERROR_CHECK(ret);                               /* 一般不会有问题 */
}

/**
 * @brief       SPI发送数据
 * @param       handle : SPI句柄
 * @param       data   : 要发送的数据
 * @param       len    : 要发送的数据长度 
 * @retval      无
 */
void spi2_write_data(spi_device_handle_t handle, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t = {0};

    if (len == 0)
    {
        return;                                     /* 长度为0 没有数据要传输 */
    }

    t.length = len * 8;                             /* 要传输的位数 一个字节 8位 */
    t.tx_buffer = data;                             /* 将命令填充进去 */
    ret = spi_device_polling_transmit(handle, &t);  /* 开始传输 */
    ESP_ERROR_CHECK(ret);                           /* 一般不会有问题 */
}

/**
 * @brief       SPI处理数据
 * @param       handle       : SPI句柄
 * @param       data         : 要发送的数据 
 * @retval      t.rx_data[0] : 接收到的数据
 */
uint8_t spi2_transfer_byte(spi_device_handle_t handle, uint8_t data)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.tx_data[0] = data;
    spi_device_transmit(handle, &t);

    return t.rx_data[0];
}
