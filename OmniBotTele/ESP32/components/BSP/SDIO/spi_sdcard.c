/**
 ****************************************************************************************************
 * @file        spi_sdcard.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       SD卡 驱动代码
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

#include "spi_sdcard.h"


spi_device_handle_t MY_SD_Handle = NULL;                            /* SD卡句柄 */
sdmmc_card_t *card;                                                 /* SD / MMC卡结构 */
const char mount_point[] = MOUNT_POINT;                             /* 挂载点/根目录 */
esp_err_t ret = ESP_OK;
esp_err_t mount_ret = ESP_OK;

/**
 * @brief       SD卡初始化
 * @param       无
 * @retval      esp_err_t
 */
esp_err_t sd_spi_init(void)
{
    if (MY_SD_Handle != NULL)                                       /* 再一次挂载或者初始化SD卡 */
    {
        spi_bus_remove_device(MY_SD_Handle);                        /* 移除SPI上的SD卡设备 */

        if (mount_ret == ESP_OK)
        {
            esp_vfs_fat_sdcard_unmount(mount_point, card);          /* 取消挂载 */
        }
    }

    /* SPI驱动接口配置,SPISD卡时钟是20-25MHz */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,                         /* SPI时钟 */
        .mode = 0,                                                  /* SPI模式0 */
        .spics_io_num = SD_NUM_CS,                                  /* 片选引脚 */
        .queue_size = 7,                                            /* 事务队列尺寸 7个 */
    };

    /* 添加SPI总线设备 */
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &MY_SD_Handle);

    /* 文件系统挂载配置 */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,                            /* 如果挂载失败：true会重新分区和格式化/false不会重新分区和格式化 */
        .max_files = 5,                                             /* 打开文件最大数量 */
        .allocation_unit_size = 4 * 1024 * sizeof(uint8_t)          /* 硬盘分区簇的大小 */
    };

    /* SD卡参数配置 */
    sdmmc_host_t host = {0};
    host.flags = SDMMC_HOST_FLAG_SPI | SDMMC_HOST_FLAG_DEINIT_ARG;  /* 定义主机属性的标志：SPI协议且可调用deinit函数 */
    host.slot = SPI2_HOST;                                          /* 使用SPI2端口 */
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;                         /* 主机支持的最大频率：20000 */
    host.io_voltage = 3.3f;                                         /* 控制器使用的I/O电压 */
    host.init = &sdspi_host_init;                                   /* 用于初始化驱动程序的主机函数 */
    host.set_bus_width = NULL;                                      /* 设置总线宽度的主机功能 */
    host.get_bus_width = NULL;                                      /* 取总线宽度的主机函数 */
    host.set_bus_ddr_mode = NULL;                                   /* 设置DDR模式的主机功能 */
    host.set_card_clk = &sdspi_host_set_card_clk;                   /* 设置板卡时钟频率的主机函数 */
    host.do_transaction = &sdspi_host_do_transaction;               /* 执行事务的主机函数 */
    host.deinit_p = &sdspi_host_remove_device;                      /* 用于取消初始化驱动程序的主机函数 */
    host.io_int_enable = &sdspi_host_io_int_enable;                 /* 启用SDIO中断线的主机功能 */
    host.io_int_wait = &sdspi_host_io_int_wait;                     /* 等待SDIO中断线路激活的主机功能 */
    host.command_timeout_ms = 0;                                    /* 超时，默认为0*/
    
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
    host.get_real_freq = &sdspi_host_get_real_freq;
    host.set_cclk_always_on = NULL;
#endif

    /* SD卡引脚配置 */
    sdspi_device_config_t slot_config = {0};
    slot_config.host_id   = host.slot;
    slot_config.gpio_cs   = SD_NUM_CS;
    slot_config.gpio_cd   = GPIO_NUM_NC;
    slot_config.gpio_wp   = GPIO_NUM_NC;
    slot_config.gpio_int  = GPIO_NUM_NC;

    mount_ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);      /* 挂载文件系统 */
    ret |= mount_ret;

    return ret;
}

/**
 * @brief       获取SD卡相关信息
 * @param       out_total_bytes：总大小
 * @param       out_free_bytes：剩余大小
 * @retval      无
 */
void sd_get_fatfs_usage(size_t *out_total_bytes, size_t *out_free_bytes)
{
    // FATFS *fs;
    // size_t free_clusters;
    // int res = f_getfree("0:", (size_t *)&free_clusters, &fs);
    // assert(res == FR_OK);
    // size_t total_sectors = (fs->n_fatent - 2) * fs->csize;
    // size_t free_sectors = free_clusters * fs->csize;

    // size_t sd_total = total_sectors / 1024;
    // size_t sd_total_KB = sd_total * fs->ssize;
    // size_t sd_free = free_sectors / 1024;
    // size_t sd_free_KB = sd_free*fs->ssize;

    // /* 假设总大小小于4GiB，对于SPI Flash应该为true */
    // if (out_total_bytes != NULL)
    // {
    //     *out_total_bytes = sd_total_KB;
    // }
    
    // if (out_free_bytes != NULL)
    // {
    //     *out_free_bytes = sd_free_KB;
    // }
}
