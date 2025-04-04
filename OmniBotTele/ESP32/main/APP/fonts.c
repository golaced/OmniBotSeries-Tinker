/**
 ****************************************************************************************************
 * @file        font.c(参考基础工程汉字显示实验)
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       字库 代码
 *              提供fonts_update_font和fonts_init用于字库更新和初始化
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

#include "fonts.h"
#include "lcd.h"
#include <string.h>


/* 字库区域占用的总扇区数大小(1个字库font24,字库大小=1019287字节,约占249个扇区,一个扇区4K字节) */
#define FONTSECSIZE         249
/* 每次操作限制在 4K 之内 */
#define SECTOR_SIZE         0X1000
/* 分区表字库存放起始地址 */
#define FONTINFOADDR        0

/* 用来保存字库基本信息，地址，大小等 */
_font_info ftinfo;

static const char *TAG = "storage_partition";
const esp_partition_t *storage_partition;

/* 字库存放在磁盘中的路径 */
char *const FONT_GBK_PATH[1] =
{
    // "/SYSTEM/FONT/UNIGBK.BIN",      /* UNIGBK.BIN的存放位置 */
    // "/SYSTEM/FONT/GBK12.FON",       /* GBK12的存放位置 */
    // "/SYSTEM/FONT/GBK16.FON",       /* GBK16的存放位置 */
    // "/SYSTEM/FONT/GBK24.FON",       /* GBK24的存放位置 */
    // "/SYSTEM/LVFONT/Font12.BIN",    /* Font12的存放位置 */
    "/SYSTEM/LVFONT/Font24.BIN",       /* Font24的存放位置 */
    // "/SYSTEM/LVFONT/Font36.BIN",    /* Font36的存放位置 */
};

/* 更新时的提示信息 */
char *const FONT_UPDATE_REMIND_TBL[1] =
{
    // "Updating UNIGBK.BIN",          /* 提示正在更新UNIGBK.bin */
    // "Updating GBK12.FON ",          /* 提示正在更新GBK12 */
    // "Updating GBK16.FON ",          /* 提示正在更新GBK16 */
    // "Updating GBK24.FON ",          /* 提示正在更新GBK24 */
    // "Updating Font12.BIN",          /* 提示正在更新Font12 */
    "Updating Font24.BIN",             /* 提示正在更新Font24 */
    // "Updating Font36.BIN",          /* 提示正在更新Font36 */
};

/**
 * @brief       分区表读取数据
 * @param       buffer    : 读取数据的存储区
 * @param       offset    : 读取数据的起始地址
 * @param       length    : 读取大小
 * @retval      ESP_OK:表示成功;其他:表示失败
 */
esp_err_t fonts_partition_read(void *buffer, uint32_t offset, uint32_t length)
{
    esp_err_t err;

    if (buffer == NULL || (length > SECTOR_SIZE))
    {
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG");
        return ESP_ERR_INVALID_ARG;
    }

    err = esp_partition_read(storage_partition, offset, buffer, length);
    
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Flash read failed.");
        return err;
    }
    
    return err;
}

/**
 * @brief       分区表写入数据
 * @param       buffer    : 写入数据的存储区
 * @param       offset    : 写入数据的起始地址
 * @param       length    : 写入大小
 * @retval      ESP_OK:表示成功;其他:表示失败
 */
esp_err_t fonts_partition_write(void *buffer, uint32_t offset, uint32_t length)
{
    esp_err_t err;

    if (buffer == NULL || (length > SECTOR_SIZE))
    {
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG");
        return ESP_ERR_INVALID_ARG;
    }

    err = esp_partition_write(storage_partition, offset, buffer, length);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Flash write failed.");
        return err;
    }

    return err;
}

/**
 * @brief       擦除某个扇区
 * @param       offset    : 擦除起始地址
 * @retval      ESP_OK:表示成功;其他:表示失败
 */
esp_err_t fonts_partition_erase_sector(uint32_t offset)
{
    esp_err_t err;

    err = esp_partition_erase_range(storage_partition, offset, SECTOR_SIZE);
    
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Flash erase failed.");
        return err;
    }

    return err;
}

/**
 * @brief       显示当前字体更新进度
 * @param       x, y    : 坐标
 * @param       size    : 字体大小
 * @param       totsize : 整个文件大小
 * @param       pos     : 当前文件指针位置
 * @param       color   : 字体颜色
 * @retval      无
 */
void fonts_progress_show(uint16_t x, uint16_t y, uint8_t size, uint32_t totsize, uint32_t pos, uint16_t color)
{
    float prog;
    uint8_t t = 0XFF;
    prog = (float)pos / totsize;
    prog *= 100;

    if (t != prog)
    {
        lcd_show_string(x + 3 * size / 2, y, 16, size, size, "%", color);
        t = prog;

        if (t > 100) t = 100;

        lcd_show_num(x, y, t, 3, size, color);  /* 显示数值 */
    }
}

/**
 * @brief       更新某一个字库
 * @param       x, y    : 提示信息的显示地址
 * @param       size    : 提示信息字体大小
 * @param       fpath   : 字体路径
 * @param       fx      : 更新的内容
 *   @arg                 0, ungbk;
 *   @Arg                 1, gbk12;
 *   @arg                 2, gbk16;
 *   @arg                 3, gbk24;
 * @param       color   : 字体颜色
 * @retval      0, 成功; 其他, 错误代码;
 */
static uint8_t fonts_update_fontx(uint16_t x, uint16_t y, uint8_t size, uint8_t *fpath, uint8_t fx, uint16_t color)
{
    uint32_t flashaddr = 0;
    FIL *fftemp;
    uint8_t *tempbuf;
    uint8_t res;
    uint16_t bread;
    uint32_t offx = 0;
    uint8_t rval = 0;
    fftemp = (FIL *)malloc(sizeof(FIL));  /* 分配内存 */

    if (fftemp == NULL)rval = 1;

    tempbuf = malloc(4096);               /* 分配4096个字节空间 */

    if (tempbuf == NULL)rval = 1;

    res = f_open(fftemp, (const TCHAR *)fpath, FA_READ);

    if (res) rval = 2;   /* 打开文件失败 */

    if (rval == 0)
    {
        switch (fx)
        {
            // case 0: /* 更新 UNIGBK.BIN */
            //     ftinfo.ugbkaddr = FONTINFOADDR + sizeof(ftinfo);    /* 信息头之后，紧跟UNIGBK转换码表 */
            //     ftinfo.ugbksize = fftemp->obj.objsize;              /* UNIGBK大小 */
            //     flashaddr = ftinfo.ugbkaddr;
            //     break;

            // case 1: /* 更新 GBK12.BIN */
            //     ftinfo.f12addr = ftinfo.ugbkaddr + ftinfo.ugbksize; /* UNIGBK之后，紧跟GBK12字库 */
            //     ftinfo.gbk12size = fftemp->obj.objsize;             /* GBK12字库大小 */
            //     flashaddr = ftinfo.f12addr;                         /* GBK12的起始地址 */
            //     break;

            // case 2: /* 更新 GBK16.BIN */
            //     ftinfo.f16addr = ftinfo.f12addr + ftinfo.gbk12size; /* GBK12之后，紧跟GBK16字库 */
            //     ftinfo.gbk16size = fftemp->obj.objsize;             /* GBK16字库大小 */
            //     flashaddr = ftinfo.f16addr;                         /* GBK16的起始地址 */
            //     break;

            // case 3: /* 更新 GBK24.BIN */
            //     ftinfo.f24addr = ftinfo.f16addr + ftinfo.gbk16size; /* GBK16之后，紧跟GBK24字库 */
            //     ftinfo.gbk24size = fftemp->obj.objsize;             /* GBK24字库大小 */
            //     flashaddr = ftinfo.f24addr;                         /* GBK24的起始地址 */
            //     break;

            // case 0:/* 更新LVGL12.BIN */
            //     ftinfo.lvgl_12addr = ftinfo.f24addr + ftinfo.gbk24size;
            //     ftinfo.lvgl_12size = fftemp->obj.objsize;
            //     flashaddr = ftinfo.lvgl_12addr;
            //     break;

            case 0:/* 更新LVGL24.BIN */
                ftinfo.lvgl_24addr = FONTINFOADDR + sizeof(ftinfo);
                ftinfo.lvgl_24size = fftemp->obj.objsize;
                flashaddr = ftinfo.lvgl_24addr;
                break;

            // case 2:/* 更新LVGL36.BIN */
            //     ftinfo.lvgl_36addr=ftinfo.lvgl_24addr+ftinfo.lvgl_24size;
            //     ftinfo.lvgl_36size=fftemp->obj.objsize;
            //     flashaddr=ftinfo.lvgl_36addr;
            //     break;
        }

        while (res == FR_OK)   /* 死循环执行 */
        {
            res = f_read(fftemp, tempbuf, 1024, (UINT *)&bread);    /* 读取数据 */

            if (res != FR_OK) break;     /* 执行错误 */

            /* 写入的长度单位改为1024解决写入时候RGB屏幕会抖动 */
            fonts_partition_write(tempbuf, offx + flashaddr, bread);                /* 从0开始写入bread个数据 */
            offx += bread;
            fonts_progress_show(x, y, size, fftemp->obj.objsize, offx, color);      /* 进度显示 */

            if (bread != 1024) break;    /* 读完了 */
        }

        f_close(fftemp);
    }

    free(fftemp);     /* 释放内存 */
    free(tempbuf);    /* 释放内存 */
    return res;
}

/**
 * @brief       更新字体文件
 *   @note      所有字库一起更新(UNIGBK,GBK12,GBK16,GBK24)
 * @param       x, y    : 提示信息的显示地址
 * @param       size    : 提示信息字体大小
 * @param       src     : 字库来源磁盘
 *   @arg                 "0:", SD卡;
 *   @Arg                 "1:", FLASH盘
 * @param       color   : 字体颜色
 * @retval      0, 成功; 其他, 错误代码;
 */
uint8_t fonts_update_font(uint16_t x, uint16_t y, uint8_t size, uint8_t *src, uint16_t color)
{
    uint8_t *pname;
    uint32_t *buf;
    uint8_t res = 0;
    uint16_t i, j;
    FIL *fftemp;
    uint8_t rval = 0;
    res = 0XFF;
    ftinfo.fontok = 0XFF;
    pname = malloc(100);                  /* 申请100字节内存 */
    buf = malloc(4096);                   /* 申请4K字节内存 */
    fftemp = (FIL *)malloc(sizeof(FIL));  /* 分配内存 */

    if (buf == NULL || pname == NULL || fftemp == NULL)
    {
        free(fftemp);
        free(pname);
        free(buf);
        return 5;   /* 内存申请失败 */
    }

    for (i = 0; i < 1; i++)     /* 先查找文件LVGL24 是否正常 */
    {
        strcpy((char *)pname, (char *)src);                     /* copy src内容到pname */
        strcat((char *)pname, (char *)FONT_GBK_PATH[i]);        /* 追加具体文件路径 */
        res = f_open(fftemp, (const TCHAR *)pname, FA_READ);    /* 尝试打开 */

        if (res)
        {
            rval |= 1 << 7; /* 标记打开文件失败 */
            break;          /* 出错了,直接退出 */
        }
    }

    free(fftemp); /* 释放内存 */

    if (rval == 0)          /* 字库文件都存在. */
    {
        lcd_show_string(x, y, 300, size, size, "Erasing sectors... ", color);    /* 提示正在擦除扇区 */

        for (i = 0; i < FONTSECSIZE; i++)           /* 先擦除字库区域,提高写入速度 */
        {
            fonts_progress_show(x + 20 * size / 2, y, size, FONTSECSIZE, i, color);             /* 进度显示 */
            fonts_partition_read((uint8_t *)buf, ((FONTINFOADDR / 4096) + i) * 4096, 4096);     /* 读出整个扇区的内容 */

            for (j = 0; j < 1024; j++)              /* 校验数据 */
            {
                if (buf[j] != 0XFFFFFFFF) break;    /* 需要擦除 */
            }

            if (j != 1024)
            {
                fonts_partition_erase_sector(((FONTINFOADDR / 4096) + i) * 4096);   /* 需要擦除的扇区 */
            }
        }
        lcd_clear(WHITE);

        for (i = 0; i < 1; i++)     /* 更新LVGL24 */
        {
            lcd_show_string(x, y, 300, size, size, FONT_UPDATE_REMIND_TBL[i], color);
            strcpy((char *)pname, (char *)src);             /* copy src内容到pname */
            strcat((char *)pname, (char *)FONT_GBK_PATH[i]);/* 追加具体文件路径 */
            res = fonts_update_fontx(x + 20 * size / 2, y, size, pname, i, color);    /* 更新字库 */

            if (res)
            {
                free(buf);
                free(pname);
                return 1 + i;
            }
        }

        /* 全部更新好了 */
        ftinfo.fontok = 0XBB;
        fonts_partition_write((uint8_t *)&ftinfo, FONTINFOADDR, sizeof(ftinfo));    /* 保存字库信息 */
    }

    free(pname);    /* 释放内存 */
    free(buf);      /* 释放内存 */
    return rval;
}

/**
 * @brief       初始化字体
 * @param       无
 * @retval      0, 字库完好; 其他, 字库丢失;
 */
uint8_t fonts_init(void)
{
    uint8_t t = 0;

    storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    
    if (storage_partition == NULL)
    {
        ESP_LOGE(TAG, "Flash partition not found.");
        return 1;
    }
    
    while (t < 10)      /* 连续读取10次,都是错误,说明确实是有问题,得更新字库了 */
    {
        t++;
        fonts_partition_read((uint8_t *)&ftinfo, FONTINFOADDR, sizeof(ftinfo)); /* 读出ftinfo结构体数据 */

        if (ftinfo.fontok == 0XBB)
        {
            break;
        }
        
        vTaskDelay(20);
    }

    if (ftinfo.fontok != 0XBB)
    {
        return 1;
    }
    
    return 0;
}

