/**
 ****************************************************************************************************
 * @file        lcd.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       LCD(MCU屏) 驱动代码
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

#include "lcd.h"
#include "lcdfont.h"


static const char *TAG = "LCD";
esp_lcd_panel_handle_t panel_handle = NULL; /* LCD句柄 */
uint32_t g_back_color  = 0xFFFF;
lcd_obj_t lcd_dev;

/**
 * @brief       以一种颜色清空LCD屏
 * @param       color 清屏颜色
 * @retval      无
 */
void lcd_clear(uint16_t color)
{
    uint16_t *buffer = heap_caps_malloc(lcd_dev.width * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (uint16_t i = 0; i < lcd_dev.width ; i++)
        {
            buffer[i] = color;
        }
        
        for (uint16_t y = 0; y < lcd_dev.height; y++)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, lcd_dev.width, y + 1, buffer);
        }
        
        heap_caps_free(buffer);
    }
}

/**
 * @brief       在指定区域内填充指定颜色块
 * @note        此函数仅支持uint16_t,RGB565格式的颜色数组填充.
 *              (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
 *              注意:sx,ex,不能大于lcd_dev.width - 1; sy,ey,不能大于lcd_dev.height - 1
 * @param       sx,sy:起始坐标
 * @param       ex,ey:结束坐标
 * @param       color:填充的颜色数组首地址
 * @retval      无
 */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    /* 确保起始坐标小于结束坐标 */
    if (sx > ex || sy > ey)
    {
        return; /* 无效的填充区域，不执行填充 */
    }  
      
    /* 确保填充区域完全在LCD范围内 */
    sx = fmax(0, sx);
    sy = fmax(0, sy);
    ex = fmin(lcd_dev.width - 1, ex);
    ey = fmin(lcd_dev.height - 1, ey);
      
    /* 开始填充颜色 */
    for (int i = sx; i <= ex; i++)
    {
        for (int j = sy; j <= ey; j++)
        {
            /* 设置RGB值到LCD的对应位置 */
            lcd_draw_point(i, j, color);
        }
    }
}

/**
 * @brief       显示单色图标
 * @param       x,y,width,height:坐标及尺寸
 * @param       icosbase:点整位置
 * @param       color:画点的颜色
 * @param       bkcolor:背景色
 * @retval      无
 */
void lcd_app_show_mono_icos(uint16_t x,uint16_t y,uint8_t width,uint8_t height,uint8_t *icosbase,uint16_t color,uint16_t bkcolor)
{
    uint16_t rsize;
    uint16_t i,j;
    uint8_t temp;
    uint8_t t = 0;
    uint16_t x0 = x;                            /* 保留x的位置 */
    rsize = width / 8 + ((width % 8) ? 1 : 0);  /* 每行的字节数 */

    for (i = 0;i < rsize * height;i++)
    {
        temp = icosbase[i];

        for(j = 0;j < 8;j++)
        {
            if (temp & 0x80)
            {
                lcd_draw_point(x,y,color);
            }
            else
            {
                lcd_draw_point(x,y,bkcolor);
            }

            temp <<= 1;
            x++;
            t++;                                /* 宽度计数器 */

            if (t == width)
            {
                t=0;
                x=x0;
                y++;
                break;
            }
        }
    }
}

/**
 * @brief       画线
 * @param       x1,y1:起点坐标
 * @param       x2,y2:终点坐标
 * @param       color:线的颜色
 * @retval      无
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    delta_x = x2 - x1;      /* 计算坐标增量 */
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0)
    {
        incx = 1;           /* 设置单步方向 */
    }
    else if (delta_x == 0)
    {
        incx = 0;           /* 垂直线 */
    }
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
    {
        incy = 1;
    }
    else if (delta_y == 0)
    {
        incy = 0;            /* 水平线 */
    }
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y)
    {
        distance = delta_x; /* 选取基本增量坐标轴 */
    }
    else
    {
        distance = delta_y;
    }

    for (t = 0; t <= distance + 1; t++)     /* 画线输出 */
    {
        lcd_draw_point(row, col, color);   /* 画点 */
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * @brief       画一个矩形
 * @param       x1,y1   起点坐标
 * @param       x2,y2   终点坐标
 * @param       color 填充颜色
 * @retval      无
 */
void lcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color)
{
    lcd_draw_line(x0, y0, x1, y0, color);
    lcd_draw_line(x0, y0, x0, y1, color);
    lcd_draw_line(x0, y1, x1, y1, color);
    lcd_draw_line(x1, y0, x1, y1, color);
}

/**
 * @brief       画圆
 * @param       x0,y0:圆中心坐标
 * @param       r    :半径
 * @param       color:圆的颜色
 * @retval      无
 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);       /* 判断下个点位置的标志 */

    while (a <= b)
    {
        lcd_draw_point(x0 + a, y0 - b, color);  /* 5 */
        lcd_draw_point(x0 + b, y0 - a, color);  /* 0 */
        lcd_draw_point(x0 + b, y0 + a, color);  /* 4 */
        lcd_draw_point(x0 + a, y0 + b, color);  /* 6 */
        lcd_draw_point(x0 - a, y0 + b, color);  /* 1 */
        lcd_draw_point(x0 - b, y0 + a, color);
        lcd_draw_point(x0 - a, y0 - b, color);  /* 2 */
        lcd_draw_point(x0 - b, y0 - a, color);  /* 7 */
        a++;

        /* 使用Bresenham算法画圆 */
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

/**
 * @brief       在指定位置显示一个字符
 * @param       x,y  :坐标
 * @param       chr  :要显示的字符:" "--->"~"
 * @param       size :字体大小 12/16/24/32
 * @param       mode :叠加方式(1); 非叠加方式(0);
 * @param       color:字体颜色
 * @retval      无
 */
void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t temp, t1, t;
    uint16_t y0 = y;
    uint8_t csize = 0;
    uint8_t *pfont = 0;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2); /* 得到字体一个字符对应点阵集所占的字节数 */
    chr = (char)chr - ' ';      /* 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库） */

    switch (size)
    {
        case 12:
            pfont = (uint8_t *)asc2_1206[(uint8_t)chr];     /* 调用1206字体 */
            break;

        case 16:
            pfont = (uint8_t *)asc2_1608[(uint8_t)chr];     /* 调用1608字体 */
            break;

        case 24:
            pfont = (uint8_t *)asc2_2412[(uint8_t)chr];     /* 调用2412字体 */
            break;

        case 32:
            pfont = (uint8_t *)asc2_3216[(uint8_t)chr];     /* 调用3216字体 */
            break;

        default:
            return ;
    }


        for (t = 0; t < csize; t++)
        {
            temp = pfont[t];                                    /* 获取字符的点阵数据 */

            for (t1 = 0; t1 < 8; t1++)                          /* 一个字节8个点 */
            {
                if (temp & 0x80)                                /* 有效点,需要显示 */
                {
                    lcd_draw_point(x, y, color);               /* 画点出来,要显示这个点 */
                }
                else if (mode == 0)                             /* 无效点,不显示 */
                {
                    lcd_draw_point(x, y, g_back_color);        /* 画背景色,相当于这个点不显示(注意背景色由全局变量控制) */
                }

                temp <<= 1;                                     /* 移位, 以便获取下一个位的状态 */
                y++;

                if (y >= lcd_dev.height)return;                 /* 超区域了 */

                if ((y - y0) == size)                           /* 显示完一列了? */
                {
                    y = y0;                                     /* y坐标复位 */
                    x++;                                        /* x坐标递增 */

                    if (x >= lcd_dev.width)
                    {
                        return;                                 /* x坐标超区域了 */
                    }

                    break;
                }
            }
        }
}

/**
 * @brief       平方函数, m^n
 * @param       m:底数
 * @param       n:指数
 * @retval      m的n次方
 */
static uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
    {
        result *= m;
    }

    return result;
}

/**
 * @brief       显示len个数字
 * @param       x,y     :起始坐标
 * @param       num     :数值(0 ~ 2^32)
 * @param       len     :显示数字的位数
 * @param       size    :选择字体 12/16/24/32
 * @param       color   :字体颜色
 * @retval      无
 */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)                                               /* 按总显示位数循环 */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10;                      /* 获取对应位的数字 */

        if (enshow == 0 && t < (len - 1))                                   /* 没有使能显示,且还有位要显示 */
        {
            if (temp == 0)
            {
                lcd_show_char(x + (size / 2) * t, y, ' ', size, 0, color); /* 显示空格,占位 */
                continue;                                                   /* 继续下个一位 */
            }
            else
            {
                enshow = 1;                                                 /* 使能显示 */
            }
        }

        lcd_show_char(x + (size / 2) * t, y, temp + '0', size, 0, color);  /* 显示字符 */
    }
}

/**
 * @brief       扩展显示len个数字(高位是0也显示)
 * @param       x,y     :起始坐标
 * @param       num     :数值(0 ~ 2^32)
 * @param       len     :显示数字的位数
 * @param       size    :选择字体 12/16/24/32
 * @param       mode    :显示模式
 *              [7]:0,不填充;1,填充0.
 *              [6:1]:保留
 *              [0]:0,非叠加显示;1,叠加显示.
 * @param       color   :字体颜色
 * @retval      无
 */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)                                                               /* 按总显示位数循环 */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10;                                       /* 获取对应位的数字 */

        if (enshow == 0 && t < (len - 1))                                                   /* 没有使能显示,且还有位要显示 */
        {
            if (temp == 0)
            {
                if (mode & 0x80)                                                            /* 高位需要填充0 */
                {
                    lcd_show_char(x + (size / 2) * t, y, '0', size, mode & 0x01, color);    /* 用0占位 */
                }
                else
                {
                    lcd_show_char(x + (size / 2) * t, y, ' ', size, mode & 0x01, color);    /* 用空格占位 */
                }

                continue;
            }
            else
            {
                enshow = 1;                                                                 /* 使能显示 */
            }

        }

        lcd_show_char(x + (size / 2) * t, y, temp + '0', size, mode & 0x01, color);
    }
}

/**
 * @brief       显示字符串
 * @param       x,y         :起始坐标
 * @param       width,height:区域大小
 * @param       size        :选择字体 12/16/24/32
 * @param       p           :字符串首地址
 * @param       color       :字体颜色
 * @retval      无
 */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p, uint16_t color)
{
    uint8_t x0 = x;
    
    width += x;
    height += y;

    while ((*p <= '~') && (*p >= ' '))   /* 判断是不是非法字符! */
    {
        if (x >= width)
        {
            x = x0;
            y += size;
        }

        if (y >= height)
        {
            break;                       /* 退出 */
        }

        lcd_show_char(x, y, *p, size, 0, color);
        x += size / 2;
        p++;
    }
}

/**
 * @brief       LCD显示方向设置
 * @param       dir:0,竖屏；1,横屏
 * @retval      无
 */
void lcd_display_dir(uint8_t dir)
{
    lcd_dev.dir = dir;                                      /* 显示方向 */

    if (lcd_dev.dir == 0)                                   /* 竖屏 */
    {
        lcd_dev.width = lcd_dev.pheight;
        lcd_dev.height = lcd_dev.pwidth;
        esp_lcd_panel_swap_xy(panel_handle, false);         /* 交换X和Y轴 */ 
        esp_lcd_panel_mirror(panel_handle, false, false);   /* 对屏幕的Y轴进行镜像处理 */
    }
    else if (lcd_dev.dir == 1)                              /* 横屏 */
    {
        lcd_dev.width = lcd_dev.pwidth;
        lcd_dev.height = lcd_dev.pheight;
        esp_lcd_panel_swap_xy(panel_handle, true);          /* 不需要交换X和Y轴 */
        esp_lcd_panel_mirror(panel_handle, true, false);    /* 对屏幕的XY轴不进行镜像处理 */
    }
}

/**
 * @brief       lcd画点函数
 * @param       x,y     :写入坐标
 * @param       color   :颜色值
 * @retval      无
 */
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 1, y + 1, (uint16_t *)&color);
}

/**
 * @brief       LCD初始化
 * @param       lcd_config:LCD配置信息
 * @retval      无
 */
void lcd_init(lcd_cfg_t lcd_config)
{
    gpio_config_t gpio_init_struct = {0};
    esp_lcd_panel_io_handle_t io_handle = NULL;

    lcd_dev.wr = LCD_NUM_WR;                                        /* 配置WR引脚 */
    lcd_dev.cs = LCD_NUM_CS;                                        /* 配置CS引脚 */
    lcd_dev.dc = LCD_NUM_DC;                                        /* 配置DC引脚 */
    lcd_dev.rd = LCD_NUM_RD;                                        /* 配置RD引脚 */

    lcd_dev.pwidth = 320;                                           /* 面板宽度,单位:像素 */
    lcd_dev.pheight = 240;                                          /* 面板高度,单位:像素 */
    
    /* 配置RD引脚 */
    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;                 /* 失能引脚中断 */
    gpio_init_struct.mode = GPIO_MODE_INPUT_OUTPUT;                 /* 配置输出模式 */
    gpio_init_struct.pin_bit_mask = 1ull << lcd_dev.rd;             /* 配置引脚位掩码 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;          /* 失能下拉 */
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;               /* 使能下拉 */
    gpio_config(&gpio_init_struct);                                 /* 引脚配置 */
    gpio_set_level(lcd_dev.rd, 1);                                  /* RD管脚拉高 */

    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {                         /* 初始化80并口总线 */
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = lcd_dev.dc,
        .wr_gpio_num = lcd_dev.wr,
        .data_gpio_nums = {
            GPIO_LCD_D0,
            GPIO_LCD_D1,
            GPIO_LCD_D2,
            GPIO_LCD_D3,
            GPIO_LCD_D4,
            GPIO_LCD_D5,
            GPIO_LCD_D6,
            GPIO_LCD_D7,
        },
        .bus_width = 8,
        .max_transfer_bytes = lcd_dev.pwidth * lcd_dev.pheight * sizeof(uint16_t),
        .psram_trans_align = 64,
        .sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));    /* 新建80并口总线 */

    esp_lcd_panel_io_i80_config_t io_config = {                     /* 80并口配置 */
        .cs_gpio_num = lcd_dev.cs,
        .pclk_hz = (10 * 1000 * 1000),
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = 1,
        },
        .on_color_trans_done = lcd_config.notify_flush_ready,
        .user_ctx = lcd_config.user_ctx,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);                                  /* 复位屏幕 */
    esp_lcd_panel_init(panel_handle);                                   /* 初始化屏幕 */
    esp_lcd_panel_invert_color(panel_handle, true);                     /* 开启颜色反转 */
    esp_lcd_panel_set_gap(panel_handle, 0, 0);                          /* 设置XY偏移 */
    esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]) {0}, 1);
    esp_lcd_panel_io_tx_param(io_handle, 0x3A, (uint8_t[]) {0x65}, 1);
    lcd_display_dir(1);                                                 /* 设置屏幕方向 */
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));     /* 启动屏幕 */
    lcd_clear(WHITE);                                                   /* 默认填充白色 */
    LCD_BL(1);                                                          /* 打开背光 */
}
