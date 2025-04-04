/*
*---------------------------------------------------------------
*                        Lvgl Font Tool                         
*                                                               
* 注:使用unicode编码                                              
* 注:本字体文件由Lvgl Font Tool V0.3 生成                          
* 作者:阿里(qq:617622104)                                         
*---------------------------------------------------------------
*/


#include "lvgl.h"
#include "text.h"
#include "fonts.h"

typedef struct{
    uint16_t min;
    uint16_t max;
    uint8_t  bpp;
    uint8_t  reserved[3];
}x_header_t;
typedef struct{
    uint32_t pos;
}x_table_t;
typedef struct{
    uint8_t adv_w;
    uint8_t box_w;
    uint8_t box_h;
    int8_t  ofs_y;
}glyph_dsc_t;


static x_header_t __g_xbf_hd = {
    .min = 0,
    .max = 0,
    .bpp = 0,
};
static uint8_t __g_font_buf[150];   //如bin文件存在SPI FLASH可使用此buff

static uint8_t *__user_font_getdata(int offset, int size)
{
    //如字模保存在SPI FLASH, SPIFLASH_Read(__g_font_buf,offset,size);
    fonts_partition_read(__g_font_buf,ftinfo.lvgl_24addr +offset,size);
    return __g_font_buf;
}


static const uint8_t * __user_font_get_bitmap(const lv_font_t * font, uint32_t unicode_letter) 
{
    if( __g_xbf_hd.max==0 ) 
    {
        uint8_t *p = __user_font_getdata(0, sizeof(x_header_t));
        memcpy(&__g_xbf_hd, p, sizeof(x_header_t));
    }
    if( unicode_letter>__g_xbf_hd.max || unicode_letter<__g_xbf_hd.min ) {
        return NULL;
    }
    uint32_t unicode_offset = sizeof(x_header_t)+(unicode_letter-__g_xbf_hd.min)*4;
    uint32_t *p_pos = (uint32_t *)__user_font_getdata(unicode_offset, 4);
    if( p_pos[0] != 0 ) {
        uint32_t pos = p_pos[0];
        glyph_dsc_t * gdsc = (glyph_dsc_t*)__user_font_getdata(pos, sizeof(glyph_dsc_t));
        return __user_font_getdata(pos+sizeof(glyph_dsc_t), gdsc->box_w*gdsc->box_h*__g_xbf_hd.bpp/8);
    }
    return NULL;
}


static bool __user_font_get_glyph_dsc(const lv_font_t * font, lv_font_glyph_dsc_t * dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next) {
    if( __g_xbf_hd.max==0 ) {
        uint8_t *p = __user_font_getdata(0, sizeof(x_header_t));
        memcpy(&__g_xbf_hd, p, sizeof(x_header_t));
    }
    if( unicode_letter>__g_xbf_hd.max || unicode_letter<__g_xbf_hd.min ) {
        return NULL;
    }
    uint32_t unicode_offset = sizeof(x_header_t)+(unicode_letter-__g_xbf_hd.min)*4;
    uint32_t *p_pos = (uint32_t *)__user_font_getdata(unicode_offset, 4);
    if( p_pos[0] != 0 ) {
        glyph_dsc_t * gdsc = (glyph_dsc_t*)__user_font_getdata(p_pos[0], sizeof(glyph_dsc_t));
        dsc_out->adv_w = gdsc->adv_w;
        dsc_out->box_h = gdsc->box_h;
        dsc_out->box_w = gdsc->box_w;
        dsc_out->ofs_x = 0;
        dsc_out->ofs_y = gdsc->ofs_y;
        dsc_out->bpp   = __g_xbf_hd.bpp;
        return true;
    }
    return false;
}


//微软雅黑,Regular,24
//字模高度：41
//XBF字体,外部bin文件
const lv_font_t Font24 = {
    .get_glyph_bitmap = __user_font_get_bitmap,
    .get_glyph_dsc = __user_font_get_glyph_dsc,
    .line_height = 41,
    .base_line = 0,
};

//end of file
