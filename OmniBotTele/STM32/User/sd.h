#ifndef _SD_H__
#define _SD_H__
#include "head.h"
extern void SD_INIT(void);
extern void SD_TEST(void);
extern void BMP_test(void);
extern void GIF_test(u16 delay);
extern void GIF_SHOW(char * fold_m,char * fold_s,u16 num_pic,u16 delay);
extern u8 GIF_SHOW_IRQ(char * fold_m,char * fold_s,u16 num_pic_now);
#endif /* _FATFS */
