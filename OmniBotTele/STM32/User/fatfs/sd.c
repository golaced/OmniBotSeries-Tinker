#include "head.h"

/******************************************************************************/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS fs;         /* Work area (file system object) for logical drive */
FIL fsrc, fdst;      /* file objects */
FRESULT res;
UINT br,bw;
char path0[512]="0:";
char buffer[4096];   /* file copy buffer */
uint8_t textFileBuffer[] = "中英文测试字符串 \r\nChinese and English test strings \r\n";
/*******************************************************************************
  * @函数名称	scan_files
  * @函数说明   搜索文件目录下所有文件 
  * @输入参数   path: 根目录 
  * @输出参数   无
  * @返回参数   FRESULT
  * @注意事项	无
  *****************************************************************************/
FRESULT scan_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
				printf("scan file - %s\n\r",path);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
                printf("scan file - %s/%s\n\r", path, fn);
            }
        }
    }else{
		printf("scan files error : %d\n\r",res);
	}

    return res;
}
/*******************************************************************************
  * @函数名称	SD_TotalSize
  * @函数说明   文件空间占用情况 
  * @输入参数   无 
  * @输出参数   无
  * @返回参数   1: 成功 
  				0: 失败
  * @注意事项	无
  *****************************************************************************/
int SD_TotalSize(char *path)
{
    FATFS *fs;
    DWORD fre_clust;        

    res = f_getfree(path, &fre_clust, &fs);  /* 必须是根目录，选择磁盘0 */
    if ( res==FR_OK ) 
    {
	  printf("\n\rget %s drive space.\n\r",path);
	  /* Print free space in unit of MB (assuming 512 bytes/sector) */
      printf("%d MB total drive space.\r\n"
           "%d MB available.\r\n",
           ( (fs->n_fatent - 2) * fs->csize ) / 2 /1024 , (fre_clust * fs->csize) / 2 /1024 );
		
	  return 1;
	}
	else
	{ 
	  printf("\n\rGet total drive space faild!\n\r");
	  return 0;   
	}
}





void SD_TEST(void)
{

	res = f_mount(0,&fs);
	if(res != FR_OK){
		printf("mount filesystem 0 failed : %d\n\r",res);
	}
	//写文件测试
	printf("write file test......\n\r");
    res = f_open(&fdst, "0:/longfilenametest.txt", FA_CREATE_ALWAYS | FA_WRITE);
	if(res != FR_OK){
		printf("open file error : %d\n\r",res);
	}else{
	    res = f_write(&fdst, textFileBuffer, sizeof(textFileBuffer), &bw);               /* Write it to the dst file */
		if(res == FR_OK){
			printf("write data ok! %d\n\r",bw);
		}else{
			printf("write data error : %d\n\r",res);
		}
		/*close file */
		f_close(&fdst);
	}

	//读文件测试
	printf("read file test......\n\r");
    res = f_open(&fsrc, "0:/test.txt", FA_OPEN_EXISTING | FA_READ);
    if(res != FR_OK){
		printf("open file error : %d\n\r",res);
	}else{
	    res = f_read(&fsrc, buffer, sizeof(textFileBuffer), &br);     /* Read a chunk of src file */
		if(res==FR_OK){
			printf("read data num : %d\n\r",br);
			printf("%s\n\r",buffer);
		}else{
			printf("read file error : %d\n\r",res);
		}
		/*close file */
		f_close(&fsrc);
	}
	//扫描已经存在的文件
	printf("\n\rbegin scan files path0......\n\r");
	scan_files(path0);

	SD_TotalSize(path0);//获取SD容量
}

void SD_INIT(void)
{
	res = f_mount(0,&fs);
	if(res != FR_OK){
		printf("mount filesystem 0 failed : %d\n\r",res);
	}
}

//BMP的图片结构
  typedef struct
{
        u8  pic_head[2];                                //1
        u16 pic_size_l;                            //2
        u16 pic_size_h;                            //3
        u16 pic_nc1;                                    //4
        u16 pic_nc2;                                    //5
        u16 pic_data_address_l;            //6
        u16 pic_data_address_h;                //7        
        u16 pic_message_head_len_l;        //8
        u16 pic_message_head_len_h;        //9
        u16 pic_w_l;                                        //10
        u16 pic_w_h;                                    //11
        u16 pic_h_l;                                    //12
        u16 pic_h_h;                                    //13        
        u16 pic_bit;                                    //14
        u16 pic_dip;                                    //15
        u16 pic_zip_l;                            //16
        u16 pic_zip_h;                            //17
        u16 pic_data_size_l;                    //18
        u16 pic_data_size_h;                    //19
        u16 pic_dipx_l;                            //20
        u16 pic_dipx_h;                            //21        
        u16 pic_dipy_l;                            //22
        u16 pic_dipy_h;                            //23
        u16 pic_color_index_l;            //24
        u16 pic_color_index_h;            //25
        u16 pic_other_l;                            //26
        u16 pic_other_h;                            //27
        u16 pic_color_p01;                    //28
        u16 pic_color_p02;                    //29
        u16 pic_color_p03;                    //30
        u16 pic_color_p04;                    //31
        u16 pic_color_p05;                    //32
        u16 pic_color_p06;                    //33
        u16 pic_color_p07;                    //34
        u16 pic_color_p08;                        //35                        
}BMP_HEAD;

BMP_HEAD bmp_data;

typedef struct
{
        u16 x;
        u16 y;
        u8  r;
        u8  g;
        u8  b;                
}BMP_POINT;

BMP_POINT point;

//24位。。变成16位图 
u32 RGB888ToRGB565(u8 r,u8 g,u8 b)
{return (u32) (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;}        //565

int get_bit(int num, int index) {
return (num>>(index-1)) & 0x00000001;
}
  //能显示320 240 。。240 320 的图片稍加修改就可以了
unsigned char buffer_bmp[1100];     // 这个数组和堆栈空间大小直接相关
void BMP_test(void)
{  
		unsigned long tx,ty,r_data,g_data,b_data;
  u16 hight=0,weight=0,i,bitcount;
  f_mount(0, &fs);
  res = f_open(&fsrc, "81.BMP", FA_OPEN_EXISTING | FA_READ);  //加上要显示的图片文件名
  res = f_read(&fsrc, &bmp_data, sizeof(bmp_data), &br);
  if((bmp_data.pic_head[0]=='B')&&(bmp_data.pic_head[1]=='M'))
   {
		 
		 weight =  bmp_data.pic_w_h<<8|(bmp_data.pic_w_l) ;//图像宽度128
     hight = bmp_data.pic_h_h<<8|(bmp_data.pic_h_l) ;//图像高度160
		 bitcount =bmp_data. pic_dip;///图像位数取值 24位
   //移动到有用 数据区。。
		 i=((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l);
    res = f_lseek(&fsrc, ((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l));
       f_read(&fsrc, buffer_bmp,8000, &br); //  f_read(&fsrc, buffer_bmp, (weight)*3, &br) ;     
     			   
		  for(tx =0; tx < 16; tx++) //128
                {
                    for(ty = 0; ty < bmp_data.pic_h_l; ty++ ) // 64
                    {
                  b_data= buffer_bmp[ty*16+tx]  ;
												for(i = 0; i < 8; i++ ) // 64
														{	
													

													if(get_bit(b_data,8-i)==1)
													{ OLED_DrawPoint( tx*8+i, ty,1);}
													else
													{	OLED_DrawPoint(  tx*8+i, ty,0);}
														}   
										}
                 }
		   OLED_Refresh_Gram();
               //}
                f_close(&fsrc);  
							 
   }
//	 for(i=0;i<9000;i++)UART_TX_CHAR(2 , buffer_bmp[i]);
//display_128x64_SD(buffer_bmp);
}

void GIF_test(u16 delay)//IMG00000
{   FILINFO fno;
	   DIR dir;
	char path1[21]="0:/gif1/IMG00000.bmp";
	u8 b,s,g=0;
		unsigned long tx,ty,r_data,g_data,b_data;
  u16 hight=0,weight=0,i,j,bitcount;
  f_mount(0, &fs);
	
	for(j=0;j<299;j++)
	{   b=j/100%100;
			s=j/10%10;
			g=j%10;

	path1[15]=g+48;
	path1[14]=s+48;
	path1[13]=b+48;
	  res = f_open(&fsrc, path1, FA_OPEN_EXISTING | FA_READ);  //加上要显示的图片文件名
  res = f_read(&fsrc, &bmp_data, sizeof(bmp_data), &br);
  if((bmp_data.pic_head[0]=='B')&&(bmp_data.pic_head[1]=='M'))
   {
		 
		 weight =  bmp_data.pic_w_h<<8|(bmp_data.pic_w_l) ;//图像宽度128
     hight = bmp_data.pic_h_h<<8|(bmp_data.pic_h_l) ;//图像高度160
		 bitcount =bmp_data. pic_dip;///图像位数取值 24位
   //移动到有用 数据区。。
		 i=((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l);
    res = f_lseek(&fsrc, ((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l));
       f_read(&fsrc, buffer_bmp,1100, &br); //  f_read(&fsrc, buffer_bmp, (weight)*3, &br) ;     
     			   
		  for(tx =0; tx < 16; tx++) //128
                {
                    for(ty = 0; ty < bmp_data.pic_h_l; ty++ ) // 64
                    {
                  b_data= buffer_bmp[ty*16+tx]  ;
												for(i = 0; i < 8; i++ ) // 64
														{	
													

													if(get_bit(b_data,8-i)==0)
													{ OLED_DrawPoint( tx*8+i, ty,1);}
													else
													{	OLED_DrawPoint(  tx*8+i, ty,0);}
														}   
										}
                 }
		   OLED_Refresh_Gram();
               //}
                f_close(&fsrc);  
							 
   }
	 delay_ms_api(delay);
 }
}


void GIF_SHOW(char * fold_m,char * fold_s,u16 num_pic,u16 delay)//IMG00000
{ FILINFO fno;
	DIR dir;
	u8 num_path=3,num_pathr=0;
	u8 b,s,g=0;
	unsigned long tx,ty,r_data,g_data,b_data;
  u16 hight=0,weight=0,i,j,bitcount;
	char path1[30]="0:/";
  f_mount(0, &fs);
	
	if(* fold_m!= '\0'){
	 while(* fold_m!= '\0')
    {
        path1[num_path++]=(* fold_m++);
    }
		path1[num_path++]='/';
	}
	while(* fold_s!= '\0')
    {
        path1[num_path++]=(* fold_s++);
    }
		path1[num_path++]='/';
		path1[num_path++]='I';
		path1[num_path++]='M';
		path1[num_path++]='G';
		path1[num_path++]='0';
		path1[num_path++]='0';
		num_pathr=num_path;
		path1[num_path++]='0';
		path1[num_path++]='0';
		path1[num_path++]='0';
		path1[num_path++]='.';
		path1[num_path++]='b';
		path1[num_path++]='m';
		path1[num_path++]='p';
		
		
	for(j=0;j<num_pic;j++)
	{   b=j/100%100;
			s=j/10%10;
			g=j%10;

	path1[num_pathr+2]=g+48;
	path1[num_pathr+1]=s+48;
	path1[num_pathr]=b+48;
	  res = f_open(&fsrc, path1, FA_OPEN_EXISTING | FA_READ);  //加上要显示的图片文件名
  res = f_read(&fsrc, &bmp_data, sizeof(bmp_data), &br);
  if((bmp_data.pic_head[0]=='B')&&(bmp_data.pic_head[1]=='M'))
   {
		 
		 weight =  bmp_data.pic_w_h<<8|(bmp_data.pic_w_l) ;//图像宽度128
     hight = bmp_data.pic_h_h<<8|(bmp_data.pic_h_l) ;//图像高度160
		 bitcount =bmp_data. pic_dip;///图像位数取值 24位
   //移动到有用 数据区。。
		 i=((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l);
    res = f_lseek(&fsrc, ((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l));
       f_read(&fsrc, buffer_bmp,1100, &br); //  f_read(&fsrc, buffer_bmp, (weight)*3, &br) ;     
     			   
		  for(tx =0; tx < 16; tx++) //128
                {
                    for(ty = 0; ty < bmp_data.pic_h_l; ty++ ) // 64
                    {
                  b_data= buffer_bmp[ty*16+tx]  ;
												for(i = 0; i < 8; i++ ) // 64
														{	
													

													if(get_bit(b_data,8-i)==0)
													{ OLED_DrawPoint( tx*8+i, ty,1);}
													else
													{	OLED_DrawPoint(  tx*8+i, ty,0);}
														}   
										}
                 }
		   OLED_Refresh_Gram();
               //}
                f_close(&fsrc);  
							 
   }
	 delay_ms_api(delay);
 }
}



u8 GIF_SHOW_IRQ(char * fold_m,char * fold_s,u16 num_pic_now)//IMG00000
{ FILINFO fno;
	DIR dir;
	u8 num_path=3,num_pathr=0;
	u8 q,b,s,g=0;
	unsigned long tx,ty,r_data,g_data,b_data;
  u16 hight=0,weight=0,i,j,bitcount;
	char path1[30]="0:/";
  f_mount(0, &fs);
	
	if(* fold_m!= '\0'){
	 while(* fold_m!= '\0')
    {
        path1[num_path++]=(* fold_m++);
    }
		path1[num_path++]='/';
	}
	while(* fold_s!= '\0')
    {
        path1[num_path++]=(* fold_s++);
    }
		path1[num_path++]='/';
		path1[num_path++]='I';
		path1[num_path++]='M';
		path1[num_path++]='G';
		path1[num_path++]='0';
		path1[num_path++]='0';
		num_pathr=num_path;
		path1[num_path++]='0';
		path1[num_path++]='0';
		path1[num_path++]='0';
		path1[num_path++]='.';
		path1[num_path++]='b';
		path1[num_path++]='m';
		path1[num_path++]='p';
		
		
	   j=num_pic_now;
		  q=j/1000;
	    b=j/100%100;
			s=j/10%10;
			g=j%10;

	path1[num_pathr+2]=g+48;
	path1[num_pathr+1]=s+48;
	path1[num_pathr]=b+48;
	path1[num_pathr-1]=q+48;
	  res = f_open(&fsrc, path1, FA_OPEN_EXISTING | FA_READ);  //加上要显示的图片文件名
  res = f_read(&fsrc, &bmp_data, sizeof(bmp_data), &br);
  if((bmp_data.pic_head[0]=='B')&&(bmp_data.pic_head[1]=='M'))
   {
		 
		 weight =  bmp_data.pic_w_h<<8|(bmp_data.pic_w_l) ;//图像宽度128
     hight = bmp_data.pic_h_h<<8|(bmp_data.pic_h_l) ;//图像高度160
		 bitcount =bmp_data. pic_dip;///图像位数取值 24位
   //移动到有用 数据区。。
		 i=((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l);
    res = f_lseek(&fsrc, ((bmp_data.pic_data_address_h<<16)|bmp_data.pic_data_address_l));
       f_read(&fsrc, buffer_bmp,1100, &br); //  f_read(&fsrc, buffer_bmp, (weight)*3, &br) ;     
     			   
		  for(tx =0; tx < 16; tx++) //128
                {
                    for(ty = 0; ty < bmp_data.pic_h_l; ty++ ) // 64
                    {
                  b_data= buffer_bmp[ty*16+tx]  ;
												for(i = 0; i < 8; i++ ) // 64
														{	
													

													if(get_bit(b_data,8-i)==0)
													{ OLED_DrawPoint( tx*8+i, ty,1);}
													else
													{	OLED_DrawPoint(  tx*8+i, ty,0);}
														}   
										}
                 }
		   OLED_Refresh_Gram();
               //}
                f_close(&fsrc);  
							 
   }
 
}