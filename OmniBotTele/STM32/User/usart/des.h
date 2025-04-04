/*-------------------------------------------------------------
		置换表 
-------------------------------------------------------------*/

#ifndef _DES_H_    // 防重复编译 
#define _DES_H_  
#include "stm32f10x.h"
extern void  des_test(char *data,char *key_lock,char * key_unlock,unsigned int num);
extern void  des_unlock(char *data,char *key_lock,char * key_unlock,unsigned int num,u8 en);
extern char MesHex_OUT[128];         // 16个字符数组用于存放 64位16进制的密文
extern char MyMessage_Kick[125];
extern char KEY_LOCK[9]; 
extern char KEY_UNLOCK[9]; 
#endif



	



