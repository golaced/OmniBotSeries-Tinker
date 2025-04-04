#ifndef __UDP_OCU_H__
#define __UDP_OCU_H__
#include "base_struct.h"
void UDP_OCU_TX(float dt);//发送
void UDP_RX_PROCESS(char* Buf, int Len);
extern int usb_send_cnt;
extern char SendBuff_USB[250];

void UDP_OCU_TX_SDK(float dt);//发送
void UDP_RX_PROCESS_SDK(char* Buf, int Len);
void UDP_RX_PROCESS_ARM(char* Buf, int Len);
extern int usb_send_cnt_SDK;
extern char SendBuff_USB_SDK[250];
#endif
