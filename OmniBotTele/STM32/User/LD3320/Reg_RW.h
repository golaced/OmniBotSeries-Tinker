/************************************************************************************
**	版权所有：Copyright (c) 2005 - 2010 ICRoute INC.
** 硬件说明：STM32f103vet6  8MHZ  内部时钟为72MHZ 
** 					 LD3320 有源晶振 12MHZ  不同的晶振请修改LDChip.h 中的定义值（＃define CLK_IN  12）
** 软件说明：并行读写寄存器方式，如采用其他方式即修改Reg_RW.c 文件即可！
** 修改者:  http://shop64790118.taobao.com/
** 时间：2011.10.15
** 说明：本程序修改于 ICRoute公司 配套的源程序，版权归其所有。
************************************************************************************/

// Reg_RW.h  读写LD3320芯片的寄存器，共有四种方法。
#ifndef REG_RW_H
#define REG_RW_H

#define uint8 unsigned char
#define uint16 unsigned int
#define uint32 unsigned long


/****************************************************************
函数名： LD_WriteReg
功能：写LD3320芯片的寄存器
参数：  address, 8位无符号整数，地址
		dataout，8位无符号整数，要写入的数据
返回值：无
****************************************************************/ 
void LD_WriteReg( unsigned char address, unsigned char dataout );

/****************************************************************
函数名： LD_ReadReg
功能：读LD3320芯片的寄存器
参数：  address, 8位无符号整数，地址
返回值：8位无符号整数，读取的结果
****************************************************************/ 
unsigned char LD_ReadReg( unsigned char address );



#endif
