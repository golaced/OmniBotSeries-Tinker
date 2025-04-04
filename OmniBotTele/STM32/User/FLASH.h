#ifndef __FLASH_H
#define __FLASH_H

#define Accel_Offset_Address  0
#define Gyro_Offset_Address   12
#define Mag_Offset_Address  24//磁力计矫正数据地址
extern u8 CHANNAL;
u8 Parameter_Init(void);
void WRITE_PARM(void);
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum);
void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData);
void WriteFlashHarfWord(uint32_t WriteAddress,uint16_t WriteData);

void WriteFlashNineFloat(uint32_t WriteAddress,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3,
                         float WriteData4,
                         float WriteData5,
                         float WriteData6,
                         float WriteData7,
                         float WriteData8,
                         float WriteData9);

uint8_t ReadFlashThreeFloat(uint32_t WriteAddress,
                         float *WriteData1,
                         float *WriteData2,
                         float *WriteData3);
#endif
