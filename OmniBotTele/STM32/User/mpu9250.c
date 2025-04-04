#include "spi.h"
#include "mpu9250.h"
#include "delay.h"
#include "head.h"
#include "data_transfer.h"
#define MPU9250_CS PBout(10)			//====MPU9250的片选位
/**
  ******************************************************************************
  * File Name          : Mpu9250.c
  * Description        : Initialize and update MPU data
  ******************************************************************************
  *	Author						 : AGKODY
  * Date							 : 20/11/2016
  ******************************************************************************
  */
	
#define DELAY 300		//初始化必须放慢速度
#define HOLDON 1		//读取时可以加快速度

int16andUint8_t rawAccel[3];
int16andUint8_t rawGyro[3];
int16andUint8_t rawMag[3];

int16andUint8_t rawMPU6050Temperature;
int16_t orientationMatrix[9];

uint8_t Mpu9250_Init(void)
{
		MPU9250_Write_Reg(USER_CTRL,0X01); 										//====复位所有寄存器，必须执行这一步，否则复位电源后MPU内部的I2C仍在执行
		delay_us(DELAY);																			//====有些地方需要长时间的延时，干脆初始化慢一些
		MPU9250_Write_Reg(PWR_MGMT_1,0X80);  									//====电源管理,复位MPU9250
		delay_us(DELAY);
		//MPU9250_Write_Reg(CONFIG,0x04);												//====低通滤波器 0x06 5hz
		MPU9250_Write_Reg(CONFIG,0x06);												//====低通滤波器 0x06 5hz
/**********************Init SLV0 i2c**********************************/	
		MPU9250_Write_Reg(INT_PIN_CFG,0X30);  								//====INT Pin / Bypass Enable Configuration  
		delay_us(DELAY);
		MPU9250_Write_Reg(INT_ENABLE,0X01);
		delay_us(DELAY);
		MPU9250_Write_Reg(SMPLRT_DIV,0x00);										//====采样率1000/(1+0)=1000HZ
		delay_us(DELAY);
		MPU9250_Write_Reg(I2C_MST_CTRL,0X5D); 								//====I2C MAster mode and Speed 400 kHz
		delay_us(DELAY);
		MPU9250_Write_Reg(USER_CTRL,0X30); 										//====使能MPU9250SPI
		delay_us(DELAY);
/*******************Init GYRO and ACCEL******************************/		
		MPU9250_Write_Reg(SMPLRT_DIV, 0x00);  								//====陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
		delay_us(DELAY);
		MPU9250_Write_Reg(GYRO_CONFIG,0X18);  								//====陀螺仪测量范围 0X18 正负2000度
		delay_us(DELAY);
		MPU9250_Write_Reg(ACCEL_CONFIG,0x10); 								//====加速度计测量范围 0X18 正负8g//(0x00 +-2g;)  ( 0x08 +-4g;)  (0x10 +-8g;)  (0x18 +-16g)
		delay_us(DELAY);
		MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x04);//0x08);							//====加速计高通滤波频率 典型值 ：0x08  （1.13kHz）	
		delay_ms(100);
/**********************Init MAG *************************************/	
//	
		MPU9250_Write_Reg(I2C_MST_CTRL, 0x5D);							
		delay_ms(DELAY);
		MPU9250_Write_Reg(I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
		delay_ms(DELAY);
		MPU9250_Write_Reg(I2C_SLV0_REG, AK8963_ST1);
		delay_ms(DELAY);
		MPU9250_Write_Reg(I2C_SLV0_CTRL,I2C_SLVx_EN | 8);			//====enable IIC	and EXT_SENS_DATA==8 Bytes
		delay_ms(DELAY);
		MPU9250_Write_Reg(I2C_SLV4_CTRL, 0x09);
		delay_ms(DELAY);
		MPU9250_Write_Reg(I2C_MST_DELAY_CTRL, 0x81);					//====开从设备0数据采集延时
		delay_ms(DELAY);

//-----------------此printf为测试MPU92_Mag_WriteReg()在上电时出错在哪所用
//		printf("1st 1st 1st \n");					
//		printf("I2C_SLV4_ADDR: %d \n",MPU9250_Read_Reg(I2C_SLV4_ADDR));
//		printf("I2C_SLV4_REG: %d \n",MPU9250_Read_Reg(I2C_SLV4_REG));
//		printf("I2C_SLV4_DO: %d \n",MPU9250_Read_Reg(I2C_SLV4_DO));
//		printf("I2C_SLV4_CTRL: %d \n",MPU9250_Read_Reg(I2C_SLV4_CTRL));
//		printf("I2C_MST_STATUS: %d \n",MPU9250_Read_Reg(I2C_MST_STATUS));
//		printf("I2C_MST_STATUS: %d \n",MPU9250_Read_Reg(I2C_MST_STATUS));
//		printf("AK8963_ASAX: %d \n",MPU92_Mag_ReadReg(AK8963_ASAX));
		
		
		
		
		MPU92_Mag_WriteReg(AK8963_CNTL2,0x01); // Reset AK8963
		delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x00); //Power-down mode
		delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x0F); //Fuse ROM access mode  	
		delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x00); //Power-down mode
		delay_us(DELAY);
		MPU92_Mag_WriteReg(AK8963_CNTL1,0x06);//0x06 0x16); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	
		delay_us(DELAY);
		
		//printf("AK8963_WIA: %d \n",MPU92_Mag_ReadReg(AK8963_WIA));
		  MPU9250_ReadValue();
		module.acc=1;	
		return 0;
}	

//====SPI写寄存器
//====reg:指定的寄存器地址
//====value:写入的值
uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	SPI_CS(CS_MPU9250,0);
	status=Spi_RW(reg); //发送写命令+寄存器号
	Spi_RW(value);//写入寄存器值
	SPI_CS(CS_MPU9250,1);
	return(status);//返回状态值
}

//====SPI读取寄存器
//====reg:指定的寄存器地址
uint8_t MPU9250_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI_CS(CS_MPU9250,0);
	Spi_RW(reg|0x80); //====发送读命令+寄存器号
	reg_val=Spi_RW(0xff);//====读取寄存器值
	SPI_CS(CS_MPU9250,1);
	return(reg_val);
}
//====SPI写磁感计寄存器
//====writeAddr:指定的寄存器地址
//====writeData:写入的值
void MPU92_Mag_WriteReg( uint8_t writeAddr, uint8_t writeData )
{
	uint8_t  status = 0;
	MPU9250_Write_Reg(I2C_SLV4_ADDR ,AK8963_ADDR);//设置磁力计地址,mode: writ
	delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_REG ,writeAddr);//set reg addr
	delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_DO ,writeData);//send value	
	delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_CTRL ,0x81);
	do {
    status = MPU9250_Read_Reg(I2C_MST_STATUS);
    delay_us(DELAY);
  } while (((status & I2C_SLV4_DONE) == 0));
}
//====SPI读磁感计寄存器
//====readAddr:指定的寄存器地址
uint8_t MPU92_Mag_ReadReg( uint8_t readAddr )
{
	u16 j=500;
	MPU9250_Write_Reg(I2C_SLV4_ADDR ,AK8963_ADDR|0x80); //设置磁力计地址，mode：read
	delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_REG ,readAddr);// set reg addr
	delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_CTRL ,0x80);
	delay_us(HOLDON);
	MPU9250_Write_Reg(I2C_SLV4_DO ,0xff);//read
	delay_us(1000);
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	return MPU9250_Read_Reg(I2C_SLV4_DI);
}
//====读取MPU9250数据
void MPU9250_ReadValue(void)
{
	uint8_t number;
	uint8_t MPU9250_buf[22],i;
	int16_t MatrixAccelData[3];
	int16_t	translateAccelData[3];
	
	int16_t MatrixGyroData[3];
	int16_t translateGyroData[3];
	
	int16_t MatrixMagData[3];
	int16_t translateMagData[3];
  SPI_CS(CS_MPU9250,0);
	delay_us(3);
	Spi_RW(ACCEL_XOUT_H|0x80); //====发送读命令+寄存器号
	for(i=0;i<22;i++)//====一共读取14字节的数据
	{
		MPU9250_buf[i]=Spi_RW(0xff); //====循环读取
	}
  SPI_CS(CS_MPU9250,1);
	delay_us(1);
	
	
    rawAccel[YORIENT].bytes[1]       = MPU9250_buf[ 0];//====传输原始数据
    rawAccel[YORIENT].bytes[0]       = MPU9250_buf[ 1];
    rawAccel[XORIENT].bytes[1]       = MPU9250_buf[ 2];
    rawAccel[XORIENT].bytes[0]       = MPU9250_buf[ 3];
    rawAccel[ZORIENT].bytes[1]       = MPU9250_buf[ 4];
    rawAccel[ZORIENT].bytes[0]       = MPU9250_buf[ 5];

    rawMPU6050Temperature.bytes[1] = MPU9250_buf[ 6];
    rawMPU6050Temperature.bytes[0] = MPU9250_buf[ 7];

    rawGyro[PITCH].bytes[1]        = MPU9250_buf[ 8];
    rawGyro[PITCH].bytes[0]        = MPU9250_buf[ 9];
    rawGyro[ROLL ].bytes[1]        = MPU9250_buf[10];
    rawGyro[ROLL ].bytes[0]        = MPU9250_buf[11];
    rawGyro[YAW  ].bytes[1]        = MPU9250_buf[12];
    rawGyro[YAW  ].bytes[0]        = MPU9250_buf[13];
	
		rawMag[YORIENT].bytes[1]       = MPU9250_buf[15];
		rawMag[YORIENT].bytes[0]       = MPU9250_buf[16];
		rawMag[XORIENT].bytes[1]       = MPU9250_buf[17];
		rawMag[XORIENT].bytes[0]       = MPU9250_buf[18];
		rawMag[ZORIENT].bytes[1]       = MPU9250_buf[19];
		rawMag[ZORIENT].bytes[0]       = MPU9250_buf[20];
	

}


MPU6050_STRUCT mpu6050_fc;

#include "cycle_cal_oldx.h"
s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
#define OFFSET_AV_NUM_ACC 50
void MPU6050_Data_Offset()
{
  xyz_f_t data;	
		static xyz_f_t ACC_Reg;
		static u8 acc_3d_step_reg;
		float sphere_x,sphere_y,sphere_z,sphere_r;
    if(mpu6050_fc.Acc_CALIBRATE == 1)
    {
       

        acc_sum_cnt++;
				if(mpu6050_fc.Cali_3d){
//			  sum_temp[A_X] += (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x ;
//        sum_temp[A_Y] += (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y ;
//        sum_temp[A_Z] += (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z - 65536/16;
				  //sum_temp_att[0]+=Pit_fc1;
					//sum_temp_att[1]+=Rol_fc1;
				}
				
				{
        sum_temp[A_X] += mpu6050_fc.Acc_I16.x;
        sum_temp[A_Y] += mpu6050_fc.Acc_I16.y;
        sum_temp[A_Z] += mpu6050_fc.Acc_I16.z - 65536/16;   // +-8G
				}
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {   
            mpu6050_fc.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[0]=(float)sum_temp_att[0]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[1]=(float)sum_temp_att[1]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            acc_sum_cnt =0;
            mpu6050_fc.Acc_CALIBRATE = 0;
            WRITE_PARM();
					
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
				  	sum_temp_att[1]=sum_temp_att[0]=0;
        }
    }
// 3d cal

		
		ACC_Reg.x=mpu6050_fc.Acc_I16.x;
	  ACC_Reg.y=mpu6050_fc.Acc_I16.y;
		ACC_Reg.z=mpu6050_fc.Acc_I16.z;


    if(mpu6050_fc.Gyro_CALIBRATE)
    {
        gyro_sum_cnt++;
        sum_temp[G_X] += mpu6050_fc.Gyro_I16.x;
        sum_temp[G_Y] += mpu6050_fc.Gyro_I16.y;
        sum_temp[G_Z] += mpu6050_fc.Gyro_I16.z;
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
            mpu6050_fc.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            gyro_sum_cnt =0;
            if(mpu6050_fc.Gyro_CALIBRATE == 1)
			{
               WRITE_PARM();
			}  
            mpu6050_fc.Gyro_CALIBRATE = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
        }
    }
}

void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
    *it_x = itx;
    *it_y = ity;
    *it_z = itz;

}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float mpu6050_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;

void MPU6050_Data_Prepare(float T)
{   	int en_off_3d_off=0;
    u8 i;
    s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
//	float auto_offset_temp[3];
    float Gyro_tmp[3];


    MPU6050_Data_Offset(); //????


		mpu6050_fc.Acc_I16.x=rawAccel[1].value;
		mpu6050_fc.Acc_I16.y=rawAccel[0].value;
		mpu6050_fc.Acc_I16.z=rawAccel[2].value;
		mpu6050_fc.Gyro_I16.x=rawGyro[1].value;
		mpu6050_fc.Gyro_I16.y=rawGyro[0].value;
		mpu6050_fc.Gyro_I16.z=rawGyro[2].value;
		mpu6050_fc.Mag_adc.x=rawMag[1].value/100.;
		mpu6050_fc.Mag_adc.y=rawMag[0].value/100.;
		mpu6050_fc.Mag_adc.z=rawMag[2].value/100.;	
	
	
    Gyro_tmp[0] = mpu6050_fc.Gyro_I16.x ;//
    Gyro_tmp[1] = mpu6050_fc.Gyro_I16.y ;//
    Gyro_tmp[2] = mpu6050_fc.Gyro_I16.z ;//
    mpu6050_fc.Tempreature=rawMPU6050Temperature.value;
    mpu6050_fc.TEM_LPF += 2 *3.14f *T *(mpu6050_fc.Tempreature - mpu6050_fc.TEM_LPF);
    mpu6050_fc.Ftempreature = mpu6050_fc.TEM_LPF/340.0f + 36.5f;

//======================================================================
    if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
    {
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
    }
//10 170 4056
		if(fabs(mpu6050_fc.Off_3d.x)>10||fabs(mpu6050_fc.Off_3d.y)>10||fabs(mpu6050_fc.Off_3d.z)>10)
			mpu6050_fc.Cali_3d=1;
	
    /* ???????? */
	 if(mpu6050_fc.Cali_3d){
			  mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x - mpu6050_fc.Acc_Offset.x*en_off_3d_off;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y - mpu6050_fc.Acc_Offset.y*en_off_3d_off;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z - mpu6050_fc.Acc_Offset.z*en_off_3d_off;
	 }
   else{	 

        mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z) ;

  }
    mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050_fc.Gyro_Offset.x ;//
    mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050_fc.Gyro_Offset.y ;//
    mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050_fc.Gyro_Offset.z ;//


    /* ?????????? */
    FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];

    for(i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }


    mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


    mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;


    /*????*/
    Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&mpu6050_fc.Acc.x,&mpu6050_fc.Acc.y,&mpu6050_fc.Acc.z);
    Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&mpu6050_fc.Gyro.x,&mpu6050_fc.Gyro.y,&mpu6050_fc.Gyro.z);

    mpu6050_fc.Gyro_deg.x = mpu6050_fc.Gyro.x *0.06103f ;
    mpu6050_fc.Gyro_deg.y = mpu6050_fc.Gyro.y *0.06103f ;
    mpu6050_fc.Gyro_deg.z = mpu6050_fc.Gyro.z *0.06103f ;

}
