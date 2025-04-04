#include "imu_cmd.h"
#include <serial/serial.h> 
#include <sstream>
#include "unistd.h"
#include "spi_node.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "spi.h"
#include "sys_time.h"
#include <pthread.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include "imu_cmd.h"
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <iostream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
serial::Serial com; //Declare serial port object
IMUData_Out imu_o;

YAML::Node config_hardware1=YAML::LoadFile("/home/odroid/Tinker/Param/param_hardware.yaml");

//------------------------------------------------------------------------------
// Description: Serial port sends data to the device
// Input: buf[Len]=content to be sent
// Return: Returns the number of bytes sent
//------------------------------------------------------------------------------
int UART_Write(const U8 *buf, int Len)
{
    return com.write(buf, Len); //Send serial port data
}

void test_uart(void)
{
    try 
    {//Set the serial port properties and open the serial port
        com.setPort("/dev/ttyUSB0");
        com.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        com.setTimeout(to); 
        com.open(); 
        printf("Open serial good\n");
    }
    catch (serial::IOException& e) 
    {
        printf("Open serial fail\n");
    }
    if(!com.isOpen()) 
    {// Serial port opening failed
        printf("Open serial fail\n");
    } 

    Cmd_12(5, 255, 0,  0, 2, 60, 1, 3, 5, 0x007f); // 1.Set parameters
    Cmd_03();//2.Wake up sensor
    Cmd_19();//3.Enable active data reporting
    Cmd_05();//4.Z axis angle reset to zero

    unsigned short  data_size;
    unsigned char   tmpdata[4096] ;
    static int cntp=0;
    static char Count=0;
    static char rs_count=0;
    static char last_rsnum=0;
    static char rsimu_flag=0;
    static char rsacc_flag=0;
    int att_cal=0,gyro_cal=0;
    //read bias
    spi_rx.att_usb_bias[0]=config_hardware1["mems_param"]["att_bias"][0].as<float>();//
    spi_rx.att_usb_bias[1]=config_hardware1["mems_param"]["att_bias"][1].as<float>();//
    spi_rx.att_usb_bias[2]=config_hardware1["mems_param"]["att_bias"][2].as<float>();//
    spi_rx.att_rate_usb_bias[0]=config_hardware1["mems_param"]["gyro_bias"][0].as<float>();//
    spi_rx.att_rate_usb_bias[1]=config_hardware1["mems_param"]["gyro_bias"][1].as<float>();//
    spi_rx.att_rate_usb_bias[2]=config_hardware1["mems_param"]["gyro_bias"][2].as<float>();//

    while(1){
        if(data_size = com.available())
        {//com.available(When the serial port does not have a cache, this function will wait until there is a cache before returning the number of characters.
            com.read(tmpdata, data_size);
            for(int i=0; i < data_size; i++)
            {
                Cmd_GetPkt(tmpdata[i]); // Transplantation: Fill in this function every time 1 byte of data is received. When a valid data packet is captured, it will call back and enter the Cmd_RxUnpack(U8 *buf, U8 DLen) function processing.
            }
 #if 1
            //cal imu
            if(att_cal==0&&(mems.Acc_CALIBRATE))
                att_cal=1;
           // printf("%d %d\n",att_cal,mems.Acc_CALIBRATE);
            if(att_cal==1){
              att_cal=2;
              spi_rx.att_usb_bias[0]=spi_rx.att_usb[0];
              spi_rx.att_usb_bias[1]=spi_rx.att_usb[1];
              config_hardware1["mems_param"]["att_bias"][0]=spi_rx.att_usb_bias[0];//
              config_hardware1["mems_param"]["att_bias"][1]=spi_rx.att_usb_bias[1];//
              config_hardware1["mems_param"]["att_bias"][2]=spi_rx.att_usb_bias[2];//
              std::ofstream fout_hard("/home/odroid/Tinker/Param/param_hardware.yaml");
              fout_hard<<config_hardware1;
              fout_hard.close();
            }
            if(att_cal==2&&mems.Acc_CALIBRATE==0&&mems.Gyro_CALIBRATE==0){
                printf("Hardware::IMU Mems Att bias is [%f] [%f] [%f]\n",spi_rx.att_usb_bias[0],spi_rx.att_usb_bias[1],spi_rx.att_usb_bias[2]);
                att_cal=0;
            }

            if(gyro_cal==0&&(mems.Gyro_CALIBRATE))
                gyro_cal=1;

            if(gyro_cal==1){
              gyro_cal=2;
              spi_rx.att_rate_usb_bias[0]=spi_rx.att_rate_usb[0];
              spi_rx.att_rate_usb_bias[1]=spi_rx.att_rate_usb[1];
              spi_rx.att_rate_usb_bias[2]=spi_rx.att_rate_usb[2];
              config_hardware1["mems_param"]["gyro_bias"][0]=spi_rx.att_rate_usb_bias[0];//
              config_hardware1["mems_param"]["gyro_bias"][1]=spi_rx.att_rate_usb_bias[1];//
              config_hardware1["mems_param"]["gyro_bias"][2]=spi_rx.att_rate_usb_bias[2];//
              std::ofstream fout_hard("/home/odroid/Tinker/Param/param_hardware.yaml");
              fout_hard<<config_hardware1;
              fout_hard.close();
            }
            if(gyro_cal==2&&mems.Acc_CALIBRATE==0&&mems.Gyro_CALIBRATE==0){
                printf("Hardware::IMU Mems Gyro bias is [%f] [%f] [%f]\n",spi_rx.att_rate_usb_bias[0],spi_rx.att_rate_usb_bias[1],spi_rx.att_rate_usb_bias[2]);
                gyro_cal=0;
            }
#endif
#if 1
            if(cntp++>50){
                cntp=0;
                printf("data_size=%d\n",data_size);
                printf("att=%.1f %.1f %.1f|gyro=%.1f %.1f %.1f|acc=%.1f %.1f %.1f\n",
                        imu_o.att[0],imu_o.att[1],imu_o.att[2],
                        imu_o.gyro[0],imu_o.gyro[1],imu_o.gyro[2],
                        imu_o.acc[0],imu_o.acc[1],imu_o.acc[2]);
            }
#endif
        }
        usleep(10*1000);
    }
}
