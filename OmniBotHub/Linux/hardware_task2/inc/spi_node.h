#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_

#define SPI_BUF_SIZE 255
#define CAN_T_DIV 500.0
#define CAN_I_DIV 100.0
#define CAN_F_DIV 100.0
#define CAN_POS_DIV 30.0
#define CAN_DPOS_DIV 20.0
#define CAN_GAIN_DIV_P 500.0
#define CAN_GAIN_DIV_I 10000.0
#define CAN_GAIN_DIV_D 1000.0
#define CAN_GAIN_DIV_P_M 10.0
#define CAN_GAIN_DIV_I_M 1000.0
#define CAN_GAIN_DIV_D_M 100.0

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


typedef struct
{
    float x;
    float y;
    float z;
    float zz;
}END_POS;

typedef struct{
    char connect;
    int loss_cnt;
    char mode;
    char up_mode;
    float rc_spd_b[3],rc_rate_b[3];

    char cmd_robot_state;
    float rc_spd_w[2],rc_att_w[2],rate_yaw_w;
    int key_ud,key_lr,key_x,key_y,key_a,key_b,key_ll,key_rr,key_st,key_back,key_ud_reg,key_lr_reg,key_x_reg,key_y_reg,key_a_reg,key_b_reg,key_ll_reg,key_rr_reg,key_st_reg,key_back_reg;
    float curve[20];

    float sbus_ch[6];
    int sbus_aux[6];
    char sbus_conncect;
    char sbus_power_sw,sbus_power_sw_reg;
    int sbus_rc_main[6];
    int sbus_rc_main_reg[6];
    int sbus_mode,sbus_mode_reg;
    int sbus_mode_e,sbus_mode_e_reg;
    float sbus_height,sbus_height_reg;
    int sbus_aux_reg[6];
}_OCU;


typedef struct{
 int rssi;
 float dis,angle;
}_AOA;

typedef struct
{
  int connect;
  char power;
  int cap;
  int mode;
  int loss_connect;
  float att_set[3];
  END_POS pos_set;
  float att_now[3];
  END_POS pos_now;
  float move_spd;
  float torque;
} _ARMSS;


typedef struct
{
float att[3];
float att_rate[3];
float acc_b[3];
float acc_n[3];
float att_usb[3];
float att_rate_usb[3];
float acc_b_usb[3];
float att_usb_bias[3];
float att_rate_usb_bias[3];
float acc_b_usb_bias[3];
float q[14],q_servo[14];
float dq[14],dq_servo[14];
float tau[14],tau_servo[14];
float bat_v;
char connect[14];
char connect_motor[14];
char connect_servo[14];
char ready[14];
_OCU ocu;
_AOA aoa;
_ARMSS arm_cmd_s;
}_SPI_RX;

extern _SPI_RX spi_rx;


typedef struct
{
float q_set[14];
float q_set_servo[14];
float q_reset[14];
float q_reset_servo[14];
float tau_ff[14];
float tau_ff_servo[14];
float max_i[14];
float kp[14],ki[14],kd[14];
float kp_servo[14],ki_servo[14],kd_servo[14];
float stiff[14];
float stiff_servo[14];
char en_motor,reser_q,reser_q_div[14],reser_q_div_servo[14],reset_err;
char en_servo;
char led_enable[2];
char beep_state;
_ARMSS arm_cmd_s;
}_SPI_TX;

extern _SPI_TX spi_tx;

typedef struct
{
  float x;
    float y;
    float z;
}xyz_f_t;

struct xyz_f_tr
{
    float x;
    float y;
    float z;
};
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}xyz_s16_t;

typedef struct
{
    char Acc_CALIBRATE;
    char Gyro_CALIBRATE;
    char Cali_3d;
    xyz_s16_t Acc_I16;
    xyz_s16_t Gyro_I16;
    xyz_f_t imu_pos;
    xyz_f_t imu_att;
    xyz_f_t gps_pos;
    xyz_f_t Acc;
    xyz_f_t Acc_rt;
    xyz_f_t Gyro;
    xyz_f_t Gyro_deg;
    xyz_f_t Gyro_deg_rt;
    xyz_f_t Mag,Mag_rt,Mago,Mago_rt;
    xyz_f_t Acc_Offset;
    xyz_f_t Acc_Scale;
    xyz_f_t Gyro_Offset;
    xyz_f_t Gyro_Scale;
    xyz_f_t Gyro_Auto_Offset;
    xyz_f_t Gain_3d;
    xyz_f_t Off_3d;
    char Mag_CALIBRATE,Mag_Have_Param,Mag_ERR,Mag_update;
    xyz_s16_t Mag_Adc,Mag_Adc_o;			//采样值
    xyz_f_t   Mag_Offset,Mag_Offseto;		//偏移值
    xyz_f_t   Mag_Offset_c,Mag_Offset_co;		//偏移值
    xyz_f_t   Mag_Gain,Mag_Gaino;		//偏移值
    xyz_f_t 	Mag_Gain_c,Mag_Gain_co;			//比例缩放
    xyz_f_t 	Mag_Val,Mag_Val_t,Mag_Valo,Mag_Val_to;			//纠正后的值
    float hmlOneMAG,hmlOneACC;
  float Yaw_Mag;
    float Ftempreature;
}_MEMS;

extern _MEMS mems;


//FDlink candata
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56+8  8组数据
#define AHRS_LEN 0x30   //48+8  7组数据
#define INSGPS_LEN 0x42 //72+8  10组数据
#define IMU_CAN 9
#define AHRS_CAN 8
#define INSGPS_CAN 11
#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11
#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80

typedef struct IMUData_Packet_t{
        float gyroscope_x;          //unit: rad/s
        float gyroscope_y;          //unit: rad/s
        float gyroscope_z;          //unit: rad/s
        float accelerometer_x;      //m/s^2
        float accelerometer_y;      //m/s^2
        float accelerometer_z;      //m/s^2
        float magnetometer_x;       //mG
        float magnetometer_y;       //mG
        float magnetometer_z;       //mG
        float imu_temperature;      //C
        float Pressure;             //Pa
        float pressure_temperature; //C
        long Timestamp;          //us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
    float RollSpeed;   //unit: rad/s
    float PitchSpeed;  //unit: rad/s
    float HeadingSpeed;//unit: rad/s
    float Roll;        //unit: rad
    float Pitch;       //unit: rad
    float Heading;     //unit: rad
    float Qw;//w          //Quaternion
    float Qx;//x
    float Qy;//y
    float Qz;//z
    long Timestamp; //unit: us
}AHRSData_Packet_t;

extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;
#endif
