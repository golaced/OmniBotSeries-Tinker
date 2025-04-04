/******************************************************************************
          Serial communication library between device and IMU module
Version: V1.04
Record: 1、Added accelerometer and gyroscope ranges that can be set
      2、Added magnetic field calibration start command
      3、Added command to set gyroscope automatic correction logo
      4、Added setting to trigger duration of static energy-saving mode
*******************************************************************************/
#ifndef _imu_cmd.h
#define _imu_cmd.h

#include <stdio.h>
#include <math.h>

typedef signed char            S8;
typedef unsigned char          U8;
typedef signed short           S16;
typedef unsigned short         U16;
typedef signed int             S32;
typedef unsigned int           U32;
typedef float                  F32;


typedef struct IMUData_Out{
 float att[3];
 float gyro[3];
 float acc[3];
} IMUData_Out;
extern IMUData_Out imu_o;

#define pow2(x) ((x)*(x)) // Find the square

// Conversion ratio during transmission--------------
#define scaleAccel       0.00478515625f // acceleration [-16g~+16g]    9.8*16/32768
#define scaleQuat        0.000030517578125f // Quaternions [-1~+1]         1/32768
#define scaleAngle       0.0054931640625f // angle   [-180~+180]     180/32768
#define scaleAngleSpeed  0.06103515625f // Angular velocity [-2000~+2000]    2000/32768
#define scaleMag         0.15106201171875f // magnetic field [-4950~+4950]   4950/32768
#define scaleTemperature 0.01f // temperature
#define scaleAirPressure 0.0002384185791f // air pressure [-2000~+2000]    2000/8388608
#define scaleHeight      0.0010728836f    // high [-9000~+9000]    9000/8388608

#define CmdPacket_Begin  0x49   // Start code
#define CmdPacket_End    0x4D   // end code
#define CmdPacketMaxDatSizeRx 73  // The maximum length of the data body of the data packet sent from the module
#define CmdPacketMaxDatSizeTx 31  // The maximum length of the data body of the data packet sent to the module


// ===============================Debug information switch====================================
    #define __Debug  // Use the debugging port to output debugging information. Just block this sentence without using debugging information.
    #ifdef __Debug
        #define Dbp(fmt, args...)  printf(fmt, ##args) // If you need to use debugging information, the user can connect the Dbp function name
        extern void Dbp_U8_buf(const char *sBeginInfo, const char *sEndInfo, const char *sFormat, const U8 *Buf, U32 Len);
    #else
        #define Dbp(fmt, args...)
        #define Dbp_U8_buf(sBeginInfo, sEndInfo, sFormat, Buf, Len)
    #endif


// =================================Migration interface======================================
    /**
     * Used to capture data packets, the user only needs to pass each byte of data received into this function.
     * @param byte 传Enter each byte of data received
     * @return U8 1=Complete data packet received, 0 did not obtain complete data packet
     */
    extern U8 Cmd_GetPkt(U8 byte);

    // When a valid data packet is received, a callback will be entered into Cmd_RxUnpack(U8 *buf, U8 DLen) In the function, the user can process the data in the function

    extern void imu_test(void); // The function of the test example is to monitor the operation instructions sent from the debugging serial port, and then operate the module and put it in the loop.

// ================================Module operation instructions=================================
    extern U8 targetDeviceAddress; // Communication address, set to 0-254 to specify the device address, set to 255 to not specify the device (i.e. broadcast). When communication in the form of 485 bus is required, select the device to be operated through this parameter. If it is only serial port 1-to-1 communication Just set it as broadcast address 255
    extern void Cmd_02(void);// sleep sensor
    extern void Cmd_03(void);// Wake up sensor
    extern void Cmd_18(void);// Turn off proactive data reporting
    extern void Cmd_19(void);// Enable active data reporting
    extern void Cmd_11(void);// Get functional data for 1 subscription
    extern void Cmd_10(void);// Get device properties and status
    /**
     * Set device parameters
     * @param accStill    Inertial Navigation-Station Acceleration Threshold Unitdm/s?
     * @param stillToZero Inertial navigation -static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
     * @param moveToZero  Inertial Navigation -Dynamic Zero Return Speed ​​(unit: cm/s) 0: No return to zero
     * @param isCompassOn Whether to use magnetic field fusion 0: Not used 1: Used
     * @param barometerFilter The filtering level of the barometer [value 0-3], the larger the value, the more stable it is but the worse the real-time performance.
     * @param reportHz The transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
     * @param gyroFilter    Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
     * @param accFilter     Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
     * @param compassFilter Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
     * @param Cmd_ReportTag Feature subscription tag
     */
    extern void Cmd_12(U8 accStill, U8 stillToZero, U8 moveToZero,  U8 isCompassOn, U8 barometerFilter, U8 reportHz, U8 gyroFilter, U8 accFilter, U8 compassFilter, U16 Cmd_ReportTag);
    extern void Cmd_13(void);// The three-dimensional spatial position of inertial navigation is cleared
    extern void Cmd_16(void);// Clear step count
    extern void Cmd_14(void);// Restore factory calibration parameters
    extern void Cmd_15(void);// Save current calibration parameters as factory calibration parameters
    extern void Cmd_07(void);// Simple calibration of accelerometer. When the module is stationary on the horizontal surface, send this command and wait 5 seconds after receiving the reply.
    /**
     * Accelerometer high-precision calibration
     * @param flag If the module is not in calibration status:
     *A value of 0 indicates a request to start a calibration and collect 1 data
     *A value of 255 means asking whether the device is calibrating
     *If the module is being calibrated:
     *A value of 1 indicates that the next data is to be collected
     *The value 255 means to collect the last data and end it
     */
    extern void Cmd_17(U8 flag);

    extern void Cmd_32(void);// Start magnetometer calibration
    extern void Cmd_04(void);// End magnetometer calibration
    extern void Cmd_05(void);// Z-axis angle zero
    extern void Cmd_06(void);// xyz world coordinate system cleared
    extern void Cmd_08(void);// Restore the default Z-axis pointing of the own coordinate system and restore the default world coordinate system
    /**
     * T
     * @param accMatrix accelerometer direction matrix
     * @param comMatrix Magnetometer Orientation Matrix
     */
    extern void Cmd_20(S8 *accMatrix, S8 *comMatrix);
    extern void Cmd_21(void);// Read PCB mounting orientation matrix
    /**
     * Set Bluetooth broadcast name
     *
     * @param bleName Bluetooth name (supports up to 15 characters in length, does not support Chinese)
     */
    extern void Cmd_22(const char *bleName);
    extern void Cmd_23(void);// Read the Bluetooth broadcast name
    /**
     * Set shutdown voltage and charging parameters
     * @param PowerDownVoltageFlag  Shutdown voltage selection 0=3.4V (for lithium batteries) 1=2.7V (for other dry batteries)
     * @param charge_full_mV  Charge cutoff voltage 0:3962mv 1:4002mv 2:4044mv 3:4086mv 4:4130mv 5:4175mv 6:4222mv 7:4270mv 8:4308mv 9:4349mv 10:4391mv
     * @param charge_full_mA Charge cut-off current 0:2ma 1:5ma 2:7ma 3:10ma 4:15ma 5:20ma 6:25ma 7:30ma
     * @param charge_mA      Charging current 0:20ma 1:30ma 2:40ma 3:50ma 4:60ma 5:70ma 6:80ma 7:90ma 8:100ma 9:110ma 10:120ma 11:140ma 12:160ma 13:180ma 14:200ma 15:220ma
     */
    extern void Cmd_24(U8 PowerDownVoltageFlag, U8 charge_full_mV, U8 charge_full_mA, U8 charge_mA);
    extern void Cmd_25(void);// Read shutdown voltage and charging parameters
    extern void Cmd_26(void);// Disconnect Bluetooth
    /**
     * Set user's GPIO pin
     *
     * @param M 0=Floating input, 1=Pull-up input, 2=Pull-down input, 3=Output 0, 4=Output 1
     */
    extern void Cmd_27(U8 M);
    extern void Cmd_2A(void);// Device restart
    extern void Cmd_2B(void);// Device shuts down
    /**
     * Set idle shutdown duration
     *
     * @param idleToPowerOffTime When there is no communication on the serial port and Bluetooth is broadcasting, and the continuous time reaches this many 10 minutes, it will shut down. 0 = Do not shut down.
     */
    extern void Cmd_2C(U8 idleToPowerOffTime);
    extern void Cmd_2D(void);// Read the idle shutdown time
    /**
     * Settings: Prohibit changing name and charging parameters via Bluetooth. Identification
     *
     * @param DisableBleSetNameAndCahrge 1=Prohibit changing the name and charging parameters through Bluetooth 0=Allow (default) Maybe the customer's product does not want others to change it casually using Bluetooth, just set it to 1
     */
    extern void Cmd_2E(U8 DisableBleSetNameAndCahrge);
    extern void Cmd_2F(void);// Read the prohibition of changing the name and charging parameters via Bluetooth.
    /**
     * Set the serial communication address
     *
     * @param address The device address can only be set to 0-254
     */
    extern void Cmd_30(U8 address);
    extern void Cmd_31(void);// Read the serial communication address
    /**
     *Setting up accelerometer and gyroscope range
     *
     * @param AccRange  Target acceleration range 0=2g 1=4g 2=8g 3=16g
     * @param GyroRange Target gyroscope range 0=256 1=512 2=1024 3=2048
     */
    extern void Cmd_33(U8 AccRange, U8 GyroRange);
    extern void Cmd_34(void);// Read accelerometer and gyroscope ranges
    /**
      * Set the gyroscope automatic correction logo
     *
     * @param GyroAutoFlag  1=Gyro automatic sensitivity correction on 0=Off
     */
    extern void Cmd_35(U8 GyroAutoFlag);
    extern void Cmd_36(void);// Read accelerometer and gyroscope ranges
    /**
      * Set the triggering time of static energy saving mode
     *
     * @param EcoTime_10s If the value is greater than 0, the automatic energy-saving mode is enabled (that is, the sensor does not actively report after sleeping, or it automatically enters the motion monitoring mode and pauses active reporting after being stationary for EcoTime_10s for 10 seconds) 0 = does not enable automatic energy-saving
     */
    extern void Cmd_37(U8 EcoTime_10s);
    extern void Cmd_38(void);// Read the triggering time of static energy saving mode
    /**
       * Set the current height to the specified value
     *
     * @param val The height value to be set in mm
     */
    extern void Cmd_42(S32 val);
    /**
       * Settings Automatic compensation height mark
     *
     * @param OnOff 0=off 1=on
     */
    extern void Cmd_43(U8 OnOff);
    // Read automatic compensation height mark
    extern void Cmd_44(void);
    /**
       * Set serial port baud rate
     *
     * @param BaudRate Target baud rate 0=9600 1=115200 2=230400 3=460800
     */
    extern void Cmd_47(U8 BaudRate);
    // Read serial port baud rate
    extern void Cmd_48(void);
    // Flash the LED indicator light several times
    extern void Cmd_49(void);

    extern void test_uart(void);

#endif

