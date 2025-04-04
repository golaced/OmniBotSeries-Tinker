/******************************************************************************
          Serial communication library between device and IMU module
Version: V1.04
Record: 1、Added accelerometer and gyroscope ranges that can be set
      2、Added magnetic field calibration start command
      3、Added command to set gyroscope automatic correction logo
      4、Added setting to trigger duration of static energy-saving mode
*******************************************************************************/
#include "imu_cmd.h"
#include <serial/serial.h> 
#include <sstream>

U8 targetDeviceAddress=255; // Communication address, set to 0-254 to specify the device address, set to 255 to not specify the device (i.e. broadcast). When communication in the form of 485 bus is required, select the device to be operated through this parameter. If it is only serial port 1-to-1 communication Just set it as broadcast address 255

U8 CalcSum1(U8 *Buf, int Len)
{
    U8 Sum = 0;
    while (Len-- > 0)
    {
        Sum += Buf[Len];
    }
    return Sum;
}
void *Memcpy(void *s1, const void *s2, unsigned int n)
{
    char *p1 = (char*)s1;
    const char *p2 = (char*)s2;
    if (n)
    {
        n++;
        while (--n > 0)
        {
            *p1++ = *p2++;
        }
    }
    return s1;
}
#ifdef __Debug
void Dbp_U8_buf(const char *sBeginInfo, const char *sEndInfo,
                const char *sFormat,
                const U8 *Buf, U32 Len)
{
    int i;
    if (sBeginInfo)
    {
        Dbp("%s", sBeginInfo);
    }
    for (i = 0; i < Len; i++)
    {
        Dbp(sFormat, Buf[i]);
    }
    if (sEndInfo)
    {
        Dbp("%s", sEndInfo);
    }
}
#endif


static void Cmd_Write(U8 *pBuf, int Len);
static void Cmd_RxUnpack(U8 *buf, U8 DLen);
/**
 * 发送CMD命令
 *
 * @param pDat The data body to be sent
 * @param DLen The length of the data body
 *
 * @return int 0=success, -1=failure
 */
int Cmd_PackAndTx(U8 *pDat, U8 DLen)
{
    U8 buf[50 + 5 + CmdPacketMaxDatSizeTx] =
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff}; // The first 50 bytes of the send packet cache are preambles, which are used to wake up modules that may be sleeping.

    if((DLen == 0) || (DLen > CmdPacketMaxDatSizeTx) || (pDat==NULL))
    {// Illegal parameters
        return -1;
    }

    buf[50] = CmdPacket_Begin; // Start code
    buf[51] = targetDeviceAddress; // Current device address code
    buf[52] = DLen;  // length
    Memcpy(&buf[53], pDat, DLen); // data body
    buf[53+DLen] = CalcSum1(&buf[51], DLen+2);// CS counts from the beginning of the address code to the end of the data body
    buf[54+DLen] = CmdPacket_End; // end code

    Cmd_Write(buf, DLen+55);
    return 0;
}
/**
 * Used to capture data packets, the user only needs to pass each byte of data received into this function.
 * @param byte Pass in each byte of data received
 * @return U8 1=Complete data packet received, 0 did not obtain complete data packet
 */
U8 Cmd_GetPkt(U8 byte)
{
    static U8 CS=0; // Checksum
    static U8 i=0;
    static U8 RxIndex=0;

    static U8 buf[5+CmdPacketMaxDatSizeRx]; // Receive packet cache
    #define cmdBegin    buf[0]  // Start code
    #define cmdAddress  buf[1]  // Correspondence address
    #define cmdLen      buf[2]  // length
    #define cmdDat     &buf[3]  // data body
    #define cmdCS       buf[3+cmdLen] // Checksum
    #define cmdEnd      buf[4+cmdLen] // end code

    CS += byte; // Calculate the check code while receiving the data. The check code is the sum of the data from the beginning of the address code (including the address code) to before the check code.
    switch (RxIndex)
    {
    case 0: // Start code
        if (byte == CmdPacket_Begin)
        {
            i = 0;
            buf[i++] = CmdPacket_Begin;
            CS = 0; // Calculate the check code starting from the next byte
            RxIndex = 1;
        }
        break;
    case 1: // The address code of the data body
        buf[i++] = byte;
        if (byte == 255)
        { // 255 is the broadcast address. As the module is a slave, its address cannot appear 255.
            RxIndex = 0;
            break;
        }
        RxIndex++;
        break;
    case 2: // The length of the data body
        buf[i++] = byte;
        if ((byte > CmdPacketMaxDatSizeRx) || (byte == 0))
        { // Invalid length
            RxIndex = 0;
            break;
        }
        RxIndex++;
        break;
    case 3: // Get the data of the data body
        buf[i++] = byte;
        if (i >= cmdLen+3)
        { // Data body has been received
            RxIndex++;
        }
        break;
    case 4: // Compare verification code
        CS -= byte;
        if (CS == byte)
        {// Verification is correct
            buf[i++] = byte;
            RxIndex++;
        }
        else
        {// Verification failed
            RxIndex = 0;
        }
        break;
    case 5: // end code
        RxIndex = 0;
        if (byte == CmdPacket_End)
        {// 捕Get the complete package
            buf[i++] = byte;

            if ((targetDeviceAddress == cmdAddress) || (targetDeviceAddress == 255))
            {// 地Address matching, data sent from the target device is processed
                //Dbp_U8_buf("rx: ", "\r\n",
                //           "%02X ",
                //           buf, i);
                Cmd_RxUnpack(&buf[3], i-5); // Process the data body of the packet
                return 1;
            }
        }
        break;
    default:
        RxIndex = 0;
        break;
    }

    return 0;
}


// ================================Module operation instructions=================================

// sleep sensor
void Cmd_02(void)
{
    U8 buf[1] = {0x02};
    Dbp("\r\nsensor off--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Wake up sensor
void Cmd_03(void)
{
    U8 buf[1] = {0x03};
    Dbp("\r\nsensor on--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Turn off proactive data reporting
void Cmd_18(void)
{
    U8 buf[1] = {0x18};
    Dbp("\r\nauto report off--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Enable active data reporting
void Cmd_19(void)
{
    U8 buf[1] = {0x19};
    Dbp("\r\nauto report on--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Get functional data for 1 subscription
void Cmd_11(void)
{
    U8 buf[1] = {0x11};
    Dbp("\r\nget report--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Get device properties and status
void Cmd_10(void)
{
    U8 buf[1] = {0x10};
    Dbp("\r\nget report--\r\n");
    Cmd_PackAndTx(buf, 1);
}
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
void Cmd_12(U8 accStill, U8 stillToZero, U8 moveToZero,  U8 isCompassOn, U8 barometerFilter, U8 reportHz, U8 gyroFilter, U8 accFilter, U8 compassFilter, U16 Cmd_ReportTag)
{
    U8 buf[11] = {0x12};
    buf[1] = accStill;
    buf[2] = stillToZero;
    buf[3] = moveToZero;
    buf[4] = ((barometerFilter&3)<<1) | (isCompassOn&1); // bit[2-1]: Filter level of BMP280 [value 0-3] bit[0]: 1=Magnetic field is turned on 0=Magnetic field is turned off
    buf[5] = reportHz;
    buf[6] = gyroFilter;
    buf[7] = accFilter;
    buf[8] = compassFilter;
    buf[9] = Cmd_ReportTag&0xff;
    buf[10] = (Cmd_ReportTag>>8)&0xff;
    Dbp("\r\nset parameters--\r\n");
    Cmd_PackAndTx(buf, 11);
}
// The three-dimensional spatial position of inertial navigation is cleared
void Cmd_13(void)
{
    U8 buf[1] = {0x13};
    Dbp("\r\nclear INS position--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Clear step count
void Cmd_16(void)
{
    U8 buf[1] = {0x16};
    Dbp("\r\nclear steps--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Restore factory calibration parameters
void Cmd_14(void)
{
    U8 buf[1] = {0x14};
    Dbp("\r\nRestore calibration parameters from factory mode--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Save current calibration parameters as factory calibration parameters
void Cmd_15(void)
{
    U8 buf[3] = {0x15, 0x88, 0x99};
    Dbp("\r\nSave calibration parameters to factory mode--\r\n");
    Cmd_PackAndTx(buf, 3);
}
// Simple calibration of the accelerometer. When the module is stationary on a horizontal surface, send this command and wait 9 seconds after receiving the reply.
void Cmd_07(void)
{
    U8 buf[1] = {0x07};
    Dbp("\r\nacceleration calibration--\r\n");
    Cmd_PackAndTx(buf, 1);
}
/**
 * Accelerometer high-precision calibration
 * @param flag If the module is not in calibration status:
 *A value of 0 indicates a request to start a calibration and collect 1 data
*A value of 255 means asking whether the device is calibrating
*If the module is being calibrated:
*A value of 1 indicates that the next data is to be collected
*The value 255 means to collect the last data and end it
*/
void Cmd_17(U8 flag)
{
    U8 buf[2] = {0x17};
    buf[1] = flag;
    if (flag == 0)
    {// Request to start a calibration
        Dbp("\r\ncalibration request start--\r\n");
    }
    else if (flag == 1)
    {// Request to collect the next piece of data
        Dbp("calibration request next point--\r\n");
    }
    else if (flag == 255)
    {// Request to collect the last data and end the currently ongoing calibration
        Dbp("calibration request stop--\r\n");
    }
    else
    {
        return;
    }
    Cmd_PackAndTx(buf, 2);
}
// Start magnetometer calibration
void Cmd_32(void)
{
    U8 buf[1] = {0x32};
    Dbp("\r\ncompass calibrate begin--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// End magnetometer calibration
void Cmd_04(void)
{
    U8 buf[1] = {0x04};
    Dbp("\r\ncompass calibrate end--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Z-axis angle zero
void Cmd_05(void)
{
    U8 buf[1] = {0x05};
    Dbp("\r\nz-axes to zero--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// xyz world coordinate system cleared
void Cmd_06(void)
{
    U8 buf[1] = {0x06};
    Dbp("\r\nWorldXYZ-axes to zero--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Restore the default Z-axis pointing of the own coordinate system and restore the default world coordinate system
void Cmd_08(void)
{
    U8 buf[1] = {0x08};
    Dbp("\r\naxesZ WorldXYZ-axes to zero--\r\n");
    Cmd_PackAndTx(buf, 1);
}
/**
 * Set PCB mounting orientation matrix
 * @param accMatrix accelerometer direction matrix
 * @param comMatrix Magnetometer Orientation Matrix
 */
void Cmd_20(S8 *accMatrix, S8 *comMatrix)
{
    U8 buf[19] = {0x20};
    Memcpy(&buf[1],  accMatrix, 9);
    Memcpy(&buf[10], comMatrix, 9);
    Dbp("\r\nz-axes to zero--\r\n");
    Cmd_PackAndTx(buf, 19);
}
// Read PCB mounting orientation matrix
void Cmd_21(void)
{
    U8 buf[1] = {0x21};
    Dbp("\r\nget PCB direction--\r\n");
    Cmd_PackAndTx(buf, 1);
}
/**
 * Set Bluetooth broadcast name
 * @param bleName Bluetooth name (supports up to 15 characters in length, does not support Chinese)
 */
void Cmd_22(const char *bleName)
{
    U8 buf[17] = {0x22};
    Memcpy(&buf[1],  bleName, 16);
    Dbp("\r\nset BLE name--\r\n");
    Cmd_PackAndTx(buf, 17);
}
// Read the Bluetooth broadcast name
void Cmd_23(void)
{
    U8 buf[1] = {0x23};
    Dbp("\r\nget BLE name--\r\n");
    Cmd_PackAndTx(buf, 1);
}
/**
 * Set shutdown voltage and charging parameters
 * @param PowerDownVoltageFlag  Shutdown voltage selection 0=3.4V (for lithium batteries) 1=2.7V (for other dry batteries)
 * @param charge_full_mV  Charge cutoff voltage 0:3962mv 1:4002mv 2:4044mv 3:4086mv 4:4130mv 5:4175mv 6:4222mv 7:4270mv 8:4308mv 9:4349mv 10:4391mv
 * @param charge_full_mA Charge cut-off current 0:2ma 1:5ma 2:7ma 3:10ma 4:15ma 5:20ma 6:25ma 7:30ma
 * @param charge_mA      Charging current 0:20ma 1:30ma 2:40ma 3:50ma 4:60ma 5:70ma 6:80ma 7:90ma 8:100ma 9:110ma 10:120ma 11:140ma 12:160ma 13:180ma 14:200ma 15:220ma
 */
void Cmd_24(U8 PowerDownVoltageFlag, U8 charge_full_mV, U8 charge_full_mA, U8 charge_mA)
{
    U8 buf[5] = {0x24};
    buf[1] = (PowerDownVoltageFlag <= 1)? PowerDownVoltageFlag:1;
    buf[2] = (charge_full_mV <= 10)? charge_full_mV:10;
    buf[3] = (charge_full_mA <= 7)? charge_full_mA:7;
    buf[4] = (charge_mA <= 15)? charge_mA:15;
    Dbp("\r\nset PowerDownVoltage and charge parameters--\r\n");
    Cmd_PackAndTx(buf, 5);
}
// Read shutdown voltage and charging parameters
void Cmd_25(void)
{
    U8 buf[1] = {0x25};
    Dbp("\r\nget PowerDownVoltage and charge parameters--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Disconnect the Bluetooth connection. There will be no reply to this command.
void Cmd_26(void)
{
    U8 buf[1] = {0x26};
    Dbp("\r\nble disconnect--\r\n");
    Cmd_PackAndTx(buf, 1);
}
/**
 * Set user's GPIO pin
 *
 * @param M 0=Floating input, 1=Pull-up input, 2=Pull-down input, 3=Output 0, 4=Output 1
 */
void Cmd_27(U8 M)
{
    U8 buf[2] = {0x27};
    buf[1] = (M<<4)&0xf0;
    Dbp("\r\nset gpio--\r\n");
    Cmd_PackAndTx(buf, 2);
}

// Device restart
void Cmd_2A(void)
{
    U8 buf[1] = {0x2A};
    Dbp("\r\nreset--\r\n");
    Cmd_PackAndTx(buf, 1);
}
// Device shuts down
void Cmd_2B(void)
{
    U8 buf[1] = {0x2B};
    Dbp("\r\npower off--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Set idle shutdown duration
 *
 * @param idleToPowerOffTime When there is no communication on the serial port and Bluetooth is broadcasting, and the continuous time reaches this many 10 minutes, it will shut down. 0 = Do not shut down.
 */
void Cmd_2C(U8 idleToPowerOffTime)
{
    U8 buf[2] = {0x2C};
    buf[1] = idleToPowerOffTime;
    Dbp("\r\nset idleToPowerOffTime--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read the idle shutdown time
void Cmd_2D(void)
{
    U8 buf[1] = {0x2D};
    Dbp("\r\nget idleToPowerOffTime--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Settings: Prohibit changing name and charging parameters via Bluetooth. Identification
 *
 * @param DisableBleSetNameAndCahrge 1=Prohibit changing the name and charging parameters through Bluetooth 0=Allow (default) Maybe the customer's product does not want others to change it casually using Bluetooth, just set it to 1
 */
void Cmd_2E(U8 DisableBleSetNameAndCahrge)
{
    U8 buf[2] = {0x2E};
    buf[1] = (DisableBleSetNameAndCahrge <= 1)? DisableBleSetNameAndCahrge:1;
    Dbp("\r\nset FlagForDisableBleSetNameAndCahrge--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read the prohibition of changing the name and charging parameters via Bluetooth.
void Cmd_2F(void)
{
    U8 buf[1] = {0x2F};
    Dbp("\r\nget FlagForDisableBleSetNameAndCahrge--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Set serial communication address
 *
 * @param address The device address can only be set to 0-254
 */
void Cmd_30(U8 address)
{
    U8 buf[2] = {0x30};
    buf[1] = (address <= 254)? address:254;
    Dbp("\r\nset address--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read serial communication address
void Cmd_31(void)
{
    U8 buf[1] = {0x31};
    Dbp("\r\nget address--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Setting up accelerometer and gyroscope range
 *
 * @param AccRange  Target acceleration range 0=2g 1=4g 2=8g 3=16g
 * @param GyroRange Target gyroscope range 0=256 1=512 2=1024 3=2048
 */
void Cmd_33(U8 AccRange, U8 GyroRange)
{
    U8 buf[3] = {0x33};
    buf[1] = (AccRange <= 3)? AccRange:3;
    buf[2] = (GyroRange <= 3)? GyroRange:3;
    Dbp("\r\nset accelRange and gyroRange--\r\n");
    Cmd_PackAndTx(buf, 3);
}
// Read accelerometer and gyroscope ranges
void Cmd_34(void)
{
    U8 buf[1] = {0x34};
    Dbp("\r\nget accelRange and gyroRange--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Set the gyroscope automatic correction logo
 *
 * @param GyroAutoFlag  1=Gyro automatic sensitivity correction on 0=Off
 */
void Cmd_35(U8 GyroAutoFlag)
{
    U8 buf[2] = {0x35};
    buf[1] = (GyroAutoFlag > 0)? 1:0;
    Dbp("\r\nset GyroAutoFlag--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read accelerometer and gyroscope ranges
void Cmd_36(void)
{
    U8 buf[1] = {0x36};
    Dbp("\r\nget GyroAutoFlag--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Set the triggering time of static energy saving mode
 *
 * @param EcoTime_10s If the value is greater than 0, the automatic energy-saving mode is enabled (that is, the sensor does not actively report after sleeping, or it automatically enters the motion monitoring mode and pauses active reporting after being stationary for EcoTime_10s for 10 seconds) 0 = does not enable automatic energy-saving
 */
void Cmd_37(U8 EcoTime_10s)
{
    U8 buf[2] = {0x37};
    buf[1] = EcoTime_10s;
    Dbp("\r\nset EcoTime_10s--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read accelerometer and gyroscope ranges
void Cmd_38(void)
{
    U8 buf[1] = {0x38};
    Dbp("\r\nget EcoTime_10s--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Set the current height to the specified value
 *
 * @param val The height value to be set in mm
 */
void Cmd_42(S32 val)
{
    U8 buf[5] = {0x42};
    buf[1] = (U8)(val & 0xFF);
    buf[2] = (U8)((val >> 8) & 0xFF);
    buf[3] = (U8)((val >> 16) & 0xFF);
    buf[4] = (U8)((val >> 24) & 0xFF);
    Dbp("\r\nset height--\r\n");
    Cmd_PackAndTx(buf, 5);
}

/**
 * Settings Automatic compensation height mark
 *
 * @param OnOff 0=off 1=on
 */
void Cmd_43(U8 OnOff)
{
    U8 buf[2] = {0x43};
    buf[1] = OnOff;
    Dbp("\r\nset HeightAutoFlag--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read automatic compensation height mark
void Cmd_44(void)
{
    U8 buf[1] = {0x44};
    Dbp("\r\nget HeightAutoFlag--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * Set serial port baud rate
 *
 * @param BaudRate Target baud rate 0=9600 1=115200 2=230400 3=460800
 */
void Cmd_47(U8 BaudRate)
{
    U8 buf[2] = {0x47};
    buf[1] = BaudRate;
    Dbp("\r\nset BaudRate--\r\n");
    Cmd_PackAndTx(buf, 2);
}
// Read serial port baud rate
void Cmd_48(void)
{
    U8 buf[1] = {0x48};
    Dbp("\r\nget BaudRate--\r\n");
    Cmd_PackAndTx(buf, 1);
}

// Flash the LED indicator light several times
void Cmd_49(void)
{
    U8 buf[1] = {0x49};
    Dbp("\r\nLed blink--\r\n");
    Cmd_PackAndTx(buf, 1);
}

/**
 * The data body of the received message is parsed and processed. Users can just focus on the corresponding content according to project requirements.--------------------
 * @param pDat The data body to be parsed
 * @param DLen The length of the data body
 */
static void Cmd_RxUnpack(U8 *buf, U8 DLen)
{
    U16 ctl;
    U8 L;
    U8 tmpU8;
    U16 tmpU16;
    U32 tmpU32;
    F32 tmpX, tmpY, tmpZ, tmpAbs;

    switch (buf[0])
    {
    case 0x02: // Sensor is asleep Reply
        Dbp("\t sensor off success\r\n");
        break;
    case 0x03: // Sensor woken up Reply
        Dbp("\t sensor on success\r\n");
        break;
    case 0x32: // Magnetometer Start Calibration Reply
        Dbp("\t compass calibrate begin\r\n");
        break;
    case 0x04: // Magnetometer End Calibration Reply
        Dbp("\t compass calibrate end\r\n");
        break;
    case 0x05: // Z-axis angle has been reset to zero Reply
        Dbp("\t z-axes to zero success\r\n");
        break;
    case 0x06: // Request to clear xyz world coordinate system Reply
        Dbp("\t WorldXYZ-axes to zero success\r\n");
        break;
    case 0x07: // Accelerometer simple calibration is in progress and will be completed in 9 seconds Reply
        Dbp("\t acceleration calibration, Hold still for 9 seconds\r\n");
        break;
    case 0x08: //Restore the default Z-axis pointing of the own coordinate system and restore the default world coordinate system Reply
        Dbp("\t axesZ WorldXYZ-axes to zero success\r\n");
        break;
    case 0x10: // The current properties and status of the module Reply
        Dbp("\t still limit: %u\r\n", buf[1]);   // Byte 1 Inertial Navigation-Stationary State Acceleration Threshold Unit: dm/s?
        Dbp("\t still to zero: %u\r\n", buf[2]); // Byte 2 Inertial navigation -static zero return speed (unit mm/s) 0: No return to zero 255: Return to zero immediately
        Dbp("\t move to zero: %u\r\n", buf[3]);  // Byte 3 Inertial Navigation -Dynamic Zero Return Speed ​​(unit: mm/s) 0: No return to zero
        Dbp("\t compass: %s\r\n", ((buf[4]>>0) & 0x01)? "on":"off" );     // Byte 4 bit[0]: 1=Magnetic field is on 0=Magnetic field is off
        Dbp("\t barometer filter: %u\r\n", (buf[4]>>1) & 0x03);           // Byte 4 bit[1-2]: The filtering level of the barometer [value 0-3], the larger the value, the more stable it is but the worse the real-time performance.
        Dbp("\t IMU: %s\r\n", ((buf[4]>>3) & 0x01)? "on":"off" );         // Byte 4 bit[3]: 1=sensor is on 0=sensor is sleeping
        Dbp("\t auto report: %s\r\n", ((buf[4]>>4) & 0x01)? "on":"off" ); // Byte 4 bit[4]: 1=Active reporting of sensor data is turned on 0=Active reporting of sensor data is turned off
        Dbp("\t FPS: %u\r\n", buf[5]); // Byte 5 Transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
        Dbp("\t gyro filter: %u\r\n", buf[6]);    // Byte 6 gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
        Dbp("\t acc filter: %u\r\n", buf[7]);     // Byte 7 accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
        Dbp("\t compass filter: %u\r\n", buf[8]); // Byte 8 magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
        Dbp("\t subscribe tag: 0x%04X\r\n", (U16)(((U16)buf[10]<<8) | buf[9])); // Bytes [10-9] Function subscription identifier
        Dbp("\t charged state: %u\r\n", buf[11]); // Byte 11 charging status indication 0=not connected to power 1=charging 2=fully charged
        Dbp("\t battery level: %u%%\r\n", buf[12]); // Byte 12 Current remaining power [0-100%]
        Dbp("\t battery voltage: %u mv\r\n", (U16)(((U16)buf[14]<<8) | buf[13])); // Bytes[14-13] The current voltage mv of the battery
        Dbp("\t Mac: %02X:%02X:%02X:%02X:%02X:%02X\r\n", buf[15],buf[16],buf[17],buf[18],buf[19],buf[20]); // Bytes[15-20] MAC address
        //Dbp("\t version: %s\r\n", &buf[21]); // Bytes[21-26] Firmware version string
        //Dbp("\t product model: %s\r\n", &buf[27]); // Bytes[26-32] Product model string
        break;
    case 0x11: // Get subscribed functional data Reply or proactively report
        {
//            sensor_msgs::Imu imu_data;//IMU data to publish
//            imu_data.header.stamp = ros::Time::now();
//            imu_data.header.frame_id = "base_imu_link";
            
            ctl = ((U16)buf[2] << 8) | buf[1];// Byte [2-1] is the function subscription identifier, indicating which functions are currently subscribed.
            //Dbp("\t subscribe tag: 0x%04X\r\n", ctl);
            //Dbp("\t ms: %u\r\n", (U32)(((U32)buf[6]<<24) | ((U32)buf[5]<<16) | ((U32)buf[4]<<8) | ((U32)buf[3]<<0))); // Bytes[6-3] is the timestamp after the module is powered on (unit: ms)

            L =7; // Starting from the 7th byte, the remaining data is parsed according to the subscription identification tag.
            if ((ctl & 0x0001) != 0)
            {// Acceleration xyz removes gravity and requires *scaleAccel m/s when used
                tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\taX: %.3f\r\n", tmpX); //  Acceleration ax without gravity
                tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\taY: %.3f\r\n", tmpY); // Acceleration ay without gravity
                tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\taZ: %.3f\r\n", tmpZ); // Acceleration az without gravity
                tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\ta_abs: %.3f\r\n", tmpAbs); // Absolute value of 3-axis composite
                //imu_data.linear_acceleration.x = tmpX;
                //imu_data.linear_acceleration.y = tmpY;
                //imu_data.linear_acceleration.z = tmpZ;
                imu_o.acc[0]= tmpX;
                imu_o.acc[1]= tmpY;
                imu_o.acc[2]= tmpZ;
            }
            if ((ctl & 0x0002) != 0)
            {// Acceleration xyz includes gravity. *scaleAccel m/s is required when using it.
                tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tAX: %.3f\r\n", tmpX); // Acceleration AX with gravity
                tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tAY: %.3f\r\n", tmpY); // Acceleration AY with gravity
                tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; //Dbp("\tAZ: %.3f\r\n", tmpZ); // Acceleration AZ with gravity
                tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\tA_abs: %.3f\r\n", tmpAbs); // Absolute value of 3-axis composite
//                imu_data.linear_acceleration.x = tmpX;
//                imu_data.linear_acceleration.y = tmpY;
//                imu_data.linear_acceleration.z = tmpZ;
                imu_o.acc[0]= tmpX;
                imu_o.acc[1]= tmpY;
                imu_o.acc[2]= tmpZ;
            }
            if ((ctl & 0x0004) != 0)
            {// Angular velocity xyz requires *scaleAngleSpeed ​​rad/s when used
                tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; //Dbp("\tGX: %.3f\r\n", tmpX); // Angular velocity GX
                tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; //Dbp("\tGY: %.3f\r\n", tmpY); // Angular velocity GY
                tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; //Dbp("\tGZ: %.3f\r\n", tmpZ); // Angular velocity GZ
                tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\tG_abs: %.3f\r\n", tmpAbs); // Absolute value of 3-axis composite
//                imu_data.angular_velocity.x = tmpX * 0.0174532925; // unit rad/s
//                imu_data.angular_velocity.y = tmpY * 0.0174532925; // unit rad/s
//                imu_data.angular_velocity.z = tmpZ * 0.0174532925; // unit rad/s
                imu_o.gyro[0]= tmpX;
                imu_o.gyro[1]= tmpY;
                imu_o.gyro[2]= tmpZ;
            }
            if ((ctl & 0x0008) != 0)
            {// Magnetic field xyz requires *scaleMag uT when using it
                tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; //Dbp("\tCX: %.3f\r\n", tmpX); // Magnetic field data CX
                tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; //Dbp("\tCY: %.3f\r\n", tmpY); // Magnetic field data CY
                tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; //Dbp("\tCZ: %.3f\r\n", tmpZ); // Magnetic field data CZ
                tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); //Dbp("\tC_abs: %.3f\r\n", tmpAbs); // Absolute value of 3-axis composite
            }
            if ((ctl & 0x0010) != 0)
            {// temperature air pressure altitude
                tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleTemperature; L += 2; //Dbp("\ttemperature: %.2f\r\n", tmpX); // temperature

                tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
                tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
                tmpY = (S32)tmpU32 * scaleAirPressure; L += 3; //Dbp("\tairPressure: %.3f\r\n", tmpY); // air pressure

                tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
                tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
                tmpZ = (S32)tmpU32 * scaleHeight; L += 3; //Dbp("\theight: %.3f\r\n", tmpZ); // high
            }
            if ((ctl & 0x0020) != 0)
            {// *scaleQuat is required when using the four-element wxyz
                tmpAbs = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; //Dbp("\tw: %.3f\r\n", tmpAbs); // Quaternions w
                tmpX =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; //Dbp("\tx: %.3f\r\n", tmpX); // Quaternions x
                tmpY =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; //Dbp("\ty: %.3f\r\n", tmpY); // Quaternions y
                tmpZ =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; //Dbp("\tz: %.3f\r\n", tmpZ); // Quaternions z
//                imu_data.orientation.x = tmpX;
//                imu_data.orientation.y = tmpY;
//                imu_data.orientation.z = tmpZ;
//                imu_data.orientation.w = tmpAbs;
            }
            if ((ctl & 0x0040) != 0)
            {// Euler angle xyz requires *scaleAngle when using it
                tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleX: %.3f\r\n", tmpX); // Euler angles x
                tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleY: %.3f\r\n", tmpY); // Euler angles Y
                tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; //Dbp("\tangleZ: %.3f\r\n", tmpZ); // Euler angles Z
                imu_o.att[0]= tmpX;
                imu_o.att[1]= tmpY;
                imu_o.att[2]= tmpZ;
            }

        }
        break;
    case 0x12: // Set parameters Reply
        Dbp("\t set parameters success\r\n");
        break;
    case 0x13: // Inertial navigation three-dimensional space position reset Reply
        Dbp("\t clear INS position success\r\n");
        break;
    case 0x14: // Restore factory calibration parameters Reply
        Dbp("\t Restore calibration parameters from factory mode success\r\n");
        break;
    case 0x15: //Save the current calibration parameters as factory calibration parameters Reply
        Dbp("\t Save calibration parameters to factory mode success\r\n");
        break;
    case 0x16: // Clear step count Reply
        Dbp("\t clear steps success\r\n");
        break;
    case 0x17: // Accelerometer High Accuracy Calibration Reply
        if (buf[1] == 255)
        {//Byte 1 value 255 indicates that the acquisition is completed and the calibration is ending (the device needs to remain stationary and wait for 10 seconds)
            Dbp("\t calibration success, please wait 10 seconds\r\n");
        }
        else if (buf[1] == 254)
        {// Byte 1 value 254 indicates that the gyroscope self-test failed.
            Dbp("\t calibration fail, gyro error\r\n");
        }
        else if (buf[1] == 253)
        {// Byte 1 value 253 Indicates accelerometer self-test failure
            Dbp("\t calibration fail, accelerometer error\r\n");
        }
        else if (buf[1] == 252)
        {//Byte 1 value 252 indicates that the magnetometer self-test failed.
            Dbp("\t calibration fail, compass error\r\n");
        }
        else if (buf[1] == 251)
        {// Byte 1 value 251 indicates that the device is not in calibration
            Dbp("\t calibration fail, Hasn't started\r\n");
        }
        else if (buf[1] != 0)
        {//Value [1-250] indicates the current number of collections
            Dbp("\t calibration, Points collected is %u\r\n", buf[1]);
        }
        else
        {// A value of 0 indicates that the module is already in calibration
            Dbp("\t calibration is running\r\n");
        }
        break;
    case 0x18: // Active reporting has been closed Reply
        Dbp("\t auto report off\r\n");
        break;
    case 0x19: // Active reporting has been turned on Reply
        Dbp("\t auto report on\r\n");
        break;
    case 0x20: // Set PCB mounting orientation matrix Reply
        Dbp("\t set PCB direction success\r\n");
        break;
    case 0x21: // is a request to read the installation orientation matrix
        Dbp_U8_buf("\t get PCB direction: 0x[", "]\r\n",
                   "%02x ",
                   &buf[1], 9); // Bytes [1-9] install the orientation matrix for the accelerometer
        Dbp_U8_buf("\t get PCB direction: 0x[", "]\r\n",
                   "%02x ",
                   &buf[10], 9); // Bytes [10-18] install the orientation matrix for the magnetometer
        break;
    case 0x22: // Yes Request Set Bluetooth Broadcast Name
        Dbp("\t set BLE name success\r\n");
        break;
    case 0x23: // Read bluetooth broadcast name Reply
        Dbp("\t get BLE name: %s\r\n", &buf[1]); // Bytes [1-16] are Bluetooth broadcast name strings
        break;
    case 0x24: // Set shutdown voltage and charging parameters Reply
        Dbp("\t set PowerDownVoltage and charge parameters success\r\n");
        break;
    case 0x25: // Read shutdown voltage and charging parameters Reply
        Dbp("\t PowerDownVoltageFlag: %u\r\n", buf[1]); // Byte 1 shutdown voltage selection flag 0 means 3.4V, 1 means 2.7V
        Dbp("\t charge_full_mV: %u\r\n", buf[2]); // Byte 2 Charging cut-off voltage 0:3962mv 1:4002mv 2:4044mv 3:4086mv 4:4130mv 5:4175mv 6:4222mv 7:4270mv 8:4308mv 9:4349mv 10:4391mv
        Dbp("\t charge_full_mA: %u ma\r\n", buf[3]); // Byte 3 charge cut-off current 0:2ma 1:5ma 2:7ma 3:10ma 4:15ma 5:20ma 6:25ma 7:30ma
        Dbp("\t charge_mA: %u ma\r\n", buf[4]); // Byte 3 Charging current 0:20ma 1:30ma 2:40ma 3:50ma 4:60ma 5:70ma 6:80ma 7:90ma 8:100ma 9:110ma 10:120ma 11:140ma 12:160ma 13:180ma 14:200ma 15:220ma
        break;
    case 0x27: // Set user’s GPIO pins Reply
        Dbp("\t set gpio success\r\n");
        break;
    case 0x2A: // Restart device Reply
        Dbp("\t will reset\r\n");
        break;
    case 0x2B: // Device shuts down Reply
        Dbp("\t will power off\r\n");
        break;
    case 0x2C: // Set idle shutdown duration Reply
        Dbp("\t set idleToPowerOffTime success\r\n");
        break;
    case 0x2D: // Read the idle shutdown time Reply
        Dbp("\t idleToPowerOffTime:%u minutes\r\n", buf[1]*10);
        break;
    case 0x2E: // Set to disable changing the name and charging parameter identification via Bluetooth Reply
        Dbp("\t set FlagForDisableBleSetNameAndCahrge success\r\n");
        break;
    case 0x2F: // Read the prohibition of Bluetooth method to change the name and charging parameter logo Reply
        Dbp("\t FlagForDisableBleSetNameAndCahrge:%u\r\n", buf[1]);
        break;
    case 0x30: //Set serial communication address Reply
        Dbp("\t set address success\r\n");
        break;
    case 0x31: // Read serial communication address Reply
        Dbp("\t address:%u\r\n", buf[1]);
        break;
    case 0x33: // Set accelerometer and gyroscope range Reply
        Dbp("\t set accelRange and gyroRange success\r\n");
        break;
    case 0x34: // Reading accelerometer and gyroscope ranges Reply
        Dbp("\t accelRange:%u gyroRange:%u\r\n", buf[1], buf[2]);
        break;
    case 0x35: // Set gyroscope auto-correction flag Reply
        Dbp("\t set GyroAutoFlag success\r\n");
        break;
    case 0x36: // Read the gyroscope automatic correction mark Reply
        Dbp("\t GyroAutoFlag:%u\r\n", buf[1]);
        break;
    case 0x37: // Set the triggering duration of static energy saving mode Reply
        Dbp("\t set EcoTime success\r\n");
        break;
    case 0x38: // Read the trigger duration of the stationary energy-saving mode Reply
        Dbp("\t EcoTime:%u\r\n", buf[1]);
        break;
    case 0x42: // Set height to specified value Reply
        Dbp("\t set Height success\r\n");
        break;
    case 0x43: // Set automatic compensation height mark Reply
        Dbp("\t set HeightAutoFlag success\r\n");
        break;
    case 0x44: // Read the automatic compensation height mark Reply
        Dbp("\t HeightAutoFlag:%u\r\n", buf[1]);
        break;
    case 0x47: // Set serial port baud rate Reply
        Dbp("\t set BaudRate success\r\n");
        break;
    case 0x48: // Read serial port baud rate Reply
        Dbp("\t BaudRate:%u\r\n", buf[1]);
        break;
    case 0x50: // It is the transparently transmitted data. Reply: Print out the transparently transmitted data in hexadecimal.
        Dbp_U8_buf("DTU: ", "\r\n",
           "%02X ",
           &buf[1], DLen-1);
        break;
    case 0x51: // Setup to use turns instead of Euler angles for transmission Reply
        Dbp("\t set Cycle success\r\n");
        break;

    default:
        break;
    }
}

/**
 * Sending data requires the user to implement the serial port sending data method.-------------------------------------
 * @param pBuf Content pointer to send
 * @param Len number of bytes to send
 */
static void Cmd_Write(U8 *pBuf, int Len)
{
    // The communication data stream is sent through the drv_UART_Write function. The user implements the drv_UART_Write function for the underlying hardware and sends out the Len byte data pointed to by the buf pointer.
    extern int UART_Write(const U8 *buf, int Len);
    UART_Write(pBuf, Len);

    Dbp_U8_buf("tx: ", "\r\n",
               "%02X ",
               pBuf, Len);
}












// ======================================test==============================================
U8 imu_ctl = 0;
void imu_test(void)
{
    U8 ctl = imu_ctl;
    if (imu_ctl)
    {
        imu_ctl = 0;
    }

    switch (ctl)
    {
    case '1':// 1 sleep sensor
        Cmd_02();
        break;
    case '2':// 2 Wake up sensor
        Cmd_03();
        break;

    case '3':// 3 Turn off proactive data reporting
        Cmd_18();
        break;
    case '4':// 4 Enable active data reporting
        Cmd_19();
        break;

    case '5':// 5 Get functional data for 1 subscription
        Cmd_11();
        break;

    case '6':// 6 Get device properties and status
        Cmd_10();
        break;
    case '7':// 7 Set device parameters(content 1)
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
        Cmd_12(5, 255, 0,  1, 3, 2, 2, 4, 9, 0xFFFF);
        break;
    case '8':// 8 Set device parameters (content 2)
        Cmd_12(8,   6, 5,  0, 1,30, 1, 2, 7, 0x0002);
        break;

    case '9':// 9 The three-dimensional spatial position of inertial navigation is cleared
        Cmd_13();
        break;
    case 'a':// a Clear step count
        Cmd_16();
        break;

    case 'b':// b Restore factory calibration parameters
        Cmd_14();
        break;
    case 'c':// c Save current calibration parameters as factory calibration parameters
        Cmd_15();
        break;

    case 'd':// d Accelerometer Easy Calibration
        Cmd_07();
        break;

    case 'e':// e Accelerometer High-Precision Calibration Get Started
        /**
         * Accelerometer high-precision calibration
         * @param flag If the module is not in calibration status:
         *A value of 0 indicates a request to start a calibration and collect 1 data
         *A value of 255 means asking whether the device is calibrating
         *If the module is being calibrated:
         *A value of 1 indicates that the next data is to be collected
         *The value 255 means to collect the last data and end it
         */
        Cmd_17(0);
        break;
    case 'f':// f High-precision calibration of accelerometer. Collect 1 point. Please collect at least 6 points on the stationary surface.
        /**
         * Accelerometer high-precision calibration
         * @param flag If the module is not in calibration status:
         *A value of 0 indicates a request to start a calibration and collect 1 data
         *A value of 255 means asking whether the device is calibrating
         *If the module is being calibrated:
         *A value of 1 indicates that the next data is to be collected
         *The value 255 means to collect the last data and end it
         */
        Cmd_17(1);
        break;
    case 'g':// g Accelerometer high-precision calibration. Collect the last data and end or ask the device whether it is calibrating.
        /**
         * Accelerometer high-precision calibration
         * @param flag If the module is not in calibration status:
         *A value of 0 indicates a request to start a calibration and collect 1 data
         *A value of 255 means asking whether the device is calibrating
         *If the module is being calibrated:
         *A value of 1 indicates that the next data is to be collected
         *The value 255 means to collect the last data and end it
         */
        Cmd_17(255);
        break;
    case 'H':// H Start magnetometer calibration
        Cmd_32();
    case 'h':// h End magnetometer calibration
        Cmd_04();
        break;
    case 'i':// i Z-axis angle zero
        Cmd_05();
        break;
    case 'j':// j xyz world coordinate system cleared
        Cmd_06();
        break;
    case 'k':// k Restore the default Z-axis pointing of the own coordinate system and restore the default world coordinate system
        Cmd_08();
        break;

    case 'l':// l Set the PCB installation direction matrix as the silk screen marking direction of the module
        {
            S8 accMatrix[9] =
            {// The accelerometer is in the same direction as the module logo.
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            };
            S8 comMatrix[9] =
            {// The direction of the magnetometer and module identification are consistent.
                1, 0, 0,
                0,-1, 0,
                0, 0,-1
            };
            Cmd_20(accMatrix, comMatrix);
        }
        break;
    case 'm':// m Set the PCB installation direction matrix and rotate it 90 degrees forward around the x-axis for the silk screen marking direction of the module. Place it vertically.
        {
            S8 accMatrix[9] =
            {// Place it vertically, that is, rotate the accelerometer forward 90 degrees around the x-axis of the module. OK
                1, 0, 0,
                0, 0,-1,
                0, 1, 0
            };
            S8 comMatrix[9] =
            {// Place it vertically, that is, rotate the magnetometer 90 degrees forward around the x-axis of the module. OK
                1, 0, 0,
                0, 0, 1,
                0,-1, 0
            };
            Cmd_20(accMatrix, comMatrix);
        }
        break;
    case 'n': // n Read PCB mounting orientation matrix
        Cmd_21();
        break;

    case 'o': // o Set the Bluetooth broadcast name to imu
        
        Cmd_22("imu");
        break;
    case 'p': // p Set the Bluetooth broadcast name to helloBle
        Cmd_22("helloBle");
        break;
    case 'q': // q Read the Bluetooth broadcast name
        Cmd_23();
        break;

    case 'r': // r Set to shutdown voltage 2.7V, charging cut-off voltage 4.22V, charging cut-off current 10ma, charging current 50ma. This is the factory default configuration of the module.
        Cmd_24(  1,                   6,                 3,              3);
        break;
    case 's': // s Set to shutdown voltage 3.4V, charging cut-off voltage 4.22V, charging cut-off current 15ma, charging current 200ma
        Cmd_24(  0,                   6,                 4,              14);
        break;
    case 't': // t Read AD1 pin voltage detection range, battery type, shutdown voltage
        Cmd_25();
        break;

    case 'u': // u Disconnect Bluetooth
        Cmd_26();
        break;

    case 'v': // v Set the user's GPIO pin as a pull-up input
        Cmd_27(1);
        break;
    case 'w': // w Set the user's GPIO pin as a pull-down input
        Cmd_27(2);
        break;


    case 'x': // x Restart device
        Cmd_2A();
        break;
    case 'y': // y Device shuts down
        Cmd_2B();
        break;

    case 'z': // z Set the idle shutdown duration
        Cmd_2C(0); // Does not automatically shut down when idle
        break;
    case 'A': // A Set the idle shutdown duration
        Cmd_2C(144); // Automatically shut down after 1 day of continuous idle time (this is also the factory default value)
        break;
    case 'B': // B Read the idle shutdown time
        Cmd_2D();
        break;

    case 'C': // C Set to prohibit changing name and charging parameter identification via Bluetooth
        Cmd_2E(0); // Set to Allow (this is also the factory default)
        break;
    case 'D': // D Set the setting to prohibit Bluetooth mode to change the name and charging parameter identification
        Cmd_2E(1); // Set as prohibited
        break;
    case 'E': // E Read the prohibition of changing the name and charging parameters of the Bluetooth method.
        Cmd_2F();
        break;

    case 'F': // F Set the serial port communication address. It can only be set to0-254
        Cmd_30(0); // Set to 0 (this is also the factory default)
        break;
    case 'G': // G Set serial communication address
        Cmd_30(1); // set to 1
        break;
    case 'I': // I Read the serial communication address
        Cmd_31();
        break;

    case 'J': // J Set accelerometer and gyroscope range
       /**
        * Setting up accelerometer and gyroscope range
        * @param AccRange  target acceleration range 0=2g 1=4g 2=8g 3=16g
        * @param GyroRange Target gyroscope range 0=256 1=512 2=1024 3=2048
        */
        Cmd_33(3, 3); // Set to accelerometer 16g, gyroscope 2048dps
        break;
    case 'K': // K Set accelerometer and gyroscope range
        Cmd_33(1, 2); // Set to accelerometer 4g, gyroscope 1024dps
        break;
    case 'L': // L Read accelerometer and gyroscope ranges
        Cmd_34();
        break;

    case 'M': // M Set the gyroscope automatic correction flag 1=gyro automatic correction sensitivity on 0=off
        Cmd_35(1); // Set to 1 to enable (this is also the factory default value)
        break;
    case 'N': // N Set the gyroscope automatic correction flag
        Cmd_35(0); // Set to 0 to turn off
        break;
    case 'O': // O Read the gyroscope automatic correction mark
        Cmd_36();
        break;

    case 'P': // P Set the trigger duration of the stationary energy-saving mode EcoTime_10s greater than 0, then the automatic energy-saving mode is enabled (that is, the sensor does not actively report after sleeping, or the sensor automatically enters the motion monitoring mode for 10 seconds after it EcoTime_10s is stationary for 10 seconds and suspends the active reporting) 0=Auto energy-saving is not enabled
        Cmd_37(0); // Set to 0 to turn off (this is also the factory default value)
        break;
    case 'Q': // Q Sets the duration for which the stationary energy-saving mode is triggered
        Cmd_37(6*5); // Set to 5 minutes
        break;
    case 'R': // R Read the triggering time of static energy saving mode
        Cmd_38();
        break;

    }
}

