/*************************************************************
    FileName : serialport.c
    FileFunc : 定义实现文件
    Version  : V0.1
    Author   : Sunrier
    Date     : 2012-06-13
    Descp    : Linux下实现串口库
*************************************************************/
/*#include "serialport.h"*/
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int open_port(int iPortNumber)
{
    int fd = -1;
    char *pDev[]={"/dev/ttyACM0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3",
                                "/dev/ttyS4","/dev/ttyS5","/dev/ttyS6","/dev/ttyS7",
                                "/dev/ttyS8","/dev/ttyS9","/dev/ttyS10","/dev/ttyS11"};

    switch( iPortNumber )
    {
        case	1:
        case	2:
        case	3:
        case	4:
        case	5:
        case  6:
        case  7:
        case  8:
        case  9:
        case  10:
        case  11:
        case  12:
        fd = open(pDev[iPortNumber-1],O_RDWR|O_NOCTTY|O_NDELAY);
        if( fd<0 )
        {
            perror("Can't Open Serial Port !");
            return (-1);
        }
        else
        {
            printf("Open ttyS%d ......\n",iPortNumber-1);
        }
        break;
        default:
                        /*perror("Don't exist iPortNumber !");*/
                        printf("Don't exist iPortNumber%d under /dev/? !\n",iPortNumber);
                        return (-1);
    }

    if( fcntl(fd,F_SETFL,0)<0 )/*恢复串口的状态为阻塞状态，用于等待串口数据的读入*/
    {
        printf("fcntl failed !\n");
        return (-1);
    }
    else
    {
        printf("fcntl = %d !\n",fcntl(fd,F_SETFL,0));
    }

    /*测试打开的文件描述符是否应用一个终端设备，以进一步确认串口是否正确打开*/
    if( !isatty(STDIN_FILENO) )
    {
        printf("Standard input isn't a terminal device !\n");
        return (-1);
    }
    else
    {
        printf("It's a serial terminal device!\n");
    }

    printf("open_port file ID= %d !\n",fd);

    return fd;

}

int set_port(int fd,int iBaudRate,int iDataSize,char cParity,int iStopBit)
{
    int iResult = 0;
    struct termios oldtio,newtio;


    iResult = tcgetattr(fd,&oldtio);/*保存原先串口配置*/
    if( iResult )
    {
        perror("Can't get old terminal description !");
        return (-1);
    }


    bzero(&newtio,sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;/*设置本地连接和接收使用*/

    /*设置输入输出波特率*/
    switch( iBaudRate )
    {
        case 2400:
                            cfsetispeed(&newtio,B2400);
                            cfsetospeed(&newtio,B2400);
                            break;
        case 4800:
                            cfsetispeed(&newtio,B4800);
                            cfsetospeed(&newtio,B4800);
                            break;
        case 9600:
                            cfsetispeed(&newtio,B9600);
                            cfsetospeed(&newtio,B9600);
                            break;
        case 19200:
                            cfsetispeed(&newtio,B19200);
                            cfsetospeed(&newtio,B19200);
                            break;
        case 38400:
                            cfsetispeed(&newtio,B38400);
                            cfsetospeed(&newtio,B38400);
                            break;
        case 57600:
                            cfsetispeed(&newtio,B57600);
                            cfsetospeed(&newtio,B57600);
                            break;
        case 115200:
                            cfsetispeed(&newtio,B115200);
                            cfsetospeed(&newtio,B115200);
                            break;
        case 460800:
                            cfsetispeed(&newtio,B460800);
                            cfsetospeed(&newtio,B460800);
                            break;
        case 1000000:
                            cfsetispeed(&newtio,B1000000);
                            cfsetospeed(&newtio,B1000000);
                            break;
        case 2000000:
                            cfsetispeed(&newtio,B4000000);
                            cfsetospeed(&newtio,B4000000);
                            break;
        default		:
                            /*perror("Don't exist iBaudRate !");*/
                            printf("Don't exist iBaudRate %d !\n",iBaudRate);
                            return (-1);
    }

    /*设置数据位*/
    newtio.c_cflag &= (~CSIZE);
    switch( iDataSize )
    {
        case	7:
                        newtio.c_cflag |= CS7;
                        break;
        case	8:
                        newtio.c_cflag |= CS8;
                        break;
        default:
                        /*perror("Don't exist iDataSize !");*/
                        printf("Don't exist iDataSize %d !\n",iDataSize);
                        return (-1);
    }

    /*设置校验位*/
    switch( cParity )
    {
        case	'N':					/*无校验*/
                            newtio.c_cflag &= (~PARENB);
                            break;
        case	'O':					/*奇校验*/
                            newtio.c_cflag |= PARENB;
                            newtio.c_cflag |= PARODD;
                            newtio.c_iflag |= (INPCK | ISTRIP);
                            break;
        case	'E':					/*偶校验*/
                            newtio.c_cflag |= PARENB;
                            newtio.c_cflag &= (~PARODD);
                            newtio.c_iflag |= (INPCK | ISTRIP);
                            break;
        default:
                            /*perror("Don't exist cParity  !");*/
                            printf("Don't exist cParity %c !\n",cParity);
                            return (-1);
    }

    /*设置停止位*/
    switch( iStopBit )
    {
        case	1:
                        newtio.c_cflag &= (~CSTOPB);
                        break;
        case	2:
                        newtio.c_cflag |= CSTOPB;
                        break;
        default:
                        /*perror("Don't exist iStopBit !");*/
                        printf("Don't exist iStopBit %d !\n",iStopBit);
                        return (-1);
    }

    newtio.c_cc[VTIME] = 1;	/*设置等待时间*/
    newtio.c_cc[VMIN] = 1;	/*设置最小字符*/
    tcflush(fd,TCIFLUSH);		/*刷新输入队列(TCIOFLUSH为刷新输入输出队列)*/
    iResult = tcsetattr(fd,TCSANOW,&newtio);	/*激活新的设置使之生效,参数TCSANOW表示更改立即发生*/

    if( iResult )
    {
        perror("Set new terminal description error !");
        return (-1);
    }

    printf("set_port success !\n");

    return 0;
}

int read_port(int fd,void *buf,int iByte)
{
    int iLen = 0;
    if( !iByte )
    {
        printf("Read byte number error !\n");
        return iLen;
    }

    iLen = read(fd,buf,iByte);

    return iLen;
}

int write_port(int fd,void *buf,int iByte)
{
    int iLen = 0;
    if( !iByte )
    {
        printf("Write byte number error !\n");
        return iLen;
    }

    iLen = write(fd,buf,iByte);

    return iLen;
}


int close_port(int fd)
{
    int iResult = -1;

    iResult = close(fd);

    return iResult;
}



