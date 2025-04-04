#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi.h"

static const char *spiDev0 = "/dev/spidev0.0" ;
static const char *spiDev1 = "/dev/spidev0.1" ;
static uint8_t spiBPW = 8 ;
static uint16_t spiDelay = 0 ;
static uint32_t spiSpeeds [2] ;
static int spiFds [2] ;

/*

* SPIDataRW:

* Write and Read a block of data over the SPI bus.

* Note the data ia being read into the transmit buffer, so will

* overwrite it!

* This is also a full-duplex operation.

*********************************************************************************

*********************************************************************************/

int SPIDataRW (int channel, uint8_t *tx_data, uint8_t *rx_data, int len)

{

int i = 0;
struct spi_ioc_transfer spi ;

channel &= 1 ;

memset (&spi, 0, sizeof (spi)) ;

spi.tx_buf = (unsigned long)tx_data ;
spi.rx_buf = (unsigned long)rx_data ;
spi.len = len ;
spi.delay_usecs = spiDelay ;
spi.speed_hz = spiSpeeds [channel] ;
spi.bits_per_word = spiBPW ;

return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ; //SPI_IOC_MESSAGE(1)的1表示spi_ioc_transfer的数量
}



int SPISetupMode (int channel, int speed, int mode)
{
int fd ;

if ((fd = open (channel == 0 ? spiDev0 : spiDev1, O_RDWR)) < 0)
{
printf("Unable to open SPI device: %s\n", strerror (errno)) ;
return -1;
}

spiSpeeds [channel] = speed ;
spiFds [channel] = fd ;

if (ioctl (fd, SPI_IOC_WR_MODE, &mode) < 0)

{
printf("Can't set spi mode: %s\n", strerror (errno)) ;
return -1;
}

if (ioctl (fd, SPI_IOC_RD_MODE, &mode) < 0)
{
printf("Can't get spi mode: %s\n", strerror (errno)) ;
return -1;
}


if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0)
{
printf("Can't set bits per word: %s\n", strerror (errno)) ;
return -1;
}

if (ioctl (fd, SPI_IOC_RD_BITS_PER_WORD, &spiBPW) < 0)
{
printf("Can't get bits per word: %s\n", strerror (errno)) ;
return -1;
}


if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
{
printf("Can't set max speed hz: %s\n", strerror (errno));
return -1;
}


if (ioctl (fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
{
printf("Can't get max speed hz: %s\n", strerror (errno));
return -1;
}

return fd ;
}

int SPISetup (int channel, int speed)
{
return SPISetupMode (channel, speed, SPI_MODE_0) ;
}
