#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>

/* SPI Commands */
#define ADS124X_USER_DEFINE   0x64
#define ADS124X_SPI_INIT      (0x00 + ADS124X_USER_DEFINE)
#define ADS124X_SPI_RADC      (0x01 + ADS124X_USER_DEFINE)
#define ADS124X_SPI_PT2T      (0x02 + ADS124X_USER_DEFINE)
#define ADS124X_SPI_TEST      (0x03 + ADS124X_USER_DEFINE)


#define ADS124X_SPI_WAKEUP    0x00
#define ADS124X_SPI_SLEEP     0x02
#define ADS124X_SPI_SYNC1     0x04
#define ADS124X_SPI_SYNC2     0x04
#define ADS124X_SPI_RESET     0x06
#define ADS124X_SPI_NOP       0xff
#define ADS124X_SPI_RDATA     0x12
#define ADS124X_SPI_RDATAC    0x14
#define ADS124X_SPI_SDATAC    0x16
#define ADS124X_SPI_RREG      0x20
#define ADS124X_SPI_WREG      0x40
#define ADS124X_SPI_SYSOCAL   0x60
#define ADS124X_SPI_SYSGCAL   0x61
#define ADS124X_SPI_SELFOCAL  0x62

#define ADS124X_SINGLE_REG    0x00
#define ADS124X_DOUBLE_REG    0x01



int main()
{
    int fd = open("/dev/ads1248",O_RDWR);
    if(fd < 0)
    printf("open failed\n");


    while(1)
    {
        //ioctl(fd,ADS124X_SPI_PT2T,ADS124X_SPI_PT2T);
        sleep(2);
    }
}
