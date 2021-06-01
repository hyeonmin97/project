#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main()
{
    int fd, data;
    if((fd=serialOpen("/dev/ttyAMA1", 115200))<0){
        fprintf(stderr, "unable to open serial device :%s\n", strerror(errno));
        return 1;
    }
    printf("\nRaspberry UART test");
    while(1){
        data = serialGetchar(fd);
        printf("\nPc > RPi = %c", (char)data);
        serialPutchar(fd, data);
        serialPuts(fd, "\n");
        fflush(stdout);
    }
    return 0;
}