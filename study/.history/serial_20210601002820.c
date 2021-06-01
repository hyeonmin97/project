#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h> //대소문자 구분 주의
#include <wiringSerial.h> //대소문자 구분 주의

    ​

int main() ​

{
    int fd;
    int data;

        if ((fd = serialOpen("/dev/ttyAMA1", 115200)) < 0)

    {

        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));

        return 1;
    }
        printf("\nRaspberry Pi UART Test");
        while (1){

        data = serialGetchar(fd);

        printf("\nPC > RPi = %c", (char)data);

        serialPutchar(fd, data);

        serialPuts(fd, "\n");

        fflush(stdout);
    }

    ​

        return 0;
}