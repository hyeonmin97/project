#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define PWR_MGMT_1 0x6B
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_L 0x48
int main(void)
{
    int fd;
    const int mpu6050 = 0x68;
    short GyX, GyY, GyZ;

    wiringPiSetup();

    fd = wiringPiI2CSetup(mpu6050);
    if(fd<0)
        return -1;

    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0);
    while(1){
        GyX = (wiringPiI2CReadReg8(fd, GYRO_XOUT_H) & 0xFF) << 8;
        GyX |= wiringPiI2CReadReg8(fd, GYRO_XOUT_L) & 0xFF;
        GyY = (wiringPiI2CReadReg8(fd, GYRO_YOUT_H) & 0xFF) << 8;
        GyY |= wiringPiI2CReadReg8(fd, GYRO_YOUT_L) & 0xFF;
        GyZ = (wiringPiI2CReadReg8(fd, GYRO_ZOUT_H) & 0xFF) << 8;
        GyZ |= wiringPiI2CReadReg8(fd, GYRO_ZOUT_L) & 0xFF;

        printf("Gyx : %6d | Gyy : %6d | Gyz : %6d", GyX, GyY, GyZ);
    }
}