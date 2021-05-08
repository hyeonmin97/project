#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define ACCEL_XOUT_L 0x44
#define ACCEL_YOUT_L 0x46
#define ACCEL_ZOUT_L 0x48

double angleAcX, angleAcY;
const double RADIAN_TO_DEGREE = 180 / 3.141592;

int main(void)
{
    int fd;
    const int mpu6050 = 0x68;
    short GyX, GyY, GyZ, AcX, AcY, AcZ;

    wiringPiSetup();

    fd = wiringPiI2CSetup(mpu6050);
    if (fd < 0)
        return -1;
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0); //










    while (1)
    {
        GyX = (wiringPiI2CReadReg8(fd, GYRO_XOUT_H) & 0xFF) << 8;
        GyX |= wiringPiI2CReadReg8(fd, GYRO_XOUT_L) & 0xFF;
        GyY = (wiringPiI2CReadReg8(fd, GYRO_YOUT_H) & 0xFF) << 8;
        GyY |= wiringPiI2CReadReg8(fd, GYRO_YOUT_L) & 0xFF;
        GyZ = (wiringPiI2CReadReg8(fd, GYRO_ZOUT_H) & 0xFF) << 8;
        GyZ |= wiringPiI2CReadReg8(fd, GYRO_ZOUT_L) & 0xFF;
        AcX = (wiringPiI2CReadReg8(fd, ACCEL_XOUT_H) & 0xFF) << 8;
        AcX |= wiringPiI2CReadReg8(fd, ACCEL_XOUT_L) & 0xFF;
        AcY = (wiringPiI2CReadReg8(fd, ACCEL_YOUT_H) & 0xFF) << 8;
        AcY |= wiringPiI2CReadReg8(fd, ACCEL_YOUT_L) & 0xFF;
        AcZ = (wiringPiI2CReadReg8(fd, ACCEL_ZOUT_H) & 0xFF) << 8;
        AcZ |= wiringPiI2CReadReg8(fd, ACCEL_ZOUT_L) & 0xFF;



        angleAcX = atan(AcY / sqrt(pow(AcX,2) + pow(AcZ, 2)))
    }
}

