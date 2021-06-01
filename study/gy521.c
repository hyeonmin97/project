#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "/usr/include/mariadb/mysql.h"

#define DBHOST "127.0.0.1"
#define DBUSER "admin"
#define DBPASS "gusals97"
#define DBNAME "sensor"

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

#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_L 0x48

double angleAcX, angleAcY;
const double RADIAN_TO_DEGREE = 180 / 3.141592;

int main(void)
{
    MYSQL *mysql1;
    mysql1 = mysql_init(NULL);
    int fd;
    const int mpu6050 = 0x68;
    short GyX, GyY, GyZ, AcX, AcY, AcZ;
    char query[1024];
    char delete[1024];
    int num = 0;
    wiringPiSetup();

    fd = wiringPiI2CSetup(mpu6050);
    if (fd < 0)
        return -1;
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0); //

    if (!mysql_real_connect(mysql1, DBHOST, DBUSER, DBPASS, DBNAME, 3306, NULL, 0))
    {
        printf("connect error");
        return 0;
    }

    sprintf(delete, "truncate test"); //테이블의 데이터 삭제 구문
    printf("mysql open\n");
    mysql_query(mysql1, delete);
    while (1)
    {
        if (num > 100)
        {
            mysql_query(mysql1, delete); //데이터 삭제 실행
            num = 0;
        }
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

        angleAcX = atan((double)AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
        angleAcX *= RADIAN_TO_DEGREE;
        angleAcY = atan(-(double)AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
        angleAcY *= RADIAN_TO_DEGREE;

        printf("num %5d\t angle x : %7.2f\t angle Y : %7.2f\n", num, angleAcX, angleAcY);
        sprintf(query, "insert into test(num, angleAcX, angleAcY) values(%d, %.2f, %.2f)", num, angleAcX, angleAcY);
        if (mysql_query(mysql1, query))
        {
            fprintf(stderr, "%s\n", mysql_error(mysql1));
            printf("write dberror\n");
        }
        num++;
    }
}
