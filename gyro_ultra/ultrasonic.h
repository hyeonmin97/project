#pragma once
#include <stdio.h>
/*

���ν����忡�� ���̷� ������
������ �����忡�� ������ ������ �ص���

*/
//자이로센서
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdlib.h> //abs
#include <math.h>
#include <time.h>
//소켓
//#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

//Serial
#include <termios.h>
#include <fcntl.h>

//-----------스레드----------
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>
//-----------

//-----------초음파, 진동----------
typedef enum
{
    eAddr = 0,
    ePid,
    eVid,
    eDistanceH,
    eDistanceL,
    eInternalTempretureH,
    eInternalTempretureL,
    eExternalTempretureH,
    eExternalTempretureL,
    eConfig,
    eCmd,
    eNoise,
    eSensitivity,
    eRegNum
} regindexTypedef;

#define MEASURE_RANGE_BIT ((uint8_t)0x01 << 4)
#define MEASURE_MODE_BIT ((uint8_t)0x01 << 2)
#define TEMP_CPT_ENABLE_BIT ((uint8_t)0x01 << 1)
#define TEMP_CPT_SEL_BIT ((uint8_t)0x01 << 0)

#define IIC_SLAVE_ADDR ((uint8_t)0x12)
#define isSensorBusy() (digitalRead(busyPin))
#define ULTRA_ADDRESS 0x12



//자이로센서
#define Device_Address 0x68
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

//소켓
#define PORT 9000
#define BUF_SIZE 1024

//-----------자이로--------------
float AcX,
AcY, AcZ, Tmp, GyX, GyY, GyZ;
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
float gyro_x, gyro_y, gyro_z;

int fd;

int16_t read_raw_data_ultra(int addr);

void *thread_ultrasonic_left();
void *thread_ultrasonic_right();
void calcDT();
void initMPU6050();
short read_raw_data(int);
void readAccelGyro();
void calibAccelGyro();
void calcAccelYPR();
void calcGyroYPR();
void calcFilteredYPR();
//-----------

////소켓
//int sock;
//char recv_buffer[BUF_SIZE];
//void error_handling(char*);
//void connSocket();
//void toLatte();

//시리얼
int serialFd;
void connSerial();
void serialSend();
void serialRead();