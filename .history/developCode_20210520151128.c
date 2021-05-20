#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h> //abs
#include <math.h>
#include <time.h>

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

float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
float gyro_x, gyro_y, gyro_z;
unsigned long now = 0;  // 현재 시간 저장용 변수
unsigned long past = 0; // 이전 시간 저장용 변수
int fd;

void calcDT();
void initMPU6050();
void read_raw_data(int);
void readAccelGyro();
void calibAccelGyro();
void calcAccelYPR();
void calcGyroYPR();
void calcFilteredYPR();

void main()
{
    fd = wiringPiI2CSetup(Device_Address);
    initMPU6050();
    calibAccelGyro(); // 안정된 상태에서의 가속도 자이로 값 계산

    past = millis();

    while (1)
    {

        readAccelGyro();
        calcDT();
        calcAccelYPR();
        calcFilteredYPR();
        printf("FX : %6.2f | FY : %6.2f | FZ : %6.2f\n", filtered_angle_x, filtered_angle_y, filtered_angle_z);
    }
}

void calcDT()
{
    now = millis();
    dt = (now - past) / 1000.0;
    past = now;
}
void initMPU6050()
{
    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07); /* Write to sample rate register */
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01); /* Write to power management register */
    wiringPiI2CWriteReg8(fd, CONFIG, 0);        /* Write to Configuration register */
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);  /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01); /*Write to interrupt enable register */
}
short read_raw_data(int addr)
{
    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}

void readAccelGyro()
{
    AcX = read_raw_data(ACCEL_XOUT_H);
    AcY = read_raw_data(ACCEL_YOUT_H);
    AcZ = read_raw_data(ACCEL_ZOUT_H);

    GyX = read_raw_data(GYRO_XOUT_H);
    GyY = read_raw_data(GYRO_YOUT_H);
    GyZ = read_raw_data(GYRO_ZOUT_H);
    
}
void calibAccelGyro()
{
    float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
    float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
    readAccelGyro(0);
    for (int i = 0; i < 10; i++)
    {
        readAccelGyro(0);
        sumAcX += AcX;
        sumAcY += AcY;
        sumAcZ += AcZ;
        sumGyX += GyX;
        sumGyY += GyY;
        sumGyZ += GyZ;
        delay(100);
    }
    baseAcX = sumAcX / 10;
    baseAcY = sumAcY / 10;
    baseAcZ = sumAcZ / 10;
    baseGyX = sumGyX / 10;
    baseGyY = sumGyY / 10;
    baseGyZ = sumGyZ / 10;
    //printf("baseAcX %f\t", baseAcX);
    //printf("baseAcY %f\t", baseAcY);
    //printf("baseAcZ %f\t\n", baseAcZ);
}
void calcAccelYPR()
{
    float accel_x, accel_y, accel_z;
    float accel_xz, accel_yz;
    const float RADIANS_TO_DEGREES = 180 / 3.14159;
    accel_x = AcX - baseAcX;
    accel_y = AcY - baseAcY;
    accel_z = AcZ + (16384 - baseAcZ);

    accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
    accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;
    accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
    accel_angle_x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;
    accel_angle_z = 0;
}
void calcGyroYPR() {
    const float GYROXYZ_TO_DEGREES_PER_SEC = 131;
    gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
    gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
    gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
    gyro_angle_x += gyro_x * dt;
    gyro_angle_y += gyro_y * dt;
    gyro_angle_z += gyro_z * dt;
}
void calcFilteredYPR() {
    const float ALPHA = 0.96;
    float tmp_angle_x, tmp_angle_y, tmp_angle_z;
    tmp_angle_x = filtered_angle_x + gyro_x * dt;
    tmp_angle_y = filtered_angle_y + gyro_y * dt;
    tmp_angle_z = filtered_angle_z + gyro_z * dt;
    filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
    filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
    filtered_angle_z = tmp_angle_z;
}

