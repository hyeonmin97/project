
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h> //abs
#include <math.h>
#include <time.h>
const int MPU_ADDR = 0x68;                 // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // 가속도(Acceleration)와 자이로(Gyro)
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;

const double RADIAN_TO_DEGREE = 180 / 3.14159;
const double DEG_PER_SEC = 32767 / 250; // 1초에 회전하는 각도
const double ALPHA = 1 / (1 + 0.04);
// GyX, GyY, GyZ 값의 범위 : -32768 ~ +32767 (16비트 정수범위)

unsigned long now = 0;  // 현재 시간 저장용 변수
unsigned long past = 0; // 이전 시간 저장용 변수
double dt = 0;          // 한 사이클 동안 걸린 시간 변수

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

int fd;
short read_raw_data(int addr)
{
    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}

void initSensor()
{
    fd = wiringPiI2CSetup(MPU_ADDR);
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0); //disable sleep mode
}

void getData()
{
    acclX = read_raw_data(ACCEL_XOUT_H);
    acclY = read_raw_data(ACCEL_YOUT_H);
    acclZ = read_raw_data(ACCEL_XOUT_H);
    gyroX = read_raw_data(GYRO_XOUT_H);
    gyroY = read_raw_data(GYRO_XOUT_H);
    gyroZ = read_raw_data(GYRO_XOUT_H);
}

// loop 한 사이클동안 걸리는 시간을 알기위한 함수
void getDT()
{
    now = millis();
    dt = (now - past) / 1000.0;
    past = now;
}

// 센서의 초기값을 10회 정도 평균값으로 구하여 저장하는 함수
void caliSensor()
{
    double sumAcX = 0, sumAcY = 0, sumAcZ = 0;
    double sumGyX = 0, sumGyY = 0, sumGyZ = 0;
    getData();
    for (int i = 0; i < 10; i++)
    {
        getData();
        sumAcX += AcX;
        sumAcY += AcY;
        sumAcZ += AcZ;
        sumGyX += GyX;
        sumGyY += GyY;
        sumGyZ += GyZ;
        delay(50);
    }
    averAcX = sumAcX / 10;
    averAcY = sumAcY / 10;
    averAcZ = sumAcY / 10;
    averGyX = sumGyX / 10;
    averGyY = sumGyY / 10;
    averGyZ = sumGyZ / 10;
}
void main()
{
    initSensor();
    caliSensor();    //  초기 센서 캘리브레이션 함수 호출
    past = millis(); // past에 현재 시간 저장
    while(1){
        getData();
        getDT();

        angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
        angleAcX *= RADIAN_TO_DEGREE;
        angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
        angleAcY *= RADIAN_TO_DEGREE;
        // 가속도 센서로는 Z축 회전각 계산 불가함.

        // 가속도 현재 값에서 초기평균값을 빼서 센서값에 대한 보정
        angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt; //각속도로 변환
        angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
        angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;

        // 상보필터 처리를 위한 임시각도 저장
        double angleTmpX = angleFiX + angleGyX * dt;
        double angleTmpY = angleFiY + angleGyY * dt;
        double angleTmpZ = angleFiZ + angleGyZ * dt;

        // (상보필터 값 처리) 임시 각도에 0.96가속도 센서로 얻어진 각도 0.04의 비중을 두어 현재 각도를 구함.
        angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
        angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
        angleFiZ = angleGyZ; // Z축은 자이로 센서만을 이용하열 구함.
        printf("FilteredX: %6.2f  |  FilteredY: %6.2f  |  FilteredZ: %6.2f\n", angleFX, angleFY, angleFZ);
    }
}
