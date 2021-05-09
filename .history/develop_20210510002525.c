#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <math.h>

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

int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double acclX_scaled, acclY_scaled, acclZ_scaled;
double gyroX_scaled, gyroY_scaled, gyroZ_scaled;
double angleFX, angleFY, angleFZ;//상보필터 임시 값
double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

const double RADIAN_TO_DEGREE = 180 / 3.14159; //각속도
const double DEG_PER_SEC = 32767 / 250; // 1각속도 값
const double ALPHA = 1 / (1 + 0.04);//가중치

unsigned long now = 0;//현재 시간 저장
unsigned long past = 0;//과거시간 저장
double dt = 0; //한 사이클 시간 저장

void defSensor(){
    double sumAcX = 0, sumAcY = 0, sumAcZ = 0; //가속도의 초기값의 합을 저장할 변수
    double sumGyX = 0, sumGyY = 0, sumGyZ = 0; //자이로의 초기값의 합을 저장할 변수
    getData();
    for (int i = 0; i < 10;i++){
        getData();
        sumAcX += acclX, sumAcY += acclY, sumAcZ += acclZ;
        sumGyX += gyroX, sumGyY += gyroY, sumGyZ += gyroZ;
        delay(50)
    }
}

int read_word_2c(int addr)
{
    int val;
    val = wiringPiI2CReadReg8(fd, addr);
    val = val << 8;
    val += wiringPiI2CReadReg8(fd, addr + 1);
    if (val >= 0x8000)
        val = -(65536 - val);

    return val;
}

double dist(double a, double b)
{
    return sqrt((a * a) + (b * b));
}

double get_y_rotation(double x, double y, double z)
{
    double radians;
    radians = atan2(x, dist(y, z));
    return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
    double radians;
    radians = atan2(y, dist(x, z));
    return (radians * (180.0 / M_PI));
}
void getData(){
    acclX = read_word_2c(ACCEL_XOUT_H);
    acclY = read_word_2c(ACCEL_YOUT_H);
    acclZ = read_word_2c(ACCEL_XOUT_H);
    gyroX = read_word_2c(GYRO_XOUT_H);
    gyroY = read_word_2c(GYRO_XOUT_H);
    gyroZ = read_word_2c(GYRO_XOUT_H);
}
int main()
{
    fd = wiringPiI2CSetup(Device_Address);
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0); //disable sleep mode
    printf("set 0x6B=%X\n", wiringPiI2CReadReg8(fd, Device_Address));

    while (1)
    {

        getData();

        acclX_scaled = acclX / 16384.0;
        acclY_scaled = acclY / 16384.0;
        acclZ_scaled = acclZ / 16384.0;

        printf("My acclX_scaled: %f\n", acclX_scaled);
        printf("My acclY_scaled: %f\n", acclY_scaled);
        printf("My acclZ_scaled: %f\n", acclZ_scaled);

        printf("My X rotation: %f\n", get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled));
        printf("My Y rotation: %f\n", get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled));

        delay(100);
    }
    return 0;
}
