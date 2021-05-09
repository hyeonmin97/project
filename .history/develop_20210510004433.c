#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
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

int fd;
int acclX, acclY, acclZ; //센서에서 읽은 값
int gyroX, gyroY, gyroZ; //센서에서 읽은 값
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;

double angleFX, angleFY, angleFZ;

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

const double RADIAN_TO_DEGREE = 180 / 3.14159; //각속도
const double DEG_PER_SEC = 32767 / 250; // 1각속도 값
const double ALPHA = 1 / (1 + 0.04);//가중치

time_t now, past;//과거시간 저장
double dt = 0; //한 사이클 시간 저장

void defSensor(){
    double sumAcX = 0, sumAcY = 0, sumAcZ = 0;                    //가속도의 초기값의 합을 저장할 변수
    double sumGyX = 0, sumGyY = 0, sumGyZ = 0; //자이로의 초기값의 합을 저장할 변수
    getData();
    for (int i = 0; i < 10;i++){
        getData();
        sumAcX += acclX, sumAcY += acclY, sumAcZ += acclZ;
        sumGyX += gyroX, sumGyY += gyroY, sumGyZ += gyroZ;
        delay(50)
    }
    averAcX = sumAcX / 10; averAcY = sumAcY / 10; averAcZ = sumAcZ / 10;
    averGyX = sumGyX / 10; averGyY = sumGyY / 10; averGyZ = sumGyZ / 10;
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
void getDt(){
    now = time(NULL);
    dt = (now - past);
    past = now;
}
int main()
{
    fd = wiringPiI2CSetup(Device_Address);
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0); //disable sleep mode
    printf("set 0x6B=%X\n", wiringPiI2CReadReg8(fd, Device_Address));
    defSensor();
    past = time(NULL);
    while (1)
    {
        getData();
        getDt();

        angleAcX = atan((double)acclY / sqrt(pow(acclX, 2) + pow(acclZ, 2)));
        angleAcX *= RADIAN_TO_DEGREE;
        angleAcY = atan(-(double)acclX / sqrt(pow(acclY, 2) + pow(acclZ, 2)));
        angleAcY *= RADIAN_TO_DEGREE;

        //(가속도 현재 - 초기평균값) => 센서값에 대한 보정
        angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt; //각속도로 변환
        angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
        angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;

        //상보필터를 위한 임시각도
        double angleTmpX = angleFiX + angleGyX * dt;
        double angleTmpY = angleFiY + angleGyY * dt;
        double angleTmpZ = angleFiZ + angleGyZ * dt;

        // (상보필터 값 처리) 임시 각도에 0.96가속도 센서로 얻어진 각도 0.04의 비중을 두어 현재 각도를 구함.
        angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
        angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
        angleFiZ = angleGyZ; // Z축은 자이로 센서만 이용
    }
    return 0;
}
