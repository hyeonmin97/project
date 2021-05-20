#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h> //abs
#include <math.h>
#include <time.h>
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
float gyro_x, gyro_y, gyro_z - unsigned long t_now;
unsigned long t_prev;
void initDT()
{
    tprev = millis0;
}
void calcDT()
{
    tnow = millisQ;
    dt = (tnow - tprev) / 1000.0;
    prev = tnow;
}
void initMPU60500
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}
void readAccelGyro()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestfrom(MPU_addr, 14, true);
    AcX @Wire.read() << 8 | Wire.read;
    AcY = Wire.read() << 8 | Wire.read0;
    AcZ = Wire.read0 << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
GyX = Wire read) << 8 | Wire.read();
GyY = Wire.read() << 8 | Wire.read();
GyZ = Wire.read() << 8 | Wire read0;
}
void calibAccelGyro()
{
    float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
    float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
    readAccelGyro(0);
    for (int i = 0; 1 < 10; i++)
    {
        readAccelGyro();
        SUMACK + ACK;
        sumACY += ACY;
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
}
void calcAccelYPR()
{
    float accel_x, accel_y, accel_z;
    float accelxz, accel_yz;
    const float RADIANS_TO_DEGREES = 180 / 3.14159;
    accel_x = AcX - baseAcX;
    accel_z = AcY - baseAcY;
    accel_z ™ AcZ + (16384 - baseAcZ);
    accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
    accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;
    accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
    accel_angle_x = atan(accel_y / accel_xz) *RADIANS_TO_DEGREES.accel_angle_z = 0;
}
vaid calcGyroYPRO (
const float GYROXYZ_TO_DEGREES_PER_SEC = 131;
gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
gyro_angle_x += gyro_x * dt;
gyro_angle_y += gyro_y * dt;
gyro_angle_z += gyro_z * dt.
}
void calcFilteredYPRO (
const float ALPHA = 0.96;
float tmp_angle_x, tmp_angle_y, tmp_angle_z;
tmp_angle_x = fittered_angle_x + gyro_x * dt;
tmp_angie_y = filtered_angle_y + gyro_y « dt;
tmp_angle_z = fittered_angle_z + gyro_z * dt
filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) « accel_angle_y;
fittered_angle_z = tmp_angle_z;
}
void setup0
{
    initMPU60500; // mpu-6050 242] SAS 27S epricy.
    Serial.begin(115200);
    calibAccelGyro(); // 24 SEHD] 2HBS, AOL] she ARHEUCt.
    initDT0;
}
void loopQ
{
readAccelGyroQ); // AHS 2p0/2 UE AMSwE] HOSycy
cakDT();
calcAccelYPRO; // THE MAB O1BEtO] Roll, Pitch, Yaw 25H AAret
Uc
}
cakGyroYPRQ;
= // ALL AA 01S SO} Roll, Pitch, Yaw Ge Mie
    uct.cakFilteredYPRO;
ff &A ZO 4 BE ASSO
} Roll, Pitch, Yaw
4a FRC.
Serial print("FX ; *);
Serial. print(filtered_angle_x);
Serial. print(* .
Serial.print(*FY ; *);
Serial. print(filtered_angle_y).
Serial. print(* ’\
Serial.print(*FZ ; *);
Serial.printin(filtered_angle_z);
}
