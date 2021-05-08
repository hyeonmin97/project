#include <Wire.h>
const int MPU_ADDR = 0x68;                 // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // 가속도(Acceleration) 와
void getRawData();                         // 센서값 얻는 서브함수의 프로토타입 선언

void setup()
{
  initSensor();
  Serial.begin(9600);
  delay(200);
}

void loop()
{
  getRawData(); // 센서값 얻어오는 함수 호출
  Serial.print("AcX:");
  Serial.print(AcX);
  Serial.print("   AcY:");
  Serial.print(AcY);
  Serial.print("   AcZ:");
  Serial.print(AcZ);
  Serial.print("   GyX:");
  Serial.print(GyX);
  Serial.print("   GyY:");
  Serial.print(GyY);
  Serial.print("   GyZ:");
  Serial.print(GyZ);
  Serial.println();
  delay(300);
}

void initSensor()
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // I2C 통신용 어드레스(주소)
  Wire.write(0x6B);                 // MPU6050과 통신을 시작하기 위해서는 0x6B번지에
  Wire.write(0);                    // MPU6050을 동작 대기 모드로 변경
  Wire.endTransmission(true);
}

void getRawData()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // AcX 레지스터 위치(주소)를 지칭합니다
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // AcX 주소 이후의 14byte의 데이터를 요청

  AcX = Wire.read() << 8 | Wire.read(); //두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}