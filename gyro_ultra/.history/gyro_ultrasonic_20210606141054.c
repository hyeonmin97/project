#include "ultrasonic.h"

int ultra;
unsigned long now = 0; // 현재 시간 저장용 변수
extern unsigned long past = 0;
//------------초음파 스레드-----------------
void *thread_ultrasonic_left()
{
    int busyPin = 25;
    
    int vib1 = 24;
    
    uint8_t cfg = 0, cmd = 0;
    uint8_t rxBuf[100] = {0};
    wiringPiSetup();
    ultra = wiringPiI2CSetup(ULTRA_ADDRESS);
    pinMode(busyPin, INPUT);
    pinMode(vib1, OUTPUT);

    //cfg &= ~MEASURE_RANGE_BIT; //clear bit4,long-range ranging mode
    //printf("cfg 1 : %u \n", cfg);
    ////cfg |= MEASURE_RANGE_BIT;//set bit4,short-range ranging mode
    //cfg |= MEASURE_MODE_BIT; //Set bit2，i2c passive mode
    //printf("cfg 2 : %u \n", cfg);
    ////cfg &= ~MEASURE_MODE_BIT;//clear bit2 , set to Automatic ranging mode
    //cfg &= ~TEMP_CPT_ENABLE_BIT; //clear bit1,enable temperature compensation
    //printf("cfg 3 : %u \n", cfg);
    ////cfg |= TEMP_CPT_ENABLE_BIT;//set bit1,disable temperature compensation
    //cfg &= ~TEMP_CPT_SEL_BIT; //clear bit0,select internal temperature compensation
    ////cfg |= TEMP_CPT_SEL_BIT;//set bit0,select external temperature compensation
    //printf("cfg 4 : %u\n", cfg);

    wiringPiI2CWriteReg8(ultra, eConfig, 4);
    delay(100);
    int i = 0;
    while (1)
    {
        int16_t distance1;
        cmd |= 0x01; //Set trig bit

        wiringPiI2CWriteReg8(ultra, eCmd, cmd);
        delay(100);
        //You can replace the delay with these two lines of code

        //while (isSensorBusy() == HIGH);//Wait for the sensor to start ranging
        //while(isSensorBusy()== LOW);   //Wait for sensor ran//
        //i2cReadBytes(IIC_SLAVE_ADDR, eDistanceH, rxBuf, 2); //Read distance register
        distance1 = read_raw_data_ultra(0x03);
        delay(10);

        printf("%u dist : %u cm --- \n", i++, distance1);
        // 진동시간을 다르게 할 경우(왼)
        if (distance1 >= 750)
        {
            digitalWrite(vib1, HIGH);
            delay(2000);
            digitalWrite(vib1, LOW);
            delay(2000);
        }
        else if (distance1 >= 600)
        {
            digitalWrite(vib1, HIGH);
            delay(1000);
            digitalWrite(vib1, LOW);
            delay(1000);
        }
        else if (distance1 >= 400)
        {
            digitalWrite(vib1, HIGH);
            delay(600);
            digitalWrite(vib1, LOW);
            delay(600);
        }
        else if (distance1 >= 200)
        {
            digitalWrite(vib1, HIGH);
            delay(400);
            digitalWrite(vib1, LOW);
            delay(400);
        }
        else if (distance1 >= 100)
        {
            digitalWrite(vib1, HIGH);
            delay(250);
            digitalWrite(vib1, LOW);
            delay(250);
        }
        else if (distance1 >= 80)
        {
            digitalWrite(vib1, 500);
            delay(100);
            digitalWrite(vib1, LOW);
            delay(100);
        }

        /*
        //일단 한쪽만
        // 진동시간을 다르게 할 경우(오)
        if (distance2 <= 900) {
            digitalWrite(vib2, HIGH);
            delay(2000);
            digitalWrite(vib2, LOW);
            delay(2000);
        }
        else if (distance2 <= 750) {
            digitalWrite(vib2, HIGH);
            delay(1000);
            digitalWrite(vib2, LOW);
            delay(1000);
        }
        else if (distance2 <= 600) {
            digitalWrite(vib2, HIGH);
            delay(500);
            digitalWrite(vib2, LOW);
            delay(500);
        }
        else {
            digitalWrite(vib2, HIGH);
            delay(100);
            digitalWrite(vib2, LOW);
            delay(100);
        }


        // 진동시간을 다르게 할 경우(양쪽모두)
        if (distance1 <= 900 && distance2 <= 900) {
            digitalWrite(vib1, HIGH);
            digitalWrite(vib2, HIGH);
            delay(2000);
            digitalWrite(vib1, LOW);
            digitalWrite(vib2, LOW);
            delay(2000);
        }
        else if (distance1 <= 750 && distance2 <= 750) {
            digitalWrite(vib1, HIGH);
            digitalWrite(vib2, HIGH);
            delay(1000);
            digitalWrite(vib1, LOW);
            digitalWrite(vib2, LOW);
            delay(1000);
        }
        else if (distance1 <= 750 && distance2 <= 750) {
            digitalWrite(vib1, HIGH);
            digitalWrite(vib2, HIGH);
            delay(500);
            digitalWrite(vib1, LOW);
            digitalWrite(vib2, LOW);
            delay(500);
        }
        else {
            digitalWrite(vib1, HIGH);
            digitalWrite(vib2, HIGH);
            delay(200);
            digitalWrite(vib1, LOW);
            digitalWrite(vib2, LOW);
            delay(200);
        }*/
    }
}

void* thread_ultrasonic_right()
{
    //오른쪽 초음파(아두이노키트)
    int trig = 13;
    int echo = 14;
    int vib2 = 23;
    int start_time, end_time;
    float distance;
    wiringPiSetup();

    pinMode(echo, INPUT);
    pinMode(trig, OUTPUT);
    pinMode(vib2, OUTPUT);

    while (1)
    {
        digitalWrite(trig, LOW);
        delay(500);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        while (digitalRead(echo) == 0)
            ;
        start_time = micros();
        while (digitalRead(echo) == 1)
            ;
        end_time = micros();
        distance = (end_time - start_time) / 29. / 2.;
        printf("dist_right : %.2f cm --- \n", distance);

        // 진동시간을 다르게 할 경우(오)
        if (distance >= 750)
        {
            digitalWrite(vib2, HIGH);
            delay(2000);
            digitalWrite(vib2, LOW);
            delay(2000);
        }
        else if (distance >= 600)
        {
            digitalWrite(vib2, HIGH);
            delay(1000);
            digitalWrite(vib2, LOW);
            delay(1000);
        }
        else if (distance >= 400)
        {
            digitalWrite(vib2, HIGH);
            delay(600);
            digitalWrite(vib2, LOW);
            delay(600);
        }
        else if (distance >= 200)
        {
            digitalWrite(vib2, HIGH);
            delay(400);
            digitalWrite(vib2, LOW);
            delay(400);
        }
        else if (distance >= 100)
        {
            digitalWrite(vib2, HIGH);
            delay(250);
            digitalWrite(vib2, LOW);
            delay(250);
        }
        else if (distance >= 80)
        {
            digitalWrite(vib2, 500);
            delay(100);
            digitalWrite(vib2, LOW);
            delay(100);
        }
        else if (distance >= 50)
        {
            digitalWrite(vib2, 500);
            delay(50);
            digitalWrite(vib2, LOW);
            delay(50);
        }
    }
}
int16_t read_raw_data_ultra(int addr)
{
    int8_t high_byte, low_byte;

    int16_t value;
    high_byte = wiringPiI2CReadReg8(ultra, addr);
    low_byte = wiringPiI2CReadReg8(ultra, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}

void calcDT()
{
    now = millis();
    dt = (now - past) / 1000.0;
    past = now;
}
void initMPU6050()
{
    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);                                          /* Write to sample rate register */
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);                                          /* Write to power management register */
    wiringPiI2CWriteReg8(fd, CONFIG, 0); /* Write to Configuration register */           //디지털 필터 사용안함
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 8); /* Write to Gyro Configuration register */ //fs_sel = 1 => +- 1000 도/초, 범위가 작으면 섬세하게, 범위가 넓으면 큰 각도변화
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);                                          /*Write to interrupt enable register */
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
void calcGyroYPR()
{
    const float GYROXYZ_TO_DEGREES_PER_SEC = 65;
    gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
    gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
    gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
    //자이로 센서의 값을 각속도로 매핑
}
void calcFilteredYPR()
{
    const float ALPHA = 0.96;
    float tmp_angle_x, tmp_angle_y, tmp_angle_z;
    tmp_angle_x = filtered_angle_x + gyro_x * dt; //각속도에서 각도로 변환
    tmp_angle_y = filtered_angle_y + gyro_y * dt;
    tmp_angle_z = filtered_angle_z + gyro_z * dt;
    filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
    filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
    filtered_angle_z = tmp_angle_z;
}

void connSerial()
{
    serialFd = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY | O_NDELAY);
    printf("open serial\n");
    if (serialFd == -1)
    { // ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART. Ensure it is not in use by another application\n");
    }
    struct termios options;
    tcgetattr(serialFd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //제어모드
    //options.c_iflag = IGNPAR; //입력모드, IGNPAR 패리티 오류 있는 모든 바이트 무시
    options.c_oflag = 0;     //출력모드
    options.c_lflag = 0;     //로컬모드
    options.c_cc[VTIME] = 0; //read()가 기다리고 있을 시간

    tcflush(serialFd, TCIFLUSH); //시리얼 포트 초기화

    tcsetattr(serialFd, TCSANOW, &options); //시리얼 포트에 설정 입력
}

void serialSend()
{
    char* p_tx_buffer = "fallen";
    int count = write(serialFd, (void*)p_tx_buffer, strlen(p_tx_buffer)); //Filestream, bytes to write, number of bytes to write

    if (count < 0)
    {
        printf("UART TX error\n");
    }
    printf("end send\n");
    //sleep(2);//flush 하기 전 딜레이
    //tcflush(serialFd, TCOFLUSH);
}

void serialRead()
{
    char rx_buffer[256];
    int rx_length = -1;
    while (1)
    {
        rx_length = read(serialFd, (void*)rx_buffer, sizeof(rx_buffer));
        if (rx_length > 0)
        {
            rx_buffer[rx_length] = '\0'; //문자열에 \n추가
            //tcflush(serialFd, TCIFLUSH);
            break;
        }
    }
}

void error_handling(char* message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}