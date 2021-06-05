/*
추가해야할 내용 : 
*/

//자이로센서
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
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

//스레드
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>

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
int busyPin = 25;
int ultra;
int vib1 = 24;
int vib2 = 23;

int16_t read_raw_data_ultra(int addr)
{
    int8_t high_byte, low_byte;

    int16_t value;
    high_byte = wiringPiI2CReadReg8(ultra, addr);
    low_byte = wiringPiI2CReadReg8(ultra, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}
uint8_t cfg = 0, cmd = 0;
uint8_t rxBuf[100] = {0};

void *thread_ultrasonic()
{
    wiringPiSetup();
    ultra = wiringPiI2CSetup(ULTRA_ADDRESS);
    pinMode(busyPin, INPUT);
    pinMode(vib1, OUTPUT);
    pinMode(vib2, OUTPUT);

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



//자이로
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
short read_raw_data(int);
void readAccelGyro();
void calibAccelGyro();
void calcAccelYPR();
void calcGyroYPR();
void calcFilteredYPR();

//
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




void main()
{
    pthread_t thread;
    pthread_create(&thread, NULL, thread_ultrasonic, NULL);
    connSerial(); //시리얼 연결하고 시작
    int timeToFall = 0;

    fd = wiringPiI2CSetup(Device_Address);
    initMPU6050();
    calibAccelGyro(); // 안정된 상태에서의 가속도 자이로 값 계산

    past = millis();

    while (1)
    {
        //센서값 읽기
        readAccelGyro();
        calcDT();
        calcAccelYPR();
        calcGyroYPR();
        calcFilteredYPR();
        printf("FX : %6.2f | FY : %6.2f | FZ : %6.2f\n", filtered_angle_x, filtered_angle_y, filtered_angle_z);

        //낙상 감지
        if (abs((int)filtered_angle_x) > 70) //x축이 70도보다 크면
        {
            printf("fall detection\n");
            timeToFall++; //넘어짐 지속시간 계산용 변수 증가
            if (timeToFall > 50)
            { //넘어짐이 일정시간 이상 지속되면, 5초에 600정도 증가
                printf("send to latte\n");
                serialSend(); //시리얼 통신으로 라떼에 넘어졌다고 알림
                serialRead(); //시리얼 통신으로 라떼에서 문자열 받음
                //tcflush(serialFd, TCIOFLUSH);
                timeToFall = 0;
            }
        }
        else
        {
            timeToFall = 0;
        }
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
/*
void connSocket(){
    char send_socket[BUF_SIZE] = {
        0,
    };
    
    
    struct sockaddr_in serv_adr;
	printf("in connSocket()\n");
    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (-1 == sock)
        error_handling("socket() error");
    memset(&serv_adr, 0, sizeof(serv_adr));
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = inet_addr("192.168.151.49");
    serv_adr.sin_port = htons(9000);
    if (-1 == connect(sock, (struct sockaddr *)&serv_adr, sizeof(serv_adr)))
    {
        printf("접속 실패\n");
        exit(1);
    }
    printf("socket connect\n");
}
void toLatte(){
	connSocket();//라떼-라즈베리 연결부터 하고 시작
    printf("in toLatte()\n");
    int n_recv;
    char message[BUF_SIZE] = "fallen";
    write(sock, message, sizeof(message) - 1); // 낙상감지후 라떼로 보냄
    
    while(1){
        // 서버단에서 메세지 받음
        n_recv = read(sock, recv_buffer, BUF_SIZE);
        
        if(n_recv>0){
            printf("%s\n", recv_buffer);
            //폰에서 넘어졌다고 확인버튼 누르면 라떼를 통해 값이 넘어오고 그때종료할 조건
            if (!strncmp(recv_buffer, "btn",3))
            {
                break;
            }
        }
        
    }
}
*/

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
    char *p_tx_buffer = "fallen";
    int count = write(serialFd, (void *)p_tx_buffer, strlen(p_tx_buffer)); //Filestream, bytes to write, number of bytes to write

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
        rx_length = read(serialFd, (void *)rx_buffer, sizeof(rx_buffer));
        if (rx_length > 0)
        {
            rx_buffer[rx_length] = '\0'; //문자열에 \n추가
            //tcflush(serialFd, TCIFLUSH);
            break;
        }
    }
}

void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}
