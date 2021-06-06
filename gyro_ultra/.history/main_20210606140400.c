#include "ultrasonic.h"
unsigned long past = 0; // 이전 시간 저장용 변수
void main()
{
    pthread_t thread1, thread2;
    pthread_create(&thread1, NULL, thread_ultrasonic_left, NULL);
    pthread_create(&thread2, NULL, thread_ultrasonic_right, NULL);

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


