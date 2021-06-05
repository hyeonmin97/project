//https://wiki.dfrobot.com/URM13_Ultrasonic_Sensor_SKU_SEN0352#target_0
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
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
#define Device_Address 0x12



int busyPin = 25;
int fd;
int vib1 = 24;
int vib2 = 23;

int16_t read_raw_data(int addr)
{
    int8_t high_byte, low_byte;
    
    int16_t value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}
uint8_t cfg = 0, cmd = 0;
uint8_t rxBuf[100] = {0};

void main(){
    wiringPiSetup();
    fd = wiringPiI2CSetup(Device_Address);
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
   
    wiringPiI2CWriteReg8(fd, eConfig, 4);
    delay(100);





    int i = 0;
    while(1){
        int16_t distance1;
        cmd |= 0x01;                                  //Set trig bit
        
        wiringPiI2CWriteReg8(fd, eCmd, cmd);
        delay(100);
        //You can replace the delay with these two lines of code
        
        //while (isSensorBusy() == HIGH);//Wait for the sensor to start ranging
        //while(isSensorBusy()== LOW);   //Wait for sensor ran//
        //i2cReadBytes(IIC_SLAVE_ADDR, eDistanceH, rxBuf, 2); //Read distance register
		distance1 = read_raw_data(0x03);
        delay(10);
        
        printf("%u dist : %u cm --- \n",i++, distance1);
		// 진동시간을 다르게 할 경우(왼)
		if (distance1 >= 750) {
			digitalWrite(vib1, HIGH);
			delay(2000);
			digitalWrite(vib1, LOW);
			delay(2000);
		}
		else if (distance1 >= 600) {
			digitalWrite(vib1, HIGH);
			delay(1000);
			digitalWrite(vib1, LOW);
			delay(1000);
		}
		else if (distance1 >= 400) {
			digitalWrite(vib1, HIGH);
			delay(600);
			digitalWrite(vib1, LOW);
			delay(600);
		}
		else if (distance1 >= 200) {
			digitalWrite(vib1, HIGH);
			delay(400);
			digitalWrite(vib1, LOW);
			delay(400);
		}
		else if (distance1 >= 100) {
			digitalWrite(vib1, HIGH);
			delay(250);
			digitalWrite(vib1, LOW);
			delay(250);
		}
		else if (distance1 >= 80) {
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