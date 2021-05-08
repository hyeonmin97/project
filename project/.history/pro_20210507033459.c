#include <Arduino.h>
#include <stdio.h>

#define LED1 24

int main(void)

{


    pinMode(LED1, OUTPUT);

    for (;;)

    {

        digitalWrite(LED1, HIGH);
    }

    return 0;
}
