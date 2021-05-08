#include <stdio.h>

#include "wiringPi.h"



#define LED1 24 



int main(void)

{

  wiringPiSetup();

  pinMode (LED1, OUTPUT) ;



  for (;;)

  {

    digitalWrite (LED1, HIGH) ; 

  }

  return 0 ;

}


