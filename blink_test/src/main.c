/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
			

int main(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER5);
  GPIOA->MODER |= GPIO_MODER_MODER5_0;

  for(;;) {
    GPIOA->BSRRL = 1<<5;
    for(int i=0;i<1000000;) {
      i += 1;
    }
    GPIOA->BSRRH = 1<<5;
    for(int i=0;i<1000000;) {
      i += 1;
    }
  }
}
