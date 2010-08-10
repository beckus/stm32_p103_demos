#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

void busyLoop(unsigned long delay )
{
  while(delay) delay--;
}

int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
					
	// Configure PC.12 as output push-pull (LED)
	GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	while(1) {
       GPIOC->BRR |= 0x00001000;
       busyLoop(500000);
       GPIOC->BSRR |= 0x00001000;
       busyLoop(500000);
	}
}
