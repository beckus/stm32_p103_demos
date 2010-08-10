#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

void SysTick_Handler(void)
{
	GPIOC->ODR ^= 0x00001000;
}

int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int last_button_state, new_button_state;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
					

	// Configure PC.12 as output push-pull (LED)
	GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Setup SysTick Timer for 1 second with AHB/8 */
	/*
	 * AHB:
	 * 72000000 ticks / sec

	 * AHB/8:
	 * 9000000 ticks / sec
	 *
	 * the reload register can hold 16777216 (i.e. this is the max number of ticks)
	 */

	if (SysTick_Config(SystemCoreClock / 8))
	{
		/* Capture error */
		while (1);
	}

	last_button_state = GPIOA->IDR & 0x00000001;

	while(1) {
		new_button_state = GPIOA->IDR & 0x00000001;
		if(new_button_state ^ last_button_state) {
			if(new_button_state) {
				SysTick->CTRL ^= SysTick_CTRL_CLKSOURCE_Msk;
			}
		}
		last_button_state = new_button_state;
	}
}
