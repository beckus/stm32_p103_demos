#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Toggle LED */
		GPIOC->ODR = GPIOC->ODR ^ 0x00001000;

		/* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

int main(void)
{
	unsigned long var;

	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);

    /* Connect Button EXTI Line to Button GPIO Pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	/* Button is already configured as input by default - we should probably configure it just to make sure, but we don't  */
	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	*NVIC_CCR = *NVIC_CCR | 0x200;

	__asm volatile (
			"mov r0, sp \n"
		    "and r0, r0, #4 \n"
			"cmp r0, #0 \n"
		    "bne	fourbytealigned1 \n"
		    "sub sp, #4 \n"
		    "fourbytealigned1: \n"
			"mov r0, sp \n"
			:
			:
			: "r0");

	EXTI->SWIER |= EXTI_Line0; //Address should end in 0 or 8 inside interrupt

	__asm volatile (
			"mov r0, sp \n"
		    "and r0, r0, #4 \n"
			"cmp r0, #0 \n"
		    "beq	eightbytealigned1 \n"
		    "sub sp, #4 \n"
		    "eightbytealigned1: \n"
			"mov r0, sp \n"
			:
			:
			: "r0");

	EXTI->SWIER |= EXTI_Line0; //Address should end in 0 or 8 inside interrupt






	*NVIC_CCR = *NVIC_CCR & 0xfffffdff;

	__asm volatile (
			"mov r0, sp \n"
		    "and r0, r0, #4 \n"
			"cmp r0, #0 \n"
		    "bne	fourbytealigned2 \n"
		    "sub sp, #4 \n"
		    "fourbytealigned2: \n"
			"mov r0, sp \n"
			:
			:
			: "r0");

	EXTI->SWIER |= EXTI_Line0; //Address should end in 4 or c inside interrupt

	__asm volatile (
			"mov r0, sp \n"
		    "and r0, r0, #4 \n"
			"cmp r0, #0 \n"
		    "beq	eightbytealigned2 \n"
		    "sub sp, #4 \n"
		    "eightbytealigned2: \n"
			"mov r0, sp \n"
			:
			:
			: "r0");

	EXTI->SWIER |= EXTI_Line0; //Address should end in 0 or 8 inside interrupt


	while(1);
}




