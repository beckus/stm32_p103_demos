#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"


volatile char in_char;
volatile char *out_str = "Got:_\n";
volatile int curr_char;

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		GPIOC->ODR = GPIOC->ODR ^ 0x00001000;
		if(curr_char == -1) {
			in_char = USART_ReceiveData(USART2);

			USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

			curr_char = 1;
			USART_SendData(USART2, out_str[0]);
		} else {
			USART_ReceiveData(USART2); // Discard byte if we are still writing
			                           // in response to the last received byte.
		}
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
		GPIOC->ODR = GPIOC->ODR ^ 0x00001000;

		if(curr_char == -1) {
			USART_SendData(USART2, '!'); //This should never happen
		} else if(out_str[curr_char] == '_') {
			curr_char++;
			USART_SendData(USART2, in_char);
		} else if(out_str[curr_char] == '\0') {
			curr_char = -1;
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		} else {
			USART_SendData(USART2, out_str[curr_char++]);
		}
	}
}




int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Configure USART2 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART2 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure PC.12 as output push-pull (LED)
	GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	curr_char = -1;

	USART_Cmd(USART2, ENABLE);

	while(1);
}
