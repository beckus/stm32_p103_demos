#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

static void setup_hardware( void );

volatile xQueueHandle serial_str_queue = NULL;
volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
volatile xQueueHandle serial_rx_queue = NULL;

typedef struct
{
	char str[100];
} serial_str_msg;

typedef struct
{
	char ch;
} serial_ch_msg;



void USART2_IRQHandler(void)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	serial_ch_msg rx_msg;

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
		xSemaphoreGiveFromISR(serial_tx_wait_sem, &xHigherPriorityTaskWoken);
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

		if(xHigherPriorityTaskWoken) {
			taskYIELD();
		}
	} else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		rx_msg.ch = USART_ReceiveData(USART2);
		if(!xQueueSendToBackFromISR(serial_rx_queue, &rx_msg, &xHigherPriorityTaskWoken)) {
			while(1);
		}

		if(xHigherPriorityTaskWoken) {
			taskYIELD();
		}
	} else {
		while(1);
	}

}

void send_byte(char ch)
{
	while(!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));
	USART_SendData(USART2, ch);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

char receive_byte(void)
{
	serial_ch_msg msg;
	while(!xQueueReceive(serial_rx_queue, &msg, portMAX_DELAY));

	return msg.ch;
}

void led_flash_task( void *pvParameters )
{
	while(1) {
		GPIOC->ODR = GPIOC->ODR ^ 0x00001000;
		vTaskDelay(100);  // Delay one second
	}
}

void serial_xmit_str_task( void *pvParameters )
{
	serial_str_msg msg;
	int curr_char;

	while(1) {
		while(!xQueueReceive(serial_str_queue, &msg, portMAX_DELAY));
		curr_char = 0;
		while(msg.str[curr_char] != '\0') {
			send_byte(msg.str[curr_char]);
			curr_char++;
		}
	}
}

void serial_write_task1( void *pvParameters )
{
	serial_str_msg msg;

	strcpy(msg.str, "Hello 1\n");

	while(1) {
		while(!xQueueSendToBack(serial_str_queue, &msg, portMAX_DELAY));
		vTaskDelay(200); // Delay two seconds
	}
}

void serial_write_task2( void *pvParameters )
{
	serial_str_msg msg;

	strcpy(msg.str, "Hello 2\n");

	while(1) {
		while(!xQueueSendToBack(serial_str_queue, &msg, portMAX_DELAY));
		vTaskDelay(50); // Delay half a second
	}
}

void serial_readwrite_task( void *pvParameters )
{
	serial_str_msg msg;
	char ch;
	int curr_char;
	int done;

	strcpy(msg.str, "Got:");

	while(1) {
		curr_char = 4;
		done = 0;
		do {
			ch = receive_byte();
			if((ch == '\r') || (ch == '\n')) {
				msg.str[curr_char] = '\n';
				msg.str[curr_char+1] = '\0';
				done = -1;
			} else {
				msg.str[curr_char++] = ch;
			}
		} while(!done);
		while(!xQueueSendToBack(serial_str_queue, &msg, portMAX_DELAY));
	}
}

int main(void)
{
	setup_hardware();

	/* Create the queue used by the serial task.  Messages for write to the RS232. */
	serial_str_queue = xQueueCreate( 10, sizeof( serial_str_msg ) );
	vSemaphoreCreateBinary(serial_tx_wait_sem);
	serial_rx_queue = xQueueCreate( 1, sizeof( serial_ch_msg ) );

	xTaskCreate( led_flash_task, ( signed portCHAR * ) "LED Flash", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 5, NULL );
	xTaskCreate( serial_xmit_str_task, ( signed portCHAR * ) "Serial Xmit Str", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 2, NULL );
	xTaskCreate( serial_write_task1, ( signed portCHAR * ) "Serial Write 1", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 10, NULL );
	xTaskCreate( serial_write_task2, ( signed portCHAR * ) "Serial Write 2", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 10, NULL );
	xTaskCreate( serial_readwrite_task, ( signed portCHAR * ) "Serial Read/Write", 512 /* stack size */, NULL, tskIDLE_PRIORITY + 10, NULL );

    // Start the real time kernel with preemption.
    vTaskStartScheduler();

	return 0;
}

void vApplicationTickHook( void )
{
}

static void setup_hardware( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// Configure PC.12 as output push-pull (LED)
	GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure USART2 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART2 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

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

	USART_Cmd(USART2, ENABLE);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Button is already configured as input by default - we should probably
	 * configure it just to make sure, but we don't
	 * Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}
