#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"


void send_byte(uint8_t b)
{
    /* Wait until the RS232 port can receive another byte. */
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);

    /* Toggle the LED just to show that progress is being made. */
    GPIOC->ODR ^= 0x00001000;

    /* Send the byte */
    USART_SendData(USART2, b);
}



int main(void)
{
    init_led();

    init_button();

    init_rs232();
    enable_rs232_interrupts();
    enable_rs232();

    while(1) {
        send_byte('H');
        send_byte('e');
        send_byte('l');
        send_byte('l');
        send_byte('o');
        send_byte('\n');
    }
}
