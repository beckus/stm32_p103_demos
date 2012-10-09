#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

void send_byte(uint8_t b)
{
    /* Send one byte */
    USART_SendData(USART2, b);

    /* Loop until USART2 DR register is empty */
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

int main(void)
{
    uint8_t b;

    init_rs232();
    USART_Cmd(USART2, ENABLE);

    while(1) {
        /* Loop until the USART2 has received a byte. */
        while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);

        /* Capture the received byte and print it out. */
        b = (USART_ReceiveData(USART2) & 0x7F);
        send_byte('G');
        send_byte('o');
        send_byte('t');
        send_byte(':');
        send_byte(b);
        send_byte('\n');
    }
}
