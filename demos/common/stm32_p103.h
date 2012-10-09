#ifndef __STM32_P103_H
#define __STM32_P103_H

/* This library contains routines for interfacing with the STM32 P103 board. */

/* Initialize the LED (the board only has one). */
void init_led(void);

/* Initialize the button (the board only has one). */
void init_button(void);

/* Configures the RS232 serial port using the following settings:
 *   9600 Baud
 *   8 bits + 1 stop bit
 *   No parity bit
 *   No hardware flow control
 * Note that the USART2 is not enabled in this routine.  It is left disabled in
 * case any additional configuration is needed.
 */
void init_rs232(void);

void enable_rs232_interrupts(void);

void enable_rs232(void);

#endif /* __STM32_P103_H */
