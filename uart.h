#ifndef _TM4C_H
#define _TM4C_H
#include "tm4c123gh6pm.h"
#endif
#ifndef _STDINT_H
#define _STDINT_H
#include <stdint.h>
#endif
#ifndef _UART_H
#define _UART_H

void uart2_init(unsigned clk,unsigned baudrate);

void uart2_send_byte(uint8_t c);

uint8_t uart2_read_byte(void);

void SYSTICKTIMER_init(void);
void SYSTICK_wait1ms(void);
void delayMillis(uint32_t delay);

#endif
