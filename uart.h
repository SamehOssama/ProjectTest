#ifndef _TM4C_H
#define _TM4C_H
#include "tm4c123gh6pm.h"
#endif

#ifndef _UART_H
#define _UART_H

void Uart0_init(void);
char Uart0_receive(void);
void Uart0_transmit(unsigned char data);
void Uart0_input_string(char *command, int len);
void Uart0_output_string(char *pt);

void Uart1_init(void);
char Uart1_receive(void);
void Uart1_transmit(unsigned char data);
void Uart1_input_string(char *command, int len);
void Uart1_output_string(char *pt);

void Uart2_init(void);
char Uart2_receive(void);
void Uart2_transmit(unsigned char data);
void Uart2_input_string(char *command, int len);
void Uart2_output_string(char *pt);

void Uart5_init(void);
char Uart5_receive(void);
void Uart5_transmit(unsigned char data);
void Uart5_input_string(char *command, int len);
void Uart5_output_string(char *pt);

#endif
