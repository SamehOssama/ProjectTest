#include "uart.h"


void uart2_init(unsigned clk,unsigned baudrate)  // for gps
{
	  unsigned BRD;
	
	  SYSCTL_RCGCUART_R |= 0X04; // activate UART2
	  SYSCTL_RCGCGPIO_R |= 0X08; //  activate port D
	
	  UART2_CTL_R &= ~(0X0001);   // disable UART
	  BRD = ((clk<<2) + (baudrate<<1))/baudrate; // SET BAUD RATE DIVISOR
	  UART2_IBRD_R = BRD >> 6;
	  UART2_FBRD_R = BRD&63;
	
	  GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;  // Unlock port D
    GPIO_PORTD_CR_R |= 0xC0;  // Allow changes to PD7-PD6
	  GPIO_PORTD_AFSEL_R |= 0XC0; // enable alt function PD7, PD6
	  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0X00FFFFFF) | 0X11000000; // configure uart for pa0,pa1
	  
	  UART2_CC_R = 0; 	   // use system clock
	  UART2_LCRH_R = 0x60; // 8-bit word length, no Fifo , no parity, 1 stop bit
	  UART2_CTL_R = 0X0301;  // enable RXE,TXE AND UART
	
	  GPIO_PORTD_DEN_R |= 0XC0;  // enable digital IO on PD6,PD7
	  GPIO_PORTD_AMSEL_R &= ~0XC0; // disable analog function on PD6, PD7
		
		delayMillis(1);
		 
}

void uart2_send_byte(uint8_t c)
{
	while((UART2_FR_R & 0x20) != 0);  
	UART2_DR_R = c; 					
}

uint8_t uart2_read_byte(void)
{
	uint8_t c;
	
	while((UART2_FR_R & 0x10) != 0);  
	c = UART2_DR_R; 					
	
	return c; 
}

void SYSTICKTIMER_init(void)
{
NVIC_ST_CTRL_R = 0;
NVIC_ST_RELOAD_R = 0XFFFFFF;
NVIC_ST_CURRENT_R = 0;
NVIC_ST_CTRL_R = 0X05 ;
}
void SYSTICK_wait1ms(void)
{
NVIC_ST_RELOAD_R = 16000;
NVIC_ST_CURRENT_R = 0;
while((NVIC_ST_CTRL_R&0x10000)==0){}; // wait for the flag
}
void delayMillis(uint32_t delay)
{
unsigned long i;
for(i = 0 ; i<delay ; i++)
{
SYSTICK_wait1ms();
}
}
