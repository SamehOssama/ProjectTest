#include "uart.h"

/*
###############################################################################
	UART 0
###############################################################################
*/

void Uart0_init(void){
	SYSCTL_RCGCUART_R |= SYSCTL_RCGC1_UART0;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOA;
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0){};
	UART0_CTL_R &= ~UART_CTL_UARTEN;
	UART0_IBRD_R = 1000000 / 9600;
	UART0_FBRD_R = 1000000 % 9600 / 9600.0 * 64 + 0.5;
	UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN) & ~ UART_LCRH_PEN;
	UART0_CTL_R = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
	GPIO_PORTA_AFSEL_R |= (1 << 0) | (1 << 1);
	GPIO_PORTA_PCTL_R=(GPIO_PORTA_PCTL_R & 0XFFFFFF00) | GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX;
	GPIO_PORTA_DEN_R |= (1 << 0) | (1 << 1);
	GPIO_PORTA_AMSEL_R &= ~ ((1 << 0) | (1 << 1));
}

char Uart0_receive(void){
	char data;
	while ((UART0_FR_R & (1 << 4)) != 0);	// Wait until Rx buffer is not full
	data = UART0_DR_R;						// before giving it another byte
	return (unsigned char)data;
}

void Uart0_transmit(unsigned char data){
	while ((UART0_FR_R & (1 << 5)) != 0);	// Wait until Tx buffer not full
	UART0_DR_R = data;						// before giving it another byte
}

void Uart0_input_string(char* command, int len){
	char c;
	int i;
	for(i = 0; i < len; i++){
		c = Uart0_receive();
		if(c == '\r'){
			Uart0_transmit('\r');
			Uart0_transmit('\n');
			break;
		}
		command[i] = c;
		Uart0_transmit(c);
	}
}

void Uart0_output_string(char* pt){
	while(*pt){
		Uart0_transmit(*pt);
		pt++;
	}
}

/*
###############################################################################
	UART 1
###############################################################################
*/

void Uart1_init(void){
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2) == 0){};
	UART1_CTL_R &= ~UART_CTL_UARTEN;
	UART1_IBRD_R = 1000000 / 9600;
	UART1_FBRD_R = 1000000 % 9600 / 9600.0 * 64 + 0.5;
	UART1_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN) & ~ UART_LCRH_PEN;
	UART1_CTL_R = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
	GPIO_PORTC_AFSEL_R |= (1 << 4) | (1 << 5);
	GPIO_PORTC_PCTL_R =(GPIO_PORTC_PCTL_R & 0XFF00FFFF) | GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX;
	GPIO_PORTC_DEN_R |= (1 << 4) | (1 << 5);
	GPIO_PORTC_AMSEL_R &= ~ ((1 << 4) | (1 << 5));
}

char Uart1_receive(void){
	char data;
	while ((UART1_FR_R & (1 << 4)) != 0);	// Wait until Rx buffer is not full
	data = UART1_DR_R;						// before giving it another byte
	return (unsigned char)data;
}

void Uart1_transmit(unsigned char data){
	while ((UART1_FR_R & (1 << 5)) != 0);	// Wait until Tx buffer not full
	UART1_DR_R = data;						// before giving it another byte
}

void Uart1_input_string(char* command, int len){
	char c;
	int i;
	for(i = 0; i < len; i++){
		c = Uart1_receive();
		if(c == '\r'){
			Uart1_transmit('\r');
			Uart1_transmit('\n');
			break;
		}
		command[i] = c;
		Uart1_transmit(c);
	}
}

void Uart1_output_string(char* pt){
	while(*pt){
		Uart1_transmit(*pt);
		pt++;
	}
}

/*
###############################################################################
	UART 2
###############################################################################
*/

void Uart2_init(void){  // for gps
	unsigned BRD;
	SYSCTL_RCGCUART_R |= 0X04; // activate UART2
	SYSCTL_RCGCGPIO_R |= 0X08; //  activate port D
	while((SYSCTL_PRGPIO_R & 0X08) == 0){};
	UART2_CTL_R &= ~(0X0001);   // disable UART
	BRD = ((16000000<<2) + (9600<<1))/9600; // SET BAUD RATE DIVISOR
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
}

char Uart2_receive(void){
	char data;
	while ((UART2_FR_R & (1 << 4)) != 0);	// Wait until Rx buffer is not full
	data = UART2_DR_R;						// before giving it another byte
	return (unsigned char)data;
}

void Uart2_transmit(unsigned char data){
	while ((UART2_FR_R & (1 << 5)) != 0);	// Wait until Tx buffer not full
	UART2_DR_R = data;						// before giving it another byte
}

void Uart2_input_string(char* command, int len){
	char c;
	int i;
	for(i = 0; i < len; i++){
		c = Uart2_receive();
		if(c == '\r'){
			Uart2_transmit('\r');
			Uart2_transmit('\n');
			break;
		}
		command[i] = c;
		Uart2_transmit(c);
	}
}

void Uart2_output_string(char* pt){
	while(*pt){
		Uart2_transmit(*pt);
		pt++;
	}
}

/*
###############################################################################
	UART 5
###############################################################################
*/

void Uart5_init(void){													// PE Rx -> 4 ; Tx -> 5
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5;							// Enable UART5 clock   0010 0000
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;							// Enable port E clock
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0){};
	UART5_CTL_R &= ~UART_CTL_UARTEN;									// Disable UART5
	UART5_IBRD_R = 1000000 / 9600;
	UART5_FBRD_R =  1000000 % 9600 / 9600.0 * 64 + 0.5;
	UART5_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN) & ~UART_LCRH_PEN; // 8 bit, no parity, 1 stop, FIFOs
	UART5_CTL_R = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;		// Enable UART5, Rx, Tx
	GPIO_PORTE_AFSEL_R |= (1 << 4) | (1 << 5);							// Alternate function PE4 - PE5   0011 0000
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF) | 0x00110000;	// Set PCTL for PE4-5 to UART function (Ref P 651)
	GPIO_PORTE_DEN_R |= (1 << 4) | (1 << 5);							// Enable digital pins PE4-PE5   0011 0000
	GPIO_PORTE_AMSEL_R &= ~(1 << 4) | (1 << 5); 						// Disable analog function PE4-PE5   0011 0000
}

char Uart5_receive(void){
	char data;
	while ((UART5_FR_R & (1 << 4)) != 0);	// Wait until Rx buffer is not full
	data = UART5_DR_R;						// before giving it another byte
	return (unsigned char)data;
}

void Uart5_transmit(unsigned char data){
	while ((UART5_FR_R & (1 << 5)) != 0);	// Wait until Tx buffer not full
	UART5_DR_R = data;						// before giving it another byte
}

void Uart5_input_string(char* command, int len){
	char c;
	int i;
	for(i = 0; i < len; i++){
		c = Uart5_receive();
		if(c == '\r'){
			Uart5_transmit('\r');
			Uart5_transmit('\n');
			break;
		}
		command[i] = c;
		Uart5_transmit(c);
	}
}

void Uart5_output_string(char* pt){
	while(*pt){
		Uart5_transmit(*pt);
		pt++;
	}
}
