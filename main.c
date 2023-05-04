#include <string.h>

#define SYSCTL_RCGCGPIO_R		(*((volatile unsigned long *)0x400FE608))
#define SYSCTL_PRGPIO_R			(*((volatile unsigned long *)0x400FEA08))
#define SYSCTL_RCGCUART_R		(*((volatile unsigned long *)0x400FE618))

// PORT F
#define GPIO_PORTF_DATA_R		(*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R		(*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R		(*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R		(*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R		(*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R		(*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R			(*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R		(*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R		(*((volatile unsigned long *)0x4002552C))

// PORT A
#define GPIO_PORTA_DATA_R		(*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R		(*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R		(*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R		(*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R		(*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R		(*((volatile unsigned long *)0x4000452C))

// UART0
#define UART0_DR_R				(*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R				(*((volatile unsigned long *)0x4000C018))
#define UART0_CTL_R				(*((volatile unsigned long *)0x4000C030))
#define UART0_IBRD_R			(*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R			(*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R			(*((volatile unsigned long *)0x4000C02C))
#define UART0_CC_R				(*((volatile unsigned long *)0x4000CFC8))

// LEDS
#define RED_LED 			0x02
#define GREEN_LED 			0x04
#define BLUE_LED			0x08

void PortF_init(void);
void Uart0_init(void);
char Uart0_Receiver(void);
void Uart0_Transmitter(unsigned char data);
void get_line(char *command, int len);
void output_string(char *pt);
void RGB(char colour);

const int len = 128;
char command[len] = {0};

int main(void){
	PortF_init();
	Uart0_init();
	
	while(1){
		get_line(command, len);
		// TODO: Get GPS data
		memset(command, 0, len);
	}
}

void PortF_init(void){
	SYSCTL_RCGCGPIO_R |= 0x00000020;		// Enable port F clock
	while((SYSCTL_PRGPIO_R & 0x00000020) == 0){};
	GPIO_PORTF_LOCK_R = 0x4C4F434B; 		// Unlock PortF PF0
	GPIO_PORTF_CR_R = 0x0E;				// Allow changes to PF1-3
	GPIO_PORTF_DIR_R = 0x0E;			// PF3,PF2,PF1 output
	GPIO_PORTF_AFSEL_R = 0x00;			// No alternate function
	GPIO_PORTF_PCTL_R = 0x00000000;			// GPIO clear bit PCTL
	GPIO_PORTF_DEN_R = 0x0E;			// Enable digital pins PF1-PF3
	GPIO_PORTF_AMSEL_R = 0x00;			// Disable analog function
}

void Uart0_init(void){
	SYSCTL_RCGCUART_R |= 0x0001;			// Activate UART0
	SYSCTL_RCGCGPIO_R |= 0x00000001;		// Enable port A clock
	while((SYSCTL_PRGPIO_R & 0x01) == 0){};
	UART0_CTL_R &= ~0x0001;				// Disable UART0
	UART0_IBRD_R = 1000000 / 9600;
	UART0_FBRD_R = (1000000 % 9600) / 9600.0 * 64 + 0.5;
	UART0_LCRH_R = 0x0070;				// 8 bit, no parity, 1 stop, FIFOs
	UART0_CTL_R |= 0x0301;				// enable UART0, Rx, Tx
	GPIO_PORTA_AFSEL_R |= 0x03;			// alternate function PA0 - PA1
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) | 0x00000011; // Set PCTL for PA0-1 to UART function
	GPIO_PORTA_DEN_R |= 0x03;			// Enable digital pins PF4-PF0
	GPIO_PORTA_AMSEL_R &= ~0x03;			// Disable analog function
}

char Uart0_Receiver(void){
	char data;
	while ((UART0_FR_R & (1 << 4)) != 0);		// wait until Rx buffer is not full
	data = UART0_DR_R;				// before giving it another byte
	return (unsigned char)data;
}

void Uart0_Transmitter(unsigned char data){
	while ((UART0_FR_R & (1 << 5)) != 0);		// wait until Tx buffer not full
	UART0_DR_R = data;				// before giving it another byte
}

void get_line(char* command, int len){
	char c;
	int i;
	for(i = 0; i < len; i++){				
		c = Uart0_Receiver();
		if(c == '\r'){				// Get the input till the line feed char.
			Uart0_Transmitter('\r');
			Uart0_Transmitter('\n');
			break;
		}
		command[i] = c;
		Uart0_Transmitter(c);
	}
}

void output_string(char* pt){
	while(*pt){					// Loop till the null char.
		Uart0_Transmitter(*pt);			// Print string to serial monitor.
		pt++;
	}
}

void RGB(char colour){
	GPIO_PORTF_DATA_R &= ~0x11;			// Stop all active LEDs
	GPIO_PORTF_DATA_R |= colour;			// Turn on this led
}
