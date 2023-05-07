#include <string.h>
#include "led.h"
#include "uart.h"

void portF_init(void);

const int len = 16;
char command[len] = {0};

int main(void){
	portF_init();
	Uart0_init();
	Uart1_init();
	Uart5_init();
	Uart5_output_string("Hello World\n");
	
	while(1){
		Uart0_input_string(command, len);
		if(!strcmp(command, "red")){
			RGB(RED_LED);
		} else if(!strcmp(command, "green")){
			RGB(GREEN_LED);
		} else if(!strcmp(command, "blue")){
			RGB(BLUE_LED);
		} else {
			RGB(0);
		}
		memset(command, 0, len);
	}
}

void portF_init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;// Enable port F clock
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;		// Unlock PortF PF0
	GPIO_PORTF_CR_R = 0x0E;					// Allow changes to PF1-3
	GPIO_PORTF_DIR_R = 0x0E;				// PF3,PF2,PF1 output
	GPIO_PORTF_AFSEL_R = 0x00;				// No alternate function
	GPIO_PORTF_PCTL_R = 0x00000000;			// GPIO clear bit PCTL
	GPIO_PORTF_DEN_R = 0x0E;				// Enable digital pins PF1-PF3
	GPIO_PORTF_AMSEL_R = 0x00;				// Disable analog function
}
