#include "stdint.h"
#include "M:\Programs\Keil\Keil 4\EE319Kware\inc\tm4c123gh6pm.h"

void SystemInit(){}

void digitalPinMode(char port, uint32_t pin, uint32_t mode){
    uint32_t pinD = 1<<pin, clk = 1;
    volatile uint32_t* baseAddress = 0;

    switch(port){
        case 'A': baseAddress = GPIO_PORTA_DATA_BITS_R; clk = clk << 0; break;
        case 'B': baseAddress = GPIO_PORTB_DATA_BITS_R; clk = clk << 1; break;
        case 'C': baseAddress = GPIO_PORTC_DATA_BITS_R; clk = clk << 2; break;
        case 'D': baseAddress = GPIO_PORTD_DATA_BITS_R; clk = clk << 3; break;
        case 'E': baseAddress = GPIO_PORTE_DATA_BITS_R; clk = clk << 4; break;
        case 'F': baseAddress = GPIO_PORTF_DATA_BITS_R; clk = clk << 5; break;
        default: break;
    }

    SYSCTL_RCGCGPIO_R |= clk + 0x2; // Connect Clock to Port
    while((SYSCTL_PRGPIO_R&clk) == 0){}; // Wait for Clock

    *((volatile uint32_t*)(baseAddress + (0x520/sizeof(uint32_t))))  = 0x4C4F434B; // Unlock

    *((volatile uint32_t*)(baseAddress + (0x524/sizeof(uint32_t)))) |=  pinD; // CR
    *((volatile uint32_t*)(baseAddress + (0x420/sizeof(uint32_t)))) &= ~pinD; // AFSEL
    *((volatile uint32_t*)(baseAddress + (0x52C/sizeof(uint32_t)))) &= ~((0xf)<<(pin*4)); // PCTL
    *((volatile uint32_t*)(baseAddress + (0x528/sizeof(uint32_t)))) &= ~pinD; // AMSEL
    *((volatile uint32_t*)(baseAddress + (0x51C/sizeof(uint32_t)))) |=  pinD; // DEN

    if(mode == 1){ // Output
        *((volatile uint32_t*)(baseAddress + (0x400/sizeof(uint32_t)))) |=  pinD; // DIR
        *((volatile uint32_t*)(baseAddress + (0x510/sizeof(uint32_t)))) |=  pinD; // PUR
        *((volatile uint32_t*)(baseAddress + (0x514/sizeof(uint32_t)))) &= ~pinD; // PDN
    }
    else if(mode == 0){ // Input
        *((volatile uint32_t*)(baseAddress + (0x400/sizeof(uint32_t)))) &= ~pinD; // DIR
        *((volatile uint32_t*)(baseAddress + (0x510/sizeof(uint32_t)))) &= ~pinD; // PUR
        *((volatile uint32_t*)(baseAddress + (0x514/sizeof(uint32_t)))) |=  pinD; // PDN
    }

    *((volatile uint32_t*)baseAddress) &= ~pinD; // DATA
}

void setup(){
    digitalPinMode('A', 5, 1);
    digitalPinMode('D', 2, 0);
}

void loop(){

}

int main(){
	setup();
	while(1)
	  loop();
}
