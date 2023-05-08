#include "stdint.h"
#include "tm4c123gh6pm.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define UART0 ((volatile uint32_t *)0x4000C000)
#define UART1 ((volatile uint32_t *)0x4000D000)
#define UART2 ((volatile uint32_t *)0x4000E000)
#define UART3 ((volatile uint32_t *)0x4000F000)
#define UART4 ((volatile uint32_t *)0x40010000)
#define UART5 ((volatile uint32_t *)0x40011000)
#define UART6 ((volatile uint32_t *)0x40012000)
#define UART7 ((volatile uint32_t *)0x40013000)

void SystemInit(){}

#define SW2   0x01
#define RED   0x02
#define BLUE  0x04
#define GREEN 0x08
#define RGB   0x0E
#define SW1   0x10

void SYSTICK_TIMER_Init(void){
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 0x0;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05;
}

void wait1us (void){
    NVIC_ST_RELOAD_R = 16-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void wait10us (void){
    NVIC_ST_RELOAD_R = 160-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void wait100us (void){
    NVIC_ST_RELOAD_R = 1600-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void wait1ms (void){
    NVIC_ST_RELOAD_R = 16000-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void wait10ms (void){
    NVIC_ST_RELOAD_R = 160000-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void wait100ms (void){
    NVIC_ST_RELOAD_R = 1600000-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void wait1s (void){
    NVIC_ST_RELOAD_R = 16000000-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void waitMax(void){
    NVIC_ST_RELOAD_R = 0x00ffffff;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void delayMillis(uint32_t delay){
    NVIC_ST_RELOAD_R = (delay*16000)-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void delayMicros(uint32_t delay){
    NVIC_ST_RELOAD_R = (delay*16)-1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & 0X10000)==0);
}

void delays(uint32_t delay){
    while(delay-->0)
        wait1s();
}

void digitalPinMode(char port, uint32_t pin, uint32_t mode){
    uint32_t pinD = 1 << pin, clk = 1;
    volatile uint32_t* baseAddress;

    switch(port){
        case 'A': baseAddress = GPIO_PORTA_DATA_BITS_R; clk = clk << 0; if(pin>7) return; break;
        case 'B': baseAddress = GPIO_PORTB_DATA_BITS_R; clk = clk << 1; if(pin>7) return; break;
        case 'C': baseAddress = GPIO_PORTC_DATA_BITS_R; clk = clk << 2; if(pin>7) return; break;
        case 'D': baseAddress = GPIO_PORTD_DATA_BITS_R; clk = clk << 3; if(pin>7) return; break;
        case 'E': baseAddress = GPIO_PORTE_DATA_BITS_R; clk = clk << 4; if(pin>5) return; break;
        case 'F': baseAddress = GPIO_PORTF_DATA_BITS_R; clk = clk << 5; if(pin>4) return; break;
        default: return;
    }

    SYSCTL_RCGCGPIO_R |= clk; // Connect Clock to Port
    while((SYSCTL_PRGPIO_R & clk) == 0); // Wait for Clock

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

    *((volatile uint32_t*)baseAddress) &= 1; // DATA
}

void digitalPortMode(char port, uint32_t mode){
    int i = (port == 'F')? 5 : (port == 'E')? 6 : 8, j = 0;
    while(j<i){
        digitalPinMode(port, j++, mode%2);
        mode = mode >> 1;
    }
}

void digitalPinWrite(char port, uint32_t pin, uint32_t out){
    uint32_t pinD = 1 << pin, pinOffset;
    volatile uint32_t* baseAddress;

    switch(port){
        case 'A': baseAddress = GPIO_PORTA_DATA_BITS_R; if(pin>7) return; break;
        case 'B': baseAddress = GPIO_PORTB_DATA_BITS_R; if(pin>7) return; break;
        case 'C': baseAddress = GPIO_PORTC_DATA_BITS_R; if(pin>7) return; break;
        case 'D': baseAddress = GPIO_PORTD_DATA_BITS_R; if(pin>7) return; break;
        case 'E': baseAddress = GPIO_PORTE_DATA_BITS_R; if(pin>5) return; break;
        case 'F': baseAddress = GPIO_PORTF_DATA_BITS_R; if(pin>4) return; break;
        default: return;
    }

    if(!((*((volatile uint32_t*)(baseAddress + (0x400/sizeof(uint32_t))))) | pinD))
        return;

    switch (pin) {
        case 0:
            pinOffset = 0x04;
            break;
        case 1:
            pinOffset = 0x08;
            break;
        case 2:
            pinOffset = 0x10;
            break;
        case 3:
            pinOffset = 0x20;
            break;
        case 4:
            pinOffset = 0x40;
            break;
        case 5:
            pinOffset = 0x80;
            break;
        case 6:
            pinOffset = 0x100;
            break;
        case 7:
            pinOffset = 0x200;
            break;
        default:
            return;
    }

    if(out == 0)
        *(((volatile uint32_t*)baseAddress + (pinOffset/sizeof(uint32_t)))) &= ~pinD;
    else if(out == 1)
        *(((volatile uint32_t*)baseAddress + (pinOffset/sizeof(uint32_t)))) |=  pinD;
    else
        return;
}

void digitalPortWrite(char port, uint32_t data){
    int i = (port == 'F')? 5 : (port == 'E')? 6 : 8, j = 0;
    while(j<i){
        digitalPinWrite(port, j++, data%2);
        data = data >> 1;
    }
}

#define LCD_Write 0
#define LCD_Read 1
#define LCD_Command 0
#define LCD_Data 1

void LCD_MOSI8bitEn(){
    digitalPinMode('D', 0, 1);
    digitalPinMode('D', 1, 1);
    digitalPinMode('D', 2, 1);
    digitalPinMode('D', 3, 1);
    digitalPinMode('E', 0, 1);
    digitalPinMode('E', 1, 1);
    digitalPinMode('E', 2, 1);
    digitalPinMode('E', 3, 1);

    /*
        A7 A6 A5    E3 E2 E1 E0 D3 D2 D1 D0
        || || ||    || || || || || || || ||
        RS RW EN    D7 D6 D5 D4 D3 D2 D1 D0
    */

    digitalPinMode('A', 5, 1);
    digitalPinMode('A', 6, 1);
    digitalPinMode('A', 7, 1);

}

void LCD_MOSI8bit(uint32_t mosi){

    /*
        E3 -> A0 -> D7
        E2 -> A1 -> D6
        E1 -> A2 -> D5
        E0 -> A3 -> D4
        D3 -> A4 -> D3
        D2 -> A5 -> D2
        D1 -> A6 -> D1
        D0 -> A7 -> D0

        E3 E2 E1 E0 D3 D2 D1 D0
        || || || || || || || ||
        D7 D6 D5 D4 D3 D2 D1 D0
    */

    //digitalPortWrite('D', mosi & 0x0f);
    //digitalPortWrite('E', (mosi >> 4) & 0x0f);

    *(GPIO_PORTD_DATA_BITS_R + (0x03C/sizeof(uint32_t))) = (mosi & 0x0f);
    *(GPIO_PORTE_DATA_BITS_R + (0x03C/sizeof(uint32_t))) = ((mosi >> 4) & 0x0f);
}

void LCD_MOSI8bitCommand(uint32_t command){

    digitalPinWrite('A', 5, 0);
    LCD_MOSI8bit(command);
    digitalPinWrite('A', 7, LCD_Command);
    digitalPinWrite('A', 6, LCD_Write);

    digitalPinWrite('A', 5, 1);
    delayMillis(1);
    digitalPinWrite('A', 5, 0);
	
    LCD_MOSI8bit(0);
}

void LCD_MOSI8bitChar(uint32_t data){

    digitalPinWrite('A', 5, 0);
    LCD_MOSI8bit(data);
    digitalPinWrite('A', 7, LCD_Data);
    digitalPinWrite('A', 6, LCD_Write);

    digitalPinWrite('A', 5, 1);
    delayMillis(1);
    digitalPinWrite('A', 5, 0);

    LCD_MOSI8bit(0);

    LCD_MOSI8bitCommand(0x06);
}

void LCD_MOSI8bitStr(unsigned char* pt){
    while(*pt){
        LCD_MOSI8bitChar(*pt);
        pt++;
    }
}

void LCD_8bit_Init(){
    LCD_MOSI8bitEn();
    LCD_MOSI8bitCommand(0x30);
    LCD_MOSI8bitCommand(0x0f);
    LCD_MOSI8bitCommand(0x01);
    LCD_MOSI8bitCommand(0x02);
    LCD_MOSI8bitCommand(0x38);
}

void portFInit(){

    SYSCTL_RCGCGPIO_R |= 0x20;
    while((SYSCTL_PRGPIO_R & 0x20) == 0) {}

    GPIO_PORTF_AFSEL_R &= ~0x1f;
    GPIO_PORTF_PCTL_R &= ~0x1f;
    GPIO_PORTF_AMSEL_R &= ~0x1f;
    GPIO_PORTF_DIR_R |= 0xE;
    GPIO_PORTF_DEN_R |= 0x1f;
    GPIO_PORTF_PUR_R &= ~0xE;
    GPIO_PORTF_DATA_R &= ~0x1f;

}

void LED_ON(uint8_t LED){
    GPIO_PORTF_DATA_R |= LED;
}

void LED_OFF(uint8_t LED){
    GPIO_PORTF_DATA_R &= ~LED;
}

uint8_t switch1(){
    return ((GPIO_PORTF_DATA_R & SW1) >> 4);
}

uint8_t switch2(){
    return (GPIO_PORTF_DATA_R & SW2);
}

void RGB_set(uint8_t mask){
    mask &= 0X0E;
    GPIO_PORTF_DATA_R = mask;
}

void RGB_clear(uint8_t mask){
    mask &= 0X0E;
    GPIO_PORTF_DATA_R &= ~mask;
}

void LED_RGB(uint8_t colour){
    GPIO_PORTF_DATA_R &= ~RGB;			// Stop all active LEDs
    GPIO_PORTF_DATA_R |= colour;			// Turn on this led
}

void UART_Init(char port, uint32_t UART, char mode, uint32_t baudRate){
    uint32_t PCTL_Val = ((port == 'C' && UART == 1))? 0x02 : 0x01;
    uint32_t clkPort = 0x01, clkUART = 0x01, pin, pinD;
    volatile uint32_t* baseAddress; volatile uint32_t* portBaseAddress;

    switch(UART){
        case 0: baseAddress = UART0; clkUART = clkUART << 0; break;
        case 1: baseAddress = UART1; clkUART = clkUART << 1; break;
        case 2: baseAddress = UART2; clkUART = clkUART << 2; break;
        case 3: baseAddress = UART3; clkUART = clkUART << 3; break;
        case 4: baseAddress = UART4; clkUART = clkUART << 4; break;
        case 5: baseAddress = UART5; clkUART = clkUART << 5; break;
        case 6: baseAddress = UART6; clkUART = clkUART << 6; break;
        case 7: baseAddress = UART7; clkUART = clkUART << 7; break;
        default: return;
    }

    switch(port){
        case 'A':
            portBaseAddress = GPIO_PORTA_DATA_BITS_R; clkPort = clkPort << 0;
            pin = (mode == 'R')? 0x00 : 0x01; break;

        case 'B':
            portBaseAddress = GPIO_PORTB_DATA_BITS_R; clkPort = clkPort << 1;
            pin = (mode == 'R')? 0x00 : 0x01; break;

        case 'C':
            portBaseAddress = GPIO_PORTC_DATA_BITS_R; clkPort = clkPort << 2;
            if((UART == 1) || (UART == 4))
                pin = (mode == 'R')? 0x04 : 0x05;
            else if(UART == 3)
                pin = (mode == 'R')? 0x06 : 0x07;
            else
                return;
            break;

        case 'D':
            portBaseAddress = GPIO_PORTD_DATA_BITS_R; clkPort = clkPort << 3;
            switch(UART) {
                case 2:
                    pin = (mode == 'R') ? 0x06 : 0x07;
                    break;

                case 6:
                    pin = (mode == 'R') ? 0x04 : 0x05;
                    break;

                default:
                    return;
                }
            break;

        case 'E':
            portBaseAddress = GPIO_PORTE_DATA_BITS_R; clkPort = clkPort << 4;
            switch(UART) {
                case 5:
                    pin = (mode == 'R') ? 0x04 : 0x05;
                    break;

                case 7:
                    pin = (mode == 'R') ? 0x00 : 0x01;
                    break;

                default:
                    return;
            }
            break;

        default:
            return;
    }

    pinD = 0x01 << pin;

    SYSCTL_RCGCUART_R |= clkUART;
    SYSCTL_RCGCGPIO_R |= clkPort;
    while((SYSCTL_PRGPIO_R & clkPort) == 0);

    *((volatile uint32_t*)(baseAddress + (0x030/sizeof(uint32_t)))) = ~UART_CTL_UARTEN;

    *((volatile uint32_t*)(baseAddress + (0x024/sizeof(uint32_t)))) = 1000000 / baudRate;
    *((volatile uint32_t*)(baseAddress + (0x028/sizeof(uint32_t)))) =  (uint32_t)((((1000000 % 9600) / 9600.0) * 64) + 0.5);

    *((volatile uint32_t*)(baseAddress + (0x02C/sizeof(uint32_t)))) = (UART_LCRH_WLEN_8 | UART_LCRH_FEN) & ~UART_LCRH_PEN;
    *((volatile uint32_t*)(baseAddress + (0x030/sizeof(uint32_t)))) = (mode == 'R')? UART_CTL_UARTEN | UART_CTL_RXE : UART_CTL_UARTEN | UART_CTL_TXE;

    *((volatile uint32_t*)(portBaseAddress + (0x420/sizeof(uint32_t)))) |= pinD;
    *((volatile uint32_t*)(portBaseAddress + (0x52C/sizeof(uint32_t)))) |= ((PCTL_Val)<<(pin*4));
    *((volatile uint32_t*)(portBaseAddress + (0x528/sizeof(uint32_t)))) &= ~pinD;
    *((volatile uint32_t*)(portBaseAddress + (0x51C/sizeof(uint32_t)))) |=  pinD;
}

unsigned char UART_receive(uint32_t UART){
    volatile uint32_t* baseAddress; unsigned char data = 'E';

    switch(UART){
        case 0: baseAddress = UART0; break;
        case 1: baseAddress = UART1; break;
        case 2: baseAddress = UART2; break;
        case 3: baseAddress = UART3; break;
        case 4: baseAddress = UART4; break;
        case 5: baseAddress = UART5; break;
        case 6: baseAddress = UART6; break;
        case 7: baseAddress = UART7; break;
        default: return data;
    }
    while (((*((volatile uint32_t*)(baseAddress + (0x018/sizeof(uint32_t))))) & (1 << 4)) != 0x00);
    data = *((volatile uint32_t*)baseAddress);
    return data;
}

void UART_transmit(uint32_t UART, unsigned char data){
    volatile uint32_t* baseAddress;
    switch(UART){
        case 0: baseAddress = UART0; break;
        case 1: baseAddress = UART1; break;
        case 2: baseAddress = UART2; break;
        case 3: baseAddress = UART3; break;
        case 4: baseAddress = UART4; break;
        case 5: baseAddress = UART5; break;
        case 6: baseAddress = UART6; break;
        case 7: baseAddress = UART7; break;
        default: return;
    }
    while (((*((volatile uint32_t*)(baseAddress + (0x018/sizeof(uint32_t))))) & (1 << 5)) != 0);
    *((volatile uint32_t*)baseAddress) = data;
}

void UART_serialInput(uint32_t UART, unsigned char* command, int len){
    
    unsigned char c;
    int i;
    for(i = 0; i < len; i++){
        c = UART_receive(UART);
        if(c == '\r' || c == '\n'){
            UART_transmit(UART, '\r');
            UART_transmit(UART, '\n');
            break;
        }
        command[i] = c;
        UART_transmit(UART, c);
    }
}

void UART_serialOutput(uint32_t UART, unsigned char* pt){
		while(*pt){
        UART_transmit(UART, *pt);
        pt++;
    }
}

#define M_PI 3.14159265358979323846
#define RADIUS 6371000 // radius of earth in meters

double dist_long,dist_latt; // dist_point
double latt, longt;   // current point
uint8_t GPRMC_message = 0;
unsigned char lat[30] = "";
unsigned char lon[30] = "";

void gps_read(void)
{
    int comma_nums = 0;
    unsigned char data;
    int idx = 0;

    char arr[10] = "$GPRMC";
    int lat_i = 0;
    int long_i = 0;


    while(1)
    {
        data = UART_receive(2);
        if(!GPRMC_message)  	// make sure it's a $GPRMC message
        {
            if(arr[idx]!= data)
            {

                while(data!='\n')
                    data = UART_receive(2);

                return;
            }
            if(arr[idx]=='C')
                GPRMC_message = 1;

            idx++;
        }

        if(data == ',')
            comma_nums++;
        else if(comma_nums==2)
        {
            if(data == 'V')
            {
                GPRMC_message = 0;
                return;
            }
        }
        else if(comma_nums==3) // latitude
        {
            lat[lat_i] = data;
            lat_i++;
        }
        else if(comma_nums==5) // longitude
        {
            lon[long_i] = data;
            long_i++;
        }

        if(data == '\n')
            break;

    }

}

double parse_degree(unsigned char* degree_str)
{
    double raw_degree = atof(degree_str);
    int dd = (int) (raw_degree / 100);
    double ss = raw_degree - (dd * 100.0);
    double degree = dd + (ss / 60);
    return degree;
}

double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = to_radians(lat2 - lat1);
    double dLon = to_radians(lon2 - lon1);
    double a = pow(sin(dLat / 2), 2) + cos(to_radians(lat1)) * cos(to_radians(lat2)) * pow(sin(dLon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return RADIUS * c;
}

void distance_indicator(int current_distance){
    if(current_distance == 0){
        LED_RGB(GREEN);
    } else if(current_distance <= 5){
        LED_RGB(GREEN | RED);
    } else {
        LED_RGB(RED);
    }
}

double tot_dis = 0;
double last_dis = -1;

void setup(){
    SystemInit();
    portFInit();
    SYSTICK_TIMER_Init();

    dist_long = 31.2804801;
    dist_latt = 30.0648808;

    UART_Init('D', 2, 'R',9600);
    UART_Init('D', 2, 'T',9600);
    
    LED_ON(RED);
    delays(3);
    LED_RGB(0x00);
    gps_read();
    delays(60);
    LED_RGB(RGB);
    delays(3);
    LED_RGB(0x00);
}

void loop(){
    double dis;
    while(!GPRMC_message)
    {
        gps_read();
    }
    latt = parse_degree(lat);
    longt = parse_degree(lon);

    dis = distance(latt,longt,dist_latt,dist_long);
    if(last_dis !=-1)
        tot_dis += abs(dis-last_dis);

    distance_indicator(dis);

    last_dis = dis;

    GPRMC_message = 0;
}

int main(){

	setup();
	
	while(1){
		loop();
	}
	
}