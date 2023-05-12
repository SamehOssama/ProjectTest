#include <string.h>
#include "led.h"
#include "uart.h"
#include <math.h>
#include <stdlib.h>


#define M_PI 3.14159265358979323846
#define RADIUS 6371000 // radius of earth in meters


double dist_long,dist_latt; // dist_point
double latt,longt;   // current point
uint8_t GPRMC_message = 0;
char lat[30] = "";
char lon[30] = "";


void portF_init(void);
void gps_read(void);
double parse_degree(char *degree_str);
double to_radians(double degrees);
double distance(double lat1, double lon1, double lat2, double lon2);

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

void uart5_send_byte(uint8_t c)
{
	while((UART5_FR_R & 0x20) != 0);  
	UART5_DR_R = c; 					
}

void Uart5_output_string(char* pt){
	while(*pt){
		uart5_send_byte(*pt);
		pt++;
	}
}


int main(void){
	
	
	double tot_dis = 0;
	double last_latt = -1;
	double last_lon = -1;
  char charray[30];
	dist_long = 31.279127;
	dist_latt = 30.064034;
	SYSTICKTIMER_init();
	portF_init();
	Uart5_init();
  uart2_init(16000000,9600);
	

	RGB_set(0x2);
	delayMillis(3000);
	RGB(0X00);
	delayMillis(1000*15);
	RGB(0x0E);
	delayMillis(3000);
	RGB(0X00);
	
	
	
	while(1)
	{
		double dis;
		while(!GPRMC_message)
		{
			gps_read();
		}
		latt = parse_degree(lat);
		longt = parse_degree(lon);
		
		dis = distance(latt,longt,dist_latt,dist_long);
		if(last_latt !=-1)
		  tot_dis += distance(latt,longt,last_latt,last_lon);
		
		distance_indicator(dis);
		
		last_latt = latt;
		last_lon = longt;
	 
    sprintf(charray, "%2.6f", tot_dis);
	  Uart5_output_string(charray);
		Uart5_output_string("\n");
		GPRMC_message = 0;	
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


void gps_read(void)
{
	int comma_nums = 0;
	char data;
	int idx = 0;

	char arr[10] = "$GPRMC";  
	int lat_i = 0;
	int long_i = 0;
	

	while(1)
	{
		data = uart2_read_byte();
		if(!GPRMC_message)  	// make sure it's a $GPRMC message
		{
			if(arr[idx]!= data)
			{
				
				 while(data!='\n')
					data = uart2_read_byte();
				
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
		else if(comma_nums==3) // lattide
		{
			lat[lat_i] = data;
			lat_i++;
		}
		else if(comma_nums==5) // longtiude
		{
			lon[long_i] = data;
			long_i++;
		}
		
		if(data == '\n')
			break;
		
  }
	
}

double parse_degree(char *degree_str)
{
    float raw_degree = atof(degree_str);
	  int dd = (int) (raw_degree / 100);
    double ss = raw_degree - (dd * 100);
    double degree = dd + (ss / 60);
    return degree;
}
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = to_radians(lat2 - lat1);
    double dlon = to_radians(lon2 - lon1);
    double a = pow(sin(dlat / 2), 2) + cos(to_radians(lat1)) * cos(to_radians(lat2)) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return RADIUS * c;
}

