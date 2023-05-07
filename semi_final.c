#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>



#define M_PI 3.14159265358979323846
#define RADIUS 6371000 // radius of earth in meters


double dist_long,dist_latt; // dist_point
double latt,longt;   // current point
uint8_t GPRMC_message = 0;
char lat[30] = "";
char lon[30] = "";


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


void timer_1ms(void) 
{
	NVIC_ST_CTRL_R=0X0;    
	NVIC_ST_CURRENT_R=0;
	NVIC_ST_RELOAD_R= 16000-1;
	NVIC_ST_CTRL_R=0X5;
	while((NVIC_ST_CTRL_R & 0x00010000) ==0);
}

void delay_ms(uint32_t delay)
{ 
	  int i;
    for(i=0;i< delay;i++)
	  {
		timer_1ms();
    }
}


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
		
		delay_ms(1);
		 
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

void gps_read()
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


int main(void)
{
  double tot_dis = 0;
	double last_dis = -1;
	
	// set dist_point
	// we will need big delay here (maybe 1 minute or some thing)

	uart2_init(16000000,9600);
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
		
		// use distance to control leds
		
		
		
		if(last_dis!=-1)
		{
			tot_dis += abs(dis-last_dis);
		}
		last_dis = dis;
		GPRMC_message = 0;
	}
	
	
	
	
	
	
	
	return 0;
}

