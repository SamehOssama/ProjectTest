void uart2_init(unsigned clk,unsigned baudrate)
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
	  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0X00FFFFFF) | 0X11000000; // configure uart for pD6,pD7
	  
	  
	  UART2_LCRH_R = 0X0070; // 8-bit word length, endable Fifo
	  UART2_CTL_R = 0X0301;  // enable RXE,TXE AND UART
	
	  GPIO_PORTD_DEN_R |= 0XC0;  // enable digital IO on PD6,PD7
	  GPIO_PORTD_AMSEL_R &= ~0XC0; // disable analog function on PD6, PD7
		 
}
char uart2_inchar(void)
{
	while((UART2_FR_R & 0X10) != 0);
	return (char)(UART2_DR_R & 0XFF);
}

void uart2_outchar(char data)
{
	while((UART2_FR_R & 0X0020) != 0);
        UART2_DR_R = data;
}
void uart2_outstring(char *pt)
{
	while(*pt){
		uart2_outchar(*pt);
		pt++;
	}
}

void uart2_instring(char *command,int len)
{
	char character;
	int i = 0;
	for(;i<len;i++)
	{
		character = uart2_inchar();
		if(character!='\r')
		{
			command[i] = character;
		  uart2_outchar(command[i]);
		}
		else
			break;
	}
	
}
