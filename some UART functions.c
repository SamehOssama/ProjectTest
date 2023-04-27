		
//values need to be fixed to satisfy UART1

   
   void UART1_Init(void){
			SYSCTL_RCGCUART_R |=OX01;
			SYSCTL_RCGCGPIO_R |=OX01;
			UART0_CTL_R &=~0X01;
			UART0_IBRD_R=520;		//baudrate 9600
			UART0_FBRD_R=53;
			UART0_LCRH_R=0X70;
			UART0_CTL_R=0X301;
			GPIO_PORTA_AFSEL_R |=0X03;
			GPIO_PORTA_PCTL_R=(GPIO_PORTA_PCTL_R&0XFFFFFF00) |0X11;
			GPIO_PORTA_DEN_R |=0X03;
			GPIO_PORTA_AMSEL_R &= ~0X03;
		}
		
		
		bool UART1_Available(void){
			return (UART0_FR_R & 0X10 !=0) ? 0:1;
		}


		char UART1_Read(void){
			while(UART0_FR_R & OX10 !=0);
			return (char)(UART0_DR_R & 0XFF);
		}


		void UART1_Write(char data){
			while(UART0_FR_R & OX20 !=0);
			UART0_DR_R=data;
		}
	
			
			
