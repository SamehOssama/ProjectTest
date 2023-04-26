void timer_1ms(void) 
{
	NVIC_ST_CTRL_R=0X0;    
	NVIC_ST_CURRENT_R=0;
	NVIC_ST_RELOAD_R= 80000-1;
	NVIC_ST_CTRL_R=0X5;
	WHILE((NVIC_ST_CTRL_R & 0x00010000) ==0);

}

void timer_multiples_1ms(uint32_t delay) //this function creates "delay" ms
{ 
    for(int i=0;i< delay;i++)
	{
		timer_1ms();
    }
}