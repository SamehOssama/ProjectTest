#include "timer.h"

void SYSTICKTIMER_init(void){
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 0XFFFFFF;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0X05;
}

void SYSTICK_wait1ms(void){
    NVIC_ST_RELOAD_R = 16000;
    NVIC_ST_CURRENT_R = 0;
    while ((NVIC_ST_CTRL_R & 0x10000) == 0) {}; // wait for the flag
}

void delayMillis(uint32_t delay){
    unsigned long i;
    for (i = 0; i < delay; i++){
        SYSTICK_wait1ms();
    }
}
