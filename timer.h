#ifndef _TM4C_H
#define _TM4C_H
#include "tm4c123gh6pm.h"
#endif
#ifndef _STDINT_H
#define _STDINT_H
#include <stdint.h>
#endif

#ifndef _TIMER_H
#define _TIMER_H

void SYSTICKTIMER_init(void);
void SYSTICK_wait1ms(void);
void delayMillis(uint32_t delay);

#endif