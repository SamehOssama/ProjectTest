#ifndef _TM4C_H
#define _TM4C_H
#include "tm4c123gh6pm.h"
#endif

#ifndef _LED_H
#define _LED_H

// LEDS
#define RED_LED 0x02
#define BLUE_LED 0x04
#define GREEN_LED 0x08
#define RGB_LED 0x0E

void RGB_set(char mask);
void RGB_clear(char mask);
void RGB(char colour);
void distance_indicator(double current_distance);

#endif
