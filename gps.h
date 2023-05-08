#ifndef _TM4C_H
#define _TM4C_H
#include "tm4c123gh6pm.h"
#endif
#ifndef _STRING_H
#define _STRING_H
#include <string.h>
#endif
#ifndef _STDINT_H
#define _STDINT_H
#include <stdint.h>
#endif
#ifndef _MATH_H
#define _MATH_H
#include <math.h>
#endif
#ifndef _STDLIB_H
#define _STDLIB_H
#include <stdlib.h>
#endif
#ifndef _UART_H
#define _UART_H
#include "uart.h"
#endif

#ifndef _GPS_H
#define _GPS_H

#define M_PI 3.14159265358979323846
#define RADIUS 6371000 // radius of earth in meters

void gps_read(uint8_t *GPRMC_message, char *lat, char *lon);
double parse_degree(char *degree_str);
double to_radians(double degrees);
double distance(double lat1, double lon1, double lat2, double lon2);

#endif
