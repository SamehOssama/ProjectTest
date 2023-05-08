#include "gps.h"

void gps_read(uint8_t *GPRMC_message, char *lat,  char*lon){
    

    int comma_nums = 0;
    char data = '0';
    int idx = 0;

    char arr[10] = "$GPRMC";
    int lat_i = 0;
    int long_i = 0;
	*GPRMC_message = 0;
	
    while (1) {
        data = Uart2_receive();
        if (!*GPRMC_message){            // make sure it's a $GPRMC message
            if (arr[idx] != data){
                while (data != '\n'){
                    data = Uart2_receive();
                }
                return;
            }
            if (arr[idx] == 'C'){
                *GPRMC_message = 1;
            }
            idx++;
        }

        if (data == ','){
            comma_nums++;
        
        } else if (comma_nums == 2){
            if (data == 'V'){
                *GPRMC_message = 0;
                return;
            }
        } else if (comma_nums == 3){    // lattide
            lat[lat_i] = data;
            lat_i++;
        } else if (comma_nums == 5){    // longtiude
            lon[long_i] = data;
            long_i++;
        }

        if (data == '\n'){
            break;
        }
    }
}

double parse_degree(char * degree_str){
    float raw_degree = atof(degree_str);
    int dd = (int)(raw_degree / 100);
    double ss = raw_degree - (dd * 100);
    double degree = dd + (ss / 60);
    return degree;
}

double to_radians(double degrees){
    return degrees * M_PI / 180.0;
}

double distance(double lat1, double lon1, double lat2, double lon2){
    double dlat = to_radians(lat2 - lat1);
    double dlon = to_radians(lon2 - lon1);
    double a = pow(sin(dlat / 2), 2) + cos(to_radians(lat1)) * cos(to_radians(lat2)) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return RADIUS * c;
}
