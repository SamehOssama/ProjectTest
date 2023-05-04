#include <stdio.h>
#include <math.h>

#define RADIUS 6371000 // radius of earth in meters

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
