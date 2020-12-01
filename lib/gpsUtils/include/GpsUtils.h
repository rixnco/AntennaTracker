//
// Created by Samuel HEIDMANN on 01/12/2020.
//

#ifndef ANTENNATRACKER_GPSUTILS_H
#define ANTENNATRACKER_GPSUTILS_H

#include <cmath>

class GpsPt {
public:
    GpsPt();
    GpsPt(double lati, double longi, double elev);
    double latitude();
    double longitude();
    double elevation();

private:
    double lati;
    double longi;
    double elev;
};

float compute_azimuth(GpsPt& from, GpsPt& to);
double compute_distance(GpsPt& from, GpsPt& to);
double compute_tilt(GpsPt& from, GpsPt& to);

#endif //ANTENNATRACKER_GPSUTILS_H
