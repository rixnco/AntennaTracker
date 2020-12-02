//
// Created by Samuel HEIDMANN on 01/12/2020.
//
#include "GeoUtils.h"

#include <cmath>

#define EARTH_RADIUS_KM 6371

#define TO_RADF(x) (((float)(M_PI))*((float)(x))/180.f)
#define TO_DEGF(x) (180.f*((float)(x))/((float)(M_PI)))


GeoPt::GeoPt() : _lat(0.f), _lon(0.f), _elev(0.f) {
}

GeoPt::GeoPt(const GeoPt& other) : _lat(other._lat), _lon(other._lon), _elev(other._elev) {
}

GeoPt::GeoPt(float lat, float lon, float elev) : _lat(lat), _lon(lon), _elev(elev) {
}

GeoPt& GeoPt::operator=(const GeoPt& other) {
    _lat = other._lat;
    _lon = other._lon;
    _elev= other._elev;
    return *this;
}

float GeoPt::getLatitude() const {
    return _lat;
}
void GeoPt::setLatitude(float lat) {
    _lat= lat;
}

float GeoPt::getLongitude() const {
    return _lon;
}
void GeoPt::setLongitude(float lon) {
    _lon= lon;
}

float GeoPt::getElevation() const {
    return _elev;
}
void GeoPt::setElevation(float elev) {
    _elev= elev;
}


float GeoPt::azimuthTo(const GeoPt& target) const {
    return azimuthTo(target._lat, target._lon);
}
float GeoPt::azimuthTo(float toLat, float toLon) const {
    float X = cosf(TO_RADF(toLat)) * sinf(TO_RADF(toLon-_lon));
    float Y = cosf(TO_RADF(_lat)) * sinf(TO_RADF(toLat)) -
              (sinf(TO_RADF(_lat)) * cosf(TO_RADF(toLat)) * cosf(TO_RADF(toLon - _lon)));
    float azimuthRad = atan2f(X, Y);
    return TO_DEGF(azimuthRad);
}



float compute_azimuth(const GeoPt& from, const GeoPt& to) {
     return from.azimuthTo(to);
}


float compute_distance(const GeoPt& from, const GeoPt& to) {
    double dLat = (M_PI / 180.0)*(to.getLatitude()-from.getLatitude());
    double dLon = (M_PI / 180.0)*(to.getLongitude()-from.getLongitude());
    double lat1 = (M_PI / 180.0)*from.getLatitude();
    double lat2 = (M_PI / 180.0)*to.getLatitude();

    double a = sin(dLat/2) * sin(dLat/2) +
            sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return 1000.* EARTH_RADIUS_KM * c;
}

float compute_tilt(const GeoPt& from, const GeoPt& to) {
    double distance = compute_distance(from, to);
    double lambda = asin((to.getElevation()-from.getElevation())/distance);
    return lambda * (180. / M_PI);
}