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

bool GeoPt::operator ==(const GeoPt& other) 
{
    return _lat==other._lat && _lon==other._lon;
}

bool GeoPt::operator !=(const GeoPt& other)
{
    return _lat!=other._lat || _lon!=other._lon;
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
    float X = cosf(TO_RADF(target._lat)) * sinf(TO_RADF(target._lon-_lon));
    float Y = cosf(TO_RADF(_lat)) * sinf(TO_RADF(target._lat)) -
              (sinf(TO_RADF(_lat)) * cosf(TO_RADF(target._lat)) * cosf(TO_RADF(target._lon - _lon)));
    float azimuthRad =  atan2f(X, Y) ;
    return TO_DEGF(azimuthRad);
}


float GeoPt::distanceTo(const GeoPt& target) const {
    float dLat = TO_RADF(target._lat-_lat);
    float dLon = TO_RADF(target._lon-_lon);
    float lat1 = TO_RADF(_lat);
    float lat2 = TO_RADF(target._lat);

    float a = sinf(dLat/2) * sinf(dLat/2) +
            sinf(dLon/2) * sinf(dLon/2) * cosf(lat1) * cosf(lat2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return 1000.f * EARTH_RADIUS_KM * c;
}

float GeoPt::tiltTo(const GeoPt& target) const {
   float distance = distanceTo(target);
    float lambda = asinf((target._elev-_elev)/distance);
    return TO_DEGF(lambda);
}


float compute_azimuth(const GeoPt& from, const GeoPt& to) {
     return from.azimuthTo(to);
}


float compute_distance(const GeoPt& from, const GeoPt& to) {
    return from.distanceTo(to);
}

float compute_tilt(const GeoPt& from, const GeoPt& to) {
    return from.tiltTo(to);
}