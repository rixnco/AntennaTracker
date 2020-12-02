//
// Created by Samuel HEIDMANN on 01/12/2020.
//

#ifndef __GPSUTILS_H__
#define __GPSUTILS_H__

class GeoPt {
public:
    GeoPt();
    GeoPt(float lat, float lon, float elev=0);
    GeoPt(const GeoPt& other);

    GeoPt& operator =(const GeoPt& other);

    float getLatitude() const;
    float getLongitude() const;
    float getElevation() const;
    void  setLatitude(float lat);
    void  setLongitude(float lon);
    void  setElevation(float elev);

    float azimuthTo(const GeoPt& target) const;
    float azimuthTo(float toLat, float toLon) const;

private:
    float _lat;
    float _lon;
    float _elev;
};

float compute_azimuth(const GeoPt& from, const GeoPt& to);

#endif // __GPSUTILS_H__
