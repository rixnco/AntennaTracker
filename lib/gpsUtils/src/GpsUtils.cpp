//
// Created by Samuel HEIDMANN on 01/12/2020.
//
#include <Arduino.h>
#include "GpsUtils.h"

GpsPt::GpsPt() {
    this->lati = 0.;
    this->longi = 0.;
    this->elev = 0;
}

GpsPt::GpsPt(double lati, double longi, double elev) {
    this->lati = lati;
    this->longi = longi;
    this->elev = elev;
}

double GpsPt::latitude() {
    return this->lati;
}

double GpsPt::longitude() {
    return this->longi;
}

double GpsPt::elevation() {
    return this->elev;
}

float compute_azimuth(GpsPt& from, GpsPt& to) {
    double X = cos((M_PI / 180.0) * to.latitude()) * sin((M_PI / 180.0) * (to.longitude()-from.longitude()));
    double Y = cos((M_PI / 180.0) * from.latitude()) * sin((M_PI / 180.0) * to.latitude()) -
              (sin((M_PI / 180.0) * from.latitude()) * cos((M_PI / 180.0) * to.latitude()) * cos((M_PI / 180.0) * (to.longitude() - from.longitude())));
    double azimuthRad = atan2(X, Y);
    Serial.print("X: ");
    double azimuthDeg = azimuthRad * 180. / M_PI;
    return (float)azimuthDeg;
}