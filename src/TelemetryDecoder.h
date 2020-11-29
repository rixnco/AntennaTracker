#ifndef __DECODER_H__
#define __DECODER_H__

#include <Arduino.h>

class TelemetryListener {
public:
    virtual ~TelemetryListener();

    virtual void onFuelData(int fuel);
    virtual void onGPSData(double latitude, double longitude);
    virtual void onVBATData(float voltage);
    virtual void onCellVoltageData(float voltage);
    virtual void onCurrentData(float current);
    virtual void onHeadingData(float heading);
    virtual void onRSSIData(int rssi);
    virtual void onGPSStateData(int satellites, bool gpsFix);
    virtual void onVSpeedData(float vspeed);
    virtual void onAltitudeData(float altitude);
    virtual void onGPSAltitudeData(float altitude);
    virtual void onDistanceData(int distance);
    virtual void onRollData(float rollAngle);
    virtual void onPitchData(float pitchAngle);
    virtual void onGSpeedData(float speed);
    virtual void onAirSpeedData(float speed);
};


class TelemetryDecoder {
public:
    TelemetryDecoder();
    virtual ~TelemetryDecoder();

    virtual void reset() = 0;

    virtual bool process(uint8_t data) = 0;

    void setTelemetryListener(TelemetryListener* listener);
protected:
    void onFuelData(int fuel);
    void onGPSData(double latitude, double longitude);
    void onVBATData(float voltage);
    void onCellVoltageData(float voltage);
    void onCurrentData(float current);
    void onHeadingData(float heading);
    void onRSSIData(int rssi);
    void onGPSStateData(int satellites, bool gpsFix);
    void onVSpeedData(float vspeed);
    void onAltitudeData(float altitude);
    void onGPSAltitudeData(float altitude);
    void onDistanceData(int distance);
    void onRollData(float rollAngle);
    void onPitchData(float pitchAngle);
    void onGSpeedData(float speed);
    void onAirSpeedData(float speed);
private:
    TelemetryListener* _listener;
};



#endif // __DECODER_H__