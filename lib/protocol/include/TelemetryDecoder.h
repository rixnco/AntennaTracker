#ifndef __TELEMETRY_DECODER_H__
#define __TELEMETRY_DECODER_H__

#include "ProtocolDecoder.h"
#include <stdint.h>

enum TelemetryError { TLM_ERROR_CRC, TLM_ERROR_OVERFLOW, TLM_ERROR_UNKNOWN_ID, TLM_ERROR_BAD_FORMAT };

class TelemetryDecoder;

class TelemetryListener {
public:
    virtual ~TelemetryListener();

    virtual void onFrameDecoded(TelemetryDecoder* decoder, uint32_t id);
    virtual void onFrameError(TelemetryDecoder* decoder, TelemetryError cause, uint32_t param);
    virtual void onFuelData(TelemetryDecoder* decoder, int fuel);
    virtual void onGPSData(TelemetryDecoder* decoder, float latitude, float longitude);
    virtual void onVBATData(TelemetryDecoder* decoder, float voltage);
    virtual void onCellVoltageData(TelemetryDecoder* decoder, float voltage);
    virtual void onCurrentData(TelemetryDecoder* decoder, float current);
    virtual void onHeadingData(TelemetryDecoder* decoder, float heading);
    virtual void onRSSIData(TelemetryDecoder* decoder, int rssi);
    virtual void onRxBtData(TelemetryDecoder* decoder, float voltage);
    virtual void onGPSStateData(TelemetryDecoder* decoder, int satellites, bool gpsFix);
    virtual void onVSpeedData(TelemetryDecoder* decoder, float vspeed);
    virtual void onAltitudeData(TelemetryDecoder* decoder, float altitude);
    virtual void onGPSAltitudeData(TelemetryDecoder* decoder, float altitude);
    virtual void onDistanceData(TelemetryDecoder* decoder, int distance);
    virtual void onRollData(TelemetryDecoder* decoder, float rollAngle);
    virtual void onPitchData(TelemetryDecoder* decoder, float pitchAngle);
    virtual void onGSpeedData(TelemetryDecoder* decoder, float speed);
    virtual void onAirSpeedData(TelemetryDecoder* decoder, float speed);
};


class TelemetryDecoder : public ProtocolDecoder {
public:
    TelemetryDecoder(std::string name);
    virtual ~TelemetryDecoder();

    virtual void reset() = 0;

    virtual void process(uint8_t data) = 0;

    void setTelemetryListener(TelemetryListener* listener);
protected:
    void fireFrameDecoded(uint32_t id);
    void fireFrameError(TelemetryError cause, uint32_t param=0);
    void fireFuelData(int fuel);
    void fireGPSData(float latitude, float longitude);
    void fireVBATData(float voltage);
    void fireCellVoltageData(float voltage);
    void fireCurrentData(float current);
    void fireHeadingData(float heading);
    void fireRSSIData(int rssi);
    void fireRxBtData(float voltage);
    void fireGPSStateData(int satellites, bool gpsFix);
    void fireVSpeedData(float vspeed);
    void fireAltitudeData(float altitude);
    void fireGPSAltitudeData(float altitude);
    void fireDistanceData(int distance);
    void fireRollData(float rollAngle);
    void firePitchData(float pitchAngle);
    void fireGSpeedData(float speed);
    void fireAirSpeedData(float speed);
private:
    TelemetryListener*  _listener;
};


#endif // __TELEMETRY_DECODER_H__