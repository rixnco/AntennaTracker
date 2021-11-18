#ifndef __FRIESH_DECODER_H__
#define __FRIESH_DECODER_H__

#include <Settings.h>
#include <ProtocolDecoder.h>
#include <TelemetryDecoder.h>
#include <Print.h>

#define FRIESH_BUFFER_SIZE 63

class FrieshDecoder;

class FrieshDecoder : public ProtocolDecoder {
public:
    FrieshDecoder();
    FrieshDecoder(SettingsManager *settings, TelemetryProvider* telemetry);
    virtual ~FrieshDecoder();

    void setOutputStream(Print* out);
    void setSettingsManager(SettingsManager* settings);
    void setTelemetryProvider(TelemetryProvider* telemetry);
    virtual void reset();
    virtual void process(uint8_t data);


protected:
    bool decodeCommand();
    bool decodeSettingRequest();
    bool decodeTelemetryRequest();

    void sendAck();
    void sendError(const char *msg);
    void sendSettings();
    void sendSetting(int p);
    bool setSetting(int p, char *ptr);
    bool adjSetting(int p, char *ptr);

    void sendTelemetries();
    void sendTelemetry(int p);

    char     _buffer[FRIESH_BUFFER_SIZE+1];
    uint32_t _index;
    Print    *_out;
    
    SettingsManager*    _settings;
    TelemetryProvider*  _telemetry;
};




#endif // __FRIESH_DECODER_H__