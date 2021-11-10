#ifndef __FRIESH_DECODER_H__
#define __FRIESH_DECODER_H__

#include <ProtocolDecoder.h>
#include <Print.h>

#define FRIESH_BUFFER_SIZE 63
#define FRIESH_PARAM_REQ '$'

// Configuration parameters ID
// Used by the protocol handler
#define PARAM_HOME      0
#define PARAM_AIM       1
#define PARAM_PAN       2
#define PARAM_TILT      3
#define PARAM_TRACKING  4
#define PARAM_LAST      5

extern const char *PARAM_NAME[];

class FrieshDecoder;

class FrieshHandler {
public:
    virtual ~FrieshHandler();
 
    virtual void  setHome(float lat, float lon, float elv) = 0;
    virtual float getHomeLatitude()= 0;
    virtual float getHomeLongitude()= 0;
    virtual float getHomeElevation()= 0;

    virtual void  setAim(float lat, float lon, float elv) = 0;
    virtual float getAimLatitude() = 0;
    virtual float getAimLongitude() = 0;
    virtual float getAimElevation() = 0;

    virtual void setTracking(bool tracking) = 0;
    virtual bool isTracking() = 0;

    virtual void setPanOffset(int pan) = 0;
    virtual void adjPanOffset(int16_t delta) = 0;
    virtual int  getPanOffset() = 0;

    virtual void setTiltOffset(int pan) = 0;
    virtual void adjTiltOffset(int16_t delta) = 0;
    virtual int  getTiltOffset() = 0;

    virtual bool storeSettings() = 0;
    virtual bool loadSettings() = 0;
};

class FrieshDecoder : public ProtocolDecoder {
public:
    FrieshDecoder();
    virtual ~FrieshDecoder();

    void setOutputStream(Print* out);
    void setFrieshHandler(FrieshHandler* handler);
    virtual void reset();
    virtual void process(uint8_t data);


protected:
    bool decodeFrame();
    bool decodeParamRequest();

    void sendAck();
    void sendError(const char *msg);
    void sendParam(int p);
    void sendSettings();
    bool setParam(int p, char *ptr);
    bool adjParam(int p, char *ptr);

    Print    *_out;
    char     _buffer[FRIESH_BUFFER_SIZE+1];
    uint32_t _index;
    
    FrieshHandler* _handler;
};




#endif // __FRIESH_DECODER_H__