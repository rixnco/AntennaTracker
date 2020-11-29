#ifndef __SPORT_DECODER_H__
#define __SPORT_DECODER_H__

#include <Arduino.h>
#include "TelemetryDecoder.h"
#include "DataSource.h"

#define SPORT_FRAME_SIZE    9

class SPortDecoder : public TelemetryDecoder, DataListener  {
public:
    SPortDecoder();
    virtual ~SPortDecoder();
    virtual void reset(); 
    virtual bool process(uint8_t data);
    virtual void onDataReceived(uint8_t data);
protected:
    enum State { IDLE, DATA, XOR };

    State _state;
    uint8_t _buffer[SPORT_FRAME_SIZE];
    uint8_t _index;

    bool    _newLatitude;
    double  _latitude;
    bool    _newLongitude;
    double  _longitude;
};




#endif // __SPORT_DECODER_H__