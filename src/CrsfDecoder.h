#ifndef __CRSF_DECODER_H__
#define __CRSF_DECODER_H__

#include <Arduino.h>
#include "Decoder.h"

#define CRSF_MAX_FRAME_SIZE    64

class CRSFDecoder : public Decoder {
public:
    CRSFDecoder();
    virtual ~CRSFDecoder();
    virtual void reset(); 
    virtual bool process(uint8_t data);

protected:
    bool decodeFrame();


    enum State { IDLE, LENGTH, DATA, CRC };

    State _state;
    uint8_t  _buffer[CRSF_MAX_FRAME_SIZE];
    uint32_t _index;
    
    uint8_t  _length;
    uint8_t  _crc;
};




#endif // __CRSF_DECODER_H__