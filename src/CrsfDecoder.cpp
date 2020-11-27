

#include "CrsfDecoder.h"
#include "crc.h"


#define RADIO_ADDRESS       0xEA
#define MAX_PAYLOAD         (CRSF_MAX_FRAME_SIZE-2)
#define MIN_PAYLOAD         (6)


CRSFDecoder::CRSFDecoder() 
{
    reset();
}

CRSFDecoder::~CRSFDecoder()
{
}

void CRSFDecoder::reset() {
    _state = IDLE;
    _index = 0;
    _length = 0;
    _crc  = 0;
}

bool CRSFDecoder::process(uint8_t data)
{
    Serial.print(" ");
    Serial.print(data, HEX);

    switch(_state) {
    case IDLE:
        if(data == RADIO_ADDRESS) {
            _state = LENGTH;
        }
        break;
    case LENGTH:
        if(data < 2 || data> MAX_PAYLOAD) {
            _state = IDLE;
        } else {
            _length = data;
            _index = 0;
            _crc  = 0;
            _state = DATA;
        }
        break;
    case DATA:
        _buffer[_index++] = data;
        _crc = crc8( _crc, data);
        if(_index >= _length) {
            _state = CRC;
        }
        break;
    case CRC:
        _state = IDLE;
        if(_crc == data) {
            decodeFrame();
            return true;
        }
        break;
    }
    return false;
}

bool CRSFDecoder::decodeFrame() {
    Serial.print("received: EA ");
    Serial.print(_length, HEX);
    for(int t=0; t< _length; ++t) {
        Serial.print(" ");
        Serial.print(_buffer[t], HEX);
    }
    Serial.print(" ");
    Serial.println(_crc, HEX);

    return true;
}
