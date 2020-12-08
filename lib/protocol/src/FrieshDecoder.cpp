#include "FrieshDecoder.h"
#include "crc.h"

#define FRAME_START         0x9D
#define MAX_PAYLOAD         (FRIESH_MAX_FRAME_SIZE-2)
#define MIN_PAYLOAD         (0)

// Request ID
#define REQ_CALIBRATE_COMPASS               0x01
#define REQ_SET_HOME                        0x02
#define REQ_START_TRACKING                  0x0A
#define REQ_STOP_TRACKING                   0x0B

FrieshListener::~FrieshListener() 
{

}

void FrieshListener::onFrameDecoded(FrieshDecoder* decoder, uint32_t id) 
{

}
void FrieshListener::onFrameError(FrieshDecoder* decoder, FrieshError cause, uint32_t param)
{

}
   



FrieshDecoder::FrieshDecoder(bool checkCRC) : DataDecoder("FrieshDecoder"), _checkCRC(checkCRC), _listener(nullptr) 
{
}

FrieshDecoder::~FrieshDecoder()
{
}


void FrieshDecoder::setFrieshListener(FrieshListener* listener) {
    _listener = listener;
}

void FrieshDecoder::setCheckCRC(bool check) {
    _checkCRC= check;
}

void FrieshDecoder::reset() {
    _state = IDLE;
    _index = 0;
    _length = 0;
    _crc  = 0;
}


void FrieshDecoder::process(uint8_t data)
{

    switch(_state) {
    case IDLE:
        if(data == FRAME_START) {
            _state = LENGTH;
        }
        break;
    case LENGTH:
        if(data> MAX_PAYLOAD) {
            _state = IDLE;
            fireFrameError(FRIESH_ERROR_BAD_FORMAT);
        } else {
            _length = data-1;  // remove CRC from the length
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
        if(!_checkCRC || _crc == data) {
            decodeFrame();
        } else {
            fireFrameError(FRIESH_ERROR_CRC);
        }
        break;
    }
}



bool FrieshDecoder::decodeFrame() {
    uint8_t id = _buffer[0];
    switch(id) {
        case REQ_CALIBRATE_COMPASS:
            fireFrameDecoded(id);
            fireCalibrateCompass();
            break;
        case REQ_SET_HOME: 
            fireFrameDecoded(id);
            fireSetHomeLocation();
            break;
        case REQ_START_TRACKING:
            fireFrameDecoded(id);
            fireStartTracking();
            break;
        case REQ_STOP_TRACKING:
            fireFrameDecoded(id);
            fireStopTracking();
            break;
        default:
            fireFrameError(FRIESH_ERROR_UNKNOWN_ID, id);
            return false;
    }

    return true;
}


void FrieshDecoder::fireFrameDecoded(uint32_t id) 
{
    if(_listener!=nullptr) _listener->onFrameDecoded(this, id);    
}
void FrieshDecoder::fireFrameError(FrieshError cause, uint32_t param) 
{
    if(_listener!=nullptr) _listener->onFrameError(this, cause, param);    
}
void FrieshDecoder::fireSetHomeLocation() 
{
    if(_listener!=nullptr) _listener->onSetHomeLocation(this);
}
void FrieshDecoder::fireCalibrateCompass()
{
    if(_listener!=nullptr) _listener->onCalibrateCompass(this);
}

void FrieshDecoder::fireStartTracking()
{
    if(_listener!=nullptr) _listener->onStartTracking(this);
}
void FrieshDecoder::fireStopTracking()
{
    if(_listener!=nullptr) _listener->onStopTracking(this);
}



