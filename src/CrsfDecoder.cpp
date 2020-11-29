

#include "CrsfDecoder.h"
#include "crc.h"


#define RADIO_ADDRESS       0xEA
#define MAX_PAYLOAD         (CRSF_MAX_FRAME_SIZE-2)
#define MIN_PAYLOAD         (6)

// Frame id
#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define LINK_ID                        0x14
#define CHANNELS_ID                    0x16
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A
#define COMMAND_ID                     0x32
#define RADIO_ID                       0x3A

bool getCrossfireTelemetryValue(int32_t & value, uint8_t * pbuffer, uint8_t N)
{
    bool result = false;
    uint8_t * byte = pbuffer;
    value = (*byte & 0x80) ? -1 : 0;
    for (uint8_t i=0; i<N; i++) {
        value <<= 8;
        if (*byte != 0xff) {
            result = true;
        }
        value += *byte++;
    }
    return result;
}

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

void CRSFDecoder::onDataReceived(uint8_t data) { process(data); }

bool CRSFDecoder::process(uint8_t data)
{

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
        if(_crc == data) {
            decodeFrame();
            return true;
        }
        break;
    }
    return false;
}

bool CRSFDecoder::decodeFrame() {
    uint8_t id = _buffer[0];
    int32_t value;
    switch(id) {
        case CF_VARIO_ID:
            if (getCrossfireTelemetryValue( value, &_buffer[1], 2))
                onVSpeedData(value / 100.f);
            break;
        case GPS_ID: {
            onGPSData(0,0);
            onGPSAltitudeData(0);
            onGPSStateData(0,true);
            break;
        }

    }


    return true;
}
