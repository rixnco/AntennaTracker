

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


#define DEG ( 180.0 / 3.14159265358979323846264 )

#define DECIDEGREES_TO_RADIANS10000(angle) ((int16_t)(1000.0f * (angle) * RAD))
#define RADIANS10000_TO_DECIDEGREES(rad1000) ( rad1000/1000.f * DEG )


static bool getCrossfireTelemetryValue(int32_t & value, uint8_t * pbuffer, uint8_t N)
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

CRSFDecoder::CRSFDecoder() : TelemetryDecoder("crossfire")
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

void CRSFDecoder::process(uint8_t data)
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
            fireFrameError(ERROR_BAD_FORMAT);
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
        } else {
            fireFrameError(ERROR_CRC);
        }
        break;
    }
}

bool CRSFDecoder::decodeFrame() {
    uint8_t id = _buffer[0];
    switch(id) {
        case CF_VARIO_ID:
            int32_t value;
            fireFrameDecoded(id);
            if (getCrossfireTelemetryValue( value, &_buffer[1], 2)) {
                fireVSpeedData(value / 100.f);
            }
            break;
        case GPS_ID: {
            int32_t lat,lon;
            fireFrameDecoded(id);
            if (getCrossfireTelemetryValue(lat, &_buffer[1], 4) &&
                getCrossfireTelemetryValue(lon, &_buffer[5], 4)) {
                fireGPSData(lat/10.f, lon/10.f);
            }
            int32_t speed;
            if (getCrossfireTelemetryValue(speed, &_buffer[9], 2)) {
                fireGSpeedData(speed);
            }
            int32_t heading;
            if (getCrossfireTelemetryValue(heading, &_buffer[11], 2)) {
                fireHeadingData(heading);
            }
            int32_t alt;
            if (getCrossfireTelemetryValue(alt, &_buffer[13], 2)) {
                fireGPSAltitudeData(alt-1000);
            }
            int32_t satellites;
            if (getCrossfireTelemetryValue(satellites, &_buffer[15], 1)) {
                fireGPSStateData(satellites, true);
            }
            break;
        }
        case ATTITUDE_ID: {
            fireFrameDecoded(id);
            int32_t pitch,roll,yaw;
            if (getCrossfireTelemetryValue(pitch, &_buffer[1], 2)) {
                firePitchData(RADIANS10000_TO_DECIDEGREES(pitch)/10.f);
            }
            if (getCrossfireTelemetryValue(roll, &_buffer[3], 2)) {
                fireRollData(RADIANS10000_TO_DECIDEGREES(roll)/10.f);
            }
            if (getCrossfireTelemetryValue(yaw, &_buffer[5], 2)) {
                fireHeadingData(RADIANS10000_TO_DECIDEGREES(yaw)/10.f);
            }
            break;
        }
        case BATTERY_ID: {
            fireFrameDecoded(id);
            int32_t voltage, current, fuel, remaining;
            if (getCrossfireTelemetryValue(voltage, &_buffer[1], 2)) {
                fireVBATData(voltage/10.f);
            }
            if (getCrossfireTelemetryValue(current, &_buffer[3], 2)) {
                fireCurrentData(current/10.f);
            }
            if (getCrossfireTelemetryValue(fuel, &_buffer[5], 3)) {
                fireFuelData(fuel);
            }
            if (getCrossfireTelemetryValue(remaining, &_buffer[8], 1)) {
                // todo: implement...
            }
            break;
        }
        case FLIGHT_MODE_ID: {
            fireFrameDecoded(id);
            // TODO: implement...
            break;
        }
        case LINK_ID: {
            fireFrameDecoded(id);
            // TODO: implement...
            break;
        }
        case RADIO_ID: {
            fireFrameDecoded(id);
            // TODO: implement...
            break;
        }
        default:
//            fireFrameDecoded(id);
            fireFrameError(ERROR_UNKNOWN_ID, id);
            break;

    }


    return true;
}
