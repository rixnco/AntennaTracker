

#include "SPortDecoder.h"

#define START_BYTE 0x7E
#define DATA_START 0x10
#define DATA_STUFF 0x7D
#define STUFF_MASK 0x20

#define VFAS_SENSOR 0x0210
#define CELL_SENSOR 0x0910
#define VSPEED_SENSOR 0x0110
#define GSPEED_SENSOR 0x0830
#define ALT_SENSOR 0x0100
#define GALT_SENSOR 0x0820
#define DISTANCE_SENSOR 0x0420
#define FUEL_SENSOR 0x0600
#define GPS_SENSOR 0x0800
#define CURRENT_SENSOR 0x0200
#define HEADING_SENSOR 0x0840
#define RSSI_SENSOR 0xF101
#define FLYMODE_SENSOR 0x0400
#define GPS_STATE_SENSOR 0x0410
#define PITCH_SENSOR 0x0430
#define ROLL_SENSOR 0x0440
#define AIRSPEED_SENSOR 0x0A00
//ardupilot passthrough sensors
#define ARDU_TEXT_SENSOR 0x5000            // status text (dynamic)
#define ARDU_ATTITUDE_SENSOR 0x5006        //Attitude and range (dynamic)
#define ARDU_VEL_YAW_SENSOR 0x5005         //Vel and Yaw
#define ARDU_AP_STATUS_SENSOR 0x5001       //AP status
#define ARDU_GPS_STATUS_SENSOR 0x5002      //GPS status
#define ARDU_HOME_SENSOR 0x5004            //Home
#define ARDU_BATT_2_SENSOR 0x5008          // Battery 2 status
#define ARDU_BATT_1_SENSOR 0x5003          // Battery 1 status
#define ARDU_PARAM_SENSOR 0x5007           // parameters
#define RxBt_SENSOR 0xF104                 //https://github.com/Clooney82/MavLink_FrSkySPort/wiki/1.2.-FrSky-Taranis-Telemetry 
                                           //ardupilot S.PORT sensors
#define DATA_ID_GPS_ALT_BP_SENSOR 0x0001   //gps altitude integer part
#define DATA_ID_TEMP1_SENSOR 0x0002        //flight mode
#define DATA_ID_FUEL_SENSOR 0x0004         //battery remaining
#define DATA_ID_TEMP2_SENSOR 0x0005        //GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
#define DATA_ID_GPS_ALT_AP_SENSOR 0x0009   //gps altitude decimals
#define DATA_ID_BARO_ALT_BP_SENSOR 0x0010  //altitude integer part
#define DATA_ID_GPS_SPEED_BP_SENSOR 0x0011 //gps speed integer part
#define DATA_ID_GPS_LONG_BP_SENSOR 0x0012  //gps longitude degree and minute integer part
#define DATA_ID_GPS_LAT_BP_SENSOR 0x0013   //send gps lattitude degree and minute integer part
#define DATA_ID_GPS_COURS_BP_SENSOR 0x0014 //heading in degree based on AHRS and not GPS
#define DATA_ID_GPS_SPEED_AP_SENSOR 0x0019 //gps speed decimal part
#define DATA_ID_GPS_LONG_AP_SENSOR 0x001A  //gps longitude minutes decimal part
#define DATA_ID_GPS_LAT_AP_SENSOR 0x001B   //send gps lattitude minutes decimal part
#define DATA_ID_BARO_ALT_AP_SENSOR 0x0021  //gps altitude decimal part
#define DATA_ID_GPS_LONG_EW_SENSOR 0x0022  //gps East / West information
#define DATA_ID_GPS_LAT_NS_SENSOR 0x0023   //gps North / South information
#define DATA_ID_CURRENT_SENSOR 0x0028      //current consumption
#define DATA_ID_VARIO_SENSOR 0x0030        //vspeed m/s
#define DATA_ID_VFAS_SENSOR 0x0039         //battery voltage

SPortDecoder::SPortDecoder() 
{
    reset();
}

SPortDecoder::~SPortDecoder()
{
}

void SPortDecoder::reset() {
    _state = IDLE;
    _index =0;
    _newLatitude = false;
    _newLongitude = false;
}

bool SPortDecoder::process(uint8_t data)
{
    switch (_state)
    {
    case IDLE:
        _index = 0;
        if (data == START_BYTE)
        {
            _state = DATA;
        }
        break;
    case DATA:
        if (data == DATA_STUFF)
        {
            _state = XOR;
        }
        else if (data == START_BYTE)
        {
            _index = 0;
        }
        else
        {
            _buffer[_index++] = data;
        }
        break;
    case XOR:
        _buffer[_index++] = data ^ STUFF_MASK;
        _state = DATA;

        break;
    }

    if (_index == SPORT_FRAME_SIZE)
    {   

        _state = IDLE;
        _index = 0;

        uint8_t sensorType = _buffer[_index++];
        uint8_t packetType = _buffer[_index++];
        if (packetType == DATA_START)
        {
            uint16_t dataType = (_buffer[_index++]) | (_buffer[_index++] << 8);
            uint32_t rawData  = (_buffer[_index++]) | (_buffer[_index++] << 8) | (_buffer[_index++] << 16) | (_buffer[_index++] << 24);

            switch (dataType)
            {
            case VFAS_SENSOR:
                onVBATData(rawData / 100.f);
                break;
            case CELL_SENSOR:
                onCellVoltageData(rawData / 100.f);
                break;
            case VSPEED_SENSOR:
                onVSpeedData(rawData / 100.f);
                break;
            case GSPEED_SENSOR:
                onGSpeedData((rawData / (1944.f / 100.f)) / 27.778f);
                break;
            case ALT_SENSOR:
                onAltitudeData(rawData / 100.f);
                break;
            case GALT_SENSOR:
                onGPSAltitudeData(rawData / 100.f);
                break;
            case DISTANCE_SENSOR:
                onDistanceData(rawData);
                break;
            case FUEL_SENSOR:
                onFuelData(rawData);
                break;
            case GPS_SENSOR:
            {
                double gpsData = (rawData && 0x3FFFFFFF) / 10000.0 / 60.0;
                if (rawData && 0x40000000 > 0)
                {
                    gpsData = -gpsData;
                }
                if (rawData && 0x80000000 == 0)
                {
                    _newLatitude = true;
                    _latitude = gpsData;
                }
                else
                {
                    _newLongitude = true;
                    _longitude = gpsData;
                }
                if (_newLatitude && _newLongitude)
                {
                    _newLongitude = false;
                    _newLatitude = false;
                    onGPSData(_latitude, _longitude);
                }
                break;
            }
            case CURRENT_SENSOR:
                onCurrentData(rawData / 10.f);
                break;
            case HEADING_SENSOR:
                onHeadingData(rawData / 100.f);
                break;
            case RSSI_SENSOR:
                onRSSIData(rawData);
                break;
            case FLYMODE_SENSOR:
                break;
            case GPS_STATE_SENSOR:
            {
                int satellites = rawData % 100;
                bool isFix = rawData > 1000;
                onGPSStateData(satellites, isFix);
                break;
            }
            case PITCH_SENSOR:
                onPitchData(rawData / 10.f);
                break;
            case ROLL_SENSOR:
                onRollData(rawData / 10.f);
                break;
            case AIRSPEED_SENSOR:
            {
                float *pdata = (float *)&rawData;
                onAirSpeedData(*pdata * 1.852f);
                break;
            }
            default:
                return false;
            }
            return true;
        }
    }
    return false;
}
