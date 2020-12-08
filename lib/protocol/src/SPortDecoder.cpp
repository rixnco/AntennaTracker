#include "SPortDecoder.h"

#define START_BYTE 0x7E
#define DATA_START 0x10
#define ESCAPE_BYTE 0x7D
#define ESCAPE_MASK 0x20


#define ALT_SENSOR          0x0100
#define VSPEED_SENSOR       0x0110

#define CURRENT_SENSOR      0x0200
#define VFAS_SENSOR         0x0210

#define FLYMODE_SENSOR      0x0400  // T1_SENSOR used to transmit fly mode
#define T11_SENSOR          0x0401
#define GPS_STATE_SENSOR    0x0410
#define DISTANCE_SENSOR     0x0420

#define FUEL_SENSOR         0x0600

#define ACCEL_X             0x0700
#define ACCEL_Y             0x0710
#define ACCEL_Z             0x0720
#define PITCH_SENSOR        0x5230
#define ROLL_SENSOR         0x5240

#define GPS_SENSOR          0x0800
#define GALT_SENSOR         0x0820
#define GSPEED_SENSOR       0x0830
#define HEADING_SENSOR      0x0840


#define CELL_SENSOR         0x0910

#define AIRSPEED_SENSOR     0x0A00

#define RSSI_SENSOR         0xF101
#define A1_SENSOR           0xF102
#define A2_SENSOR           0xF103
#define RxBt_SENSOR         0xF104          //https://github.com/Clooney82/MavLink_FrSkySPort/wiki/1.2.-FrSky-Taranis-Telemetry 


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

#define UNUSED(x) (void)(x)



// void computeCRC(int a, int b, uint8_t* buffer) {
//     uint16_t  crc;
//     crc=0;
//     for(int t=a; t<b;++t) {
//         printf("%02X ", buffer[t]);
//         crc += buffer[t];  //0-1FF
//     }
//     crc = (crc & 0xFF) + (crc >> 8);
//     // crc += crc >> 8;    //0-100
//     // crc &= 0x00FF;
//     // uint8_t ucrc = crc;
//     printf(" ####### %02X - %02X = %02X\n", (uint8_t)crc, (uint8_t)buffer[b-1], (uint8_t)(crc-buffer[b-1]));
// }

SPortDecoder::SPortDecoder() : TelemetryDecoder("smartport")
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

void SPortDecoder::process(uint8_t data)
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
        if (data == ESCAPE_BYTE)
        {
            _state = ESCAPE;
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
    case ESCAPE:
        _buffer[_index++] = data ^ ESCAPE_MASK;
        _state = DATA;

        break;
    }

    if (_index == SPORT_FRAME_SIZE)
    {   
        _state = IDLE;
        _index = 0;

        // uint16_t crc=0;
        // computeCRC(0,8, _buffer);
        // computeCRC(1,8, _buffer);
        //  computeCRC(2,9, _buffer);
        //  return;
        // computeCRC(0,9, _buffer);
        // computeCRC(1,9, _buffer);
        // computeCRC(2,9, _buffer);

        // computeCRC(0,7, _buffer);
        // computeCRC(1,7, _buffer);
        // computeCRC(2,7, _buffer);


        // if(crc != 0x00FF) {
        //     fireFrameError(TLM_ERROR_CRC);
        //     return;
        // }
        uint8_t sensorType = _buffer[0];
        UNUSED(sensorType);
        uint8_t packetType = _buffer[1];
        if (packetType == DATA_START)
        {
            uint16_t dataType = (_buffer[2]) | (_buffer[3] << 8);
            uint32_t rawData  = ((uint32_t)_buffer[4]) | (((uint32_t)_buffer[5]) << 8) | (((uint32_t)_buffer[6]) << 16) | (((uint32_t)_buffer[7]) << 24);

            switch (dataType)
            {
            case VFAS_SENSOR:
                fireFrameDecoded(dataType);
                fireVBATData(rawData / 100.f);
                break;
            case CELL_SENSOR:
                fireFrameDecoded(dataType);
                fireCellVoltageData(rawData / 100.f);
                break;
            case VSPEED_SENSOR:
                fireFrameDecoded(dataType);
                fireVSpeedData(((int32_t)rawData) / 100.f);
                break;
            case GSPEED_SENSOR:
                fireFrameDecoded(dataType);
                fireGSpeedData((rawData / (1944.f / 100.f)) / 27.778f);
                break;
            case ALT_SENSOR:
                fireFrameDecoded(dataType);
                fireAltitudeData(((int32_t)rawData) / 100.f);
                break;
            case GALT_SENSOR:
                fireFrameDecoded(dataType);
                fireGPSAltitudeData(((int32_t)rawData) / 100.f);
                break;
            case DISTANCE_SENSOR:
                fireFrameDecoded(dataType);
                fireDistanceData(rawData);
                break;
            case FUEL_SENSOR:
                fireFrameDecoded(dataType);
                fireFuelData(rawData);
                break;
            case GPS_SENSOR:
            {
                fireFrameDecoded(dataType);
                double gpsData = (rawData & 0x3FFFFFFF) / 10000.0 / 60.0;
                if ((rawData & 0x40000000) > 0)
                {
                    gpsData = -gpsData;
                }
                if ((rawData & 0x80000000) == 0)
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
                    fireGPSData(_latitude, _longitude);
                }
                break;
            }
            case CURRENT_SENSOR:
                fireFrameDecoded(dataType);
                fireCurrentData(rawData / 10.f);
                break;
            case HEADING_SENSOR:
                fireFrameDecoded(dataType);
                fireHeadingData(rawData / 100.f);
                break;
            case RSSI_SENSOR:
                fireFrameDecoded(dataType);
                fireRSSIData(rawData);
                break;
            case A1_SENSOR:
            case A2_SENSOR:
                fireFrameDecoded(dataType);
                // NOT IMPLEMENTED
                break;
            case RxBt_SENSOR:
                fireFrameDecoded(dataType);
                fireRxBtData(5.2f);
                // NOT IMPLEMENTED
                break;
            case FLYMODE_SENSOR:
                fireFrameDecoded(dataType);
                // NOT IMPLEMENTED
                break;
            case T11_SENSOR:
                fireFrameDecoded(dataType);
                // NOT IMPLEMENTED
                break;
            case GPS_STATE_SENSOR:
            {
                int satellites = rawData % 100;
                bool isFix = rawData > 1000;
                fireFrameDecoded(dataType);
                fireGPSStateData(satellites, isFix);
                break;
            }
            case PITCH_SENSOR:
                fireFrameDecoded(dataType);
                firePitchData(((int32_t)rawData) / 10.f);
                break;
            case ROLL_SENSOR:
                fireFrameDecoded(dataType);
                fireRollData(((int32_t)rawData) / 10.f);
                break;

            case AIRSPEED_SENSOR:
            {
                float *pdata = (float *)&rawData;
                fireFrameDecoded(dataType);
                fireAirSpeedData((*pdata) * 1.852f);
                break;
            }
            case ACCEL_X:
            case ACCEL_Y:
            case ACCEL_Z:
                fireFrameDecoded(dataType);
                // Not implemented
                break;
            case 0xF000:
                fireFrameDecoded(dataType);
                // Not implemented
                break;
            default:
//                fireFrameDecoded(dataType);
                fireFrameError(TLM_ERROR_UNKNOWN_ID, dataType);
                break;
            }
        }
    }
}
