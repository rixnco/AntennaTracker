#include "FrieshDecoder.h"

#define FRIESH_SETTING_REQ '$'
#define FRIESH_TELEMETRY_REQ '!'


// Settings parameters ID
enum setting_id_t {
    SETTING_HOME,
    SETTING_CALIB,
    SETTING_PAN,
    SETTING_TILT,
    SETTING_ALT,
    SETTING_MODE,
    SETTING_ADC_FACTOR,
    SETTING_LAST
};

static const char *SETTING_NAME[] = {
    "HOME",
    "CALIB",
    "PAN",
    "TILT",
    "ALTITUDE",
    "MODE",
    "ADCFACTOR"
};

static const char *TRACKER_MODE_STR[] = {
    "TRACK",
    "CALIB"
};

static const char *ALTITUDE_MODE_STR[] = {
    "GPS",
    "BARO"
};

// Telemetry parameters ID
enum telemetry_id_t {
    TELEMETRY_GPS,
    TELEMETRY_FIX,
    TELEMETRY_SATELLITES,
    TELEMETRY_ALTITUDE,
    TELEMETRY_VBAT,
    TELEMETRY_RXBT,
    TELEMETRY_LAST
};

static const char *TELEMETRY_STR[] = {
    "GPS",
    "FIX",
    "SATELLITES",
    "ALTITUDE",
    "VBAT",
    "RXBT"
};




static int getSettingID(const char *str, char **endptr);
static int getTelemetryID(const char *str, char **endptr);
static trackerMode_t getTrackerMode(const char *str, char **endptr);
static altitudeMode_t getAltitudeMode(const char *str, char **endptr);


FrieshDecoder::FrieshDecoder() : ProtocolDecoder("FrieshDecoder"), _out(nullptr), _settings(nullptr), _telemetry(nullptr)
{
}
FrieshDecoder::FrieshDecoder(SettingsManager* settings, TelemetryProvider* telemetry) : ProtocolDecoder("FrieshDecoder"), _out(nullptr),  _settings(settings), _telemetry(telemetry)
{
}

FrieshDecoder::~FrieshDecoder()
{
}

void FrieshDecoder::setOutputStream(Print *out)
{
    _out = out;
}

void FrieshDecoder::setSettingsManager(SettingsManager* settings)
{
    _settings = settings;
}

void FrieshDecoder::setTelemetryProvider(TelemetryProvider* telemetry)
{
    _telemetry = telemetry;
}

void FrieshDecoder::reset()
{
    _buffer[0] = 0;
    _index = 0;
}

void FrieshDecoder::process(uint8_t c)
{
    if (_index >= FRIESH_BUFFER_SIZE)
    {
        // overflow !!!
        _index = FRIESH_BUFFER_SIZE;
    }

    if (c == '\n')
    {
        _buffer[_index] = 0;
        decodeCommand();
        _index = 0;
    }
    else
    {
        if (c != '\r' || (_index==0 && c != ' ' && c != '\t'))
            _buffer[_index++] = c;
    }
}

bool FrieshDecoder::decodeCommand()
{
    switch (_buffer[0])
    {
    case FRIESH_SETTING_REQ:
        if (!decodeSettingRequest())
        {
            sendError("Invalid setting command");
        }
        else
        {
            sendAck();
        }
        break;
    case FRIESH_TELEMETRY_REQ:
        if (!decodeTelemetryRequest())
        {
            sendError("Invalid telemetry command");
        }
        else
        {
            sendAck();
        }
        break;
    default:
        sendError("Unknown command");
    }

    return true;
}

void FrieshDecoder::sendError(const char *msg)
{
    if (_out)
        _out->printf(">ERROR:%s\n", msg);
}

void FrieshDecoder::sendAck()
{
    if (_out)
        _out->printf(">OK\n");
}

bool FrieshDecoder::decodeSettingRequest()
{
    char *ptr = &_buffer[1];

    switch (*ptr)
    {
    case '$':
    {
        ++ptr;
        if (*ptr == 0)
        {
            sendSettings();
            return true;
        }
        break;
    }
    case '<':
    {
        ++ptr;
        if (*ptr == 0)
        {
            if (_settings)
                return _settings->store();
            return false;
        }
        break;
    }
    case '>':
    {
        ++ptr;
        if (*ptr == 0)
        {
            if (_settings)
                return _settings->load();
            return false;
        }
        break;
    }
    case '#':
    {
        ++ptr;
        if (*ptr == 0)
        {
            if (_settings) {
                _settings->reset();
                return true;
            }
            return false;
        }
        break;
    }
    default:
    {
        int p = getSettingID(ptr, &ptr);
        if (p == -1)
            return false;
        if (*ptr == 0)
        {
            sendSetting(p);
            return true;
        }
        else if (*ptr == '=')
        {
            ++ptr;
            if (*ptr == 0 || !setSetting(p, ptr))
                return false;
            return true;
        }
        else if (*ptr == '+')
        {
            if (!adjSetting(p, ptr))
                return false;
            return true;
        }
        else if (*ptr == '-')
        {
            if (!adjSetting(p, ptr))
                return false;
            return true;
        }
    }
    }
    return false;
}

void FrieshDecoder::sendSetting(int p)
{
    if (p < 0 || p >= SETTING_LAST || _settings == nullptr || _out == nullptr)
        return;
    switch (p)
    {
    case SETTING_HOME:
        _out->printf("$%s=%.5f %.5f %.1f %s\n", SETTING_NAME[p], _settings->getHomeLatitude(), _settings->getHomeLongitude(), _settings->getHomeElevation(), _settings->getHomeName());
        break;
    case SETTING_CALIB:
        _out->printf("$%s=%.5f %.5f %.1f %s\n", SETTING_NAME[p], _settings->getCalibLatitude(), _settings->getCalibLongitude(), _settings->getCalibElevation(), _settings->getCalibName());
        break;
    case SETTING_PAN:
        _out->printf("$%s=%d\n", SETTING_NAME[p], _settings->getPanOffset());
        break;
    case SETTING_TILT:
        _out->printf("$%s=%d\n", SETTING_NAME[p], _settings->getTiltOffset());
        break;
    case SETTING_ALT:
        _out->printf("$%s=%s\n", SETTING_NAME[p], ALTITUDE_MODE_STR[_settings->getAltitudeMode()]);
        break;
    case SETTING_MODE:
        _out->printf("$%s=%s\n", SETTING_NAME[p], TRACKER_MODE_STR[_settings->getTrackerMode()]);
        break;
    case SETTING_ADC_FACTOR:
        _out->printf("$%s=%s\n", SETTING_NAME[p], String(_settings->getAdcBattFactor()));
        break;
    default:
        _out->println("$UNKNOWN");
    }
}
#include <Arduino.h>
void FrieshDecoder::sendSettings()
{
    for (int t = 0; t < SETTING_LAST; ++t)
    {
        sendSetting(static_cast<setting_id_t>(t));
    }
}


static char tmpBuffer[SETTINGS_MAX_NAME_LEN+1];

bool FrieshDecoder::setSetting(int p, char *ptr)
{
    if (_settings == nullptr)
        return false;

    switch (p)
    {
    case SETTING_HOME:
    {
        float lat, lon, elv;
        char* start;
        // parse latitude
        start= ptr;
        lat = strtod(start, &ptr);
        if(start==ptr || *ptr!=' ') return false;

        // parse longitude
        start = ptr;
        lon = -100000;
        lon = strtod(start, &ptr);
        if(start==ptr || (*ptr!='\0' && *ptr!=' ')) return false;

        // parse elevation (optional)
        start = ptr;
        elv = -100000;
        elv = strtod(start, &ptr);
        if(start==ptr) elv =0;
        if(*ptr!='\0' && *ptr!=' ') return false;

        // trim withespaces
        while(isblank(*ptr)) ++ptr;

        _settings->setHome(lat, lon, elv);
        _settings->setHomeName(ptr);
        break;
    }
    case SETTING_CALIB:
    {
        float lat, lon, elv;
        char* start;
        // parse latitude
        start= ptr;
        lat = strtod(start, &ptr);
        if(start==ptr || *ptr!=' ') return false;

        // parse longitude
        start = ptr;
        lon = -100000;
        lon = strtod(start, &ptr);
        if(start==ptr || (*ptr!='\0' && *ptr!=' ')) return false;

        // parse elevation (optional)
        start = ptr;
        elv = -100000;
        elv = strtod(start, &ptr);
        if(start==ptr) elv =0;
        if(*ptr!='\0' && *ptr!=' ') return false;

        // trim withespaces
        while(isblank(*ptr)) ++ptr;
        _settings->setCalib(lat, lon, elv);
        _settings->setCalibName(ptr);
        break;
    }
    case SETTING_PAN:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        _settings->setPanOffset(val);
        break;
    }
    case SETTING_TILT:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        _settings->setTiltOffset(val);
        break;
    }
    case SETTING_ALT:
    {   
        altitudeMode_t mode = getAltitudeMode(ptr, &ptr);
        if(mode==ALT_NONE) return false;
        _settings->setAltitudeMode(mode);
        break;
    }
    case SETTING_MODE:
    {   
        trackerMode_t mode = getTrackerMode(ptr, &ptr);
        if(mode==MODE_NONE) return false;
        _settings->setTrackerMode(mode);
        break;
    }
    case SETTING_ADC_FACTOR:
    {
        float val = strtof(ptr, &ptr);
        if (*ptr != 0)
            return false;
        _settings->setAdcBattFactor(val);
        break;
    }
    default:
        return false;
    }
    return true;
}

bool FrieshDecoder::adjSetting(int p, char *ptr)
{
    if (_settings == nullptr)
        return false;

    switch (p)
    {
    case SETTING_PAN:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        _settings->adjPanOffset(val);
        break;
    }
    case SETTING_TILT:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        _settings->adjTiltOffset(val);
        break;
    }
    default:
        return false;
    }
    return true;
}


bool FrieshDecoder::decodeTelemetryRequest()
{
    char *ptr = &_buffer[1];

    switch (*ptr)
    {
        case '!':
        {
            ++ptr;
            if (*ptr == 0)
            {
                sendTelemetries();
                return true;
            }
            break;
        }
        default:
        {
            int p = getTelemetryID(ptr, &ptr);
            if (p == -1)
                return false;
            if (*ptr == 0)
            {
                sendTelemetry(p);
                return true;
            }
        }
    }
    return false;
}

void FrieshDecoder::sendTelemetry(int p)
{
    if (p < 0 || p >= TELEMETRY_LAST || _telemetry == nullptr || _out == nullptr)
        return;
    switch (p)
    {
    case TELEMETRY_GPS:
    {
        GeoPt gps = _telemetry->getGPS();
        _out->printf("!%s=%.5f %.5f %.1f\n", TELEMETRY_STR[p], gps.getLatitude(), gps.getLongitude(), gps.getElevation());
        break;
    }
    case TELEMETRY_FIX:
        _out->printf("!%s=%d\n", TELEMETRY_STR[p], _telemetry->hasFix());
        break;
    case TELEMETRY_SATELLITES:
        _out->printf("!%s=%d\n", TELEMETRY_STR[p], _telemetry->getSatellites());
        break;
    case TELEMETRY_ALTITUDE:
        _out->printf("!%s=%.1f\n", TELEMETRY_STR[p], _telemetry->getAltitude());
        break;
    case TELEMETRY_VBAT:
        _out->printf("!%s=%.1f\n", TELEMETRY_STR[p], _telemetry->getVBat());
        break;
    case TELEMETRY_RXBT:
        _out->printf("!%s=%.1f\n", TELEMETRY_STR[p], _telemetry->getRxBt());
        break;
    default:
        _out->println("!UNKNOWN");
    }
}

void FrieshDecoder::sendTelemetries()
{
    for (int t = 0; t < TELEMETRY_LAST; ++t)
    {
        sendTelemetry(static_cast<telemetry_id_t>(t));
    }
}




/**
 * Get Setting ID based on its name.
 */
int getSettingID(const char *str, char **endptr)
{
    int l, t, r;
    char *ptr = (char *)str;
    while (isalnum(*ptr) || *ptr == '_')
    {
        *ptr = toupper(*ptr);
        ++ptr;
    }
    l = ptr - str;
    if (endptr != NULL)
    {
        *endptr = (char *)str + l;
    }
    r = SETTING_LAST;
    for (t = 0; t < SETTING_LAST; ++t)
    {
        if (strncmp(SETTING_NAME[t], str, l) == 0)
        {
            if (r != SETTING_LAST)
            {
                r = SETTING_LAST;
                break;
            }
            else
            {
                r = t;
            }
        }
    }
    if (r == SETTING_LAST)
        return -1;
    return r;
}

/**
 * Get Setting ID based on its name.
 */
trackerMode_t getTrackerMode(const char *str, char **endptr)
{
    int l, t, r;
    char *ptr = (char *)str;
    while (isalnum(*ptr) || *ptr == '_')
    {
        *ptr = toupper(*ptr);
        ++ptr;
    }
    l = ptr - str;
    if (endptr != NULL)
    {
        *endptr = (char *)str + l;
    }
    r = MODE_NONE;
    for (t = 0; t < MODE_NONE; ++t)
    {
        if (strncmp(TRACKER_MODE_STR[t], str, l) == 0)
        {
            if (r != MODE_NONE)
            {
                r = MODE_NONE;
                break;
            }
            else
            {
                r = t;
            }
        }
    }
    return static_cast<trackerMode_t>(r);
}

/**
 * Get Setting ID based on its name.
 */
altitudeMode_t getAltitudeMode(const char *str, char **endptr)
{
    int l, t, r;
    char *ptr = (char *)str;
    while (isalnum(*ptr) || *ptr == '_')
    {
        *ptr = toupper(*ptr);
        ++ptr;
    }
    l = ptr - str;
    if (endptr != NULL)
    {
        *endptr = (char *)str + l;
    }
    r = ALT_NONE;
    for (t = 0; t < ALT_NONE; ++t)
    {
        if (strncmp(ALTITUDE_MODE_STR[t], str, l) == 0)
        {
            if (r != ALT_NONE)
            {
                r = ALT_NONE;
                break;
            }
            else
            {
                r = t;
            }
        }
    }
    return static_cast<altitudeMode_t>(r);
}



/**
 * Get Setting ID based on its name.
 */
int getTelemetryID(const char *str, char **endptr)
{
    int l, t, r;
    char *ptr = (char *)str;
    while (isalnum(*ptr) || *ptr == '_')
    {
        *ptr = toupper(*ptr);
        ++ptr;
    }
    l = ptr - str;
    if (endptr != NULL)
    {
        *endptr = (char *)str + l;
    }
    r = TELEMETRY_LAST;
    for (t = 0; t < TELEMETRY_LAST; ++t)
    {
        if (strncmp(TELEMETRY_STR[t], str, l) == 0)
        {
            if (r != TELEMETRY_LAST)
            {
                r = TELEMETRY_LAST;
                break;
            }
            else
            {
                r = t;
            }
        }
    }
    if (r == TELEMETRY_LAST)
        return -1;
    return r;
}


