#include "FrieshDecoder.h"

const char *PARAM_NAME[] = {
    "HOME",
    "AIM",
    "PAN",
    "TILT",
    "TRACKING"};

static int getParamID(const char *str, char **endptr);

FrieshHandler::~FrieshHandler()
{
}

FrieshDecoder::FrieshDecoder() : ProtocolDecoder("FrieshDecoder"), _out(nullptr), _handler(nullptr)
{
}

FrieshDecoder::~FrieshDecoder()
{
}

void FrieshDecoder::setOutputStream(Print *out)
{
    _out = out;
}

void FrieshDecoder::setFrieshHandler(FrieshHandler *handler)
{
    _handler = handler;
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
        decodeFrame();
        _index = 0;
    }
    else
    {
        if (c != '\r' || (_index==0 && c != ' ' && c != '\t'))
            _buffer[_index++] = c;
    }
}

bool FrieshDecoder::decodeFrame()
{
    switch (_buffer[0])
    {
    case FRIESH_PARAM_REQ:
        if (!decodeParamRequest())
        {
            sendError("Invalid parameter command");
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

bool FrieshDecoder::decodeParamRequest()
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
            if (_handler)
                return _handler->storeSettings();
            return false;
        }
        break;
    }
    case '>':
    {
        ++ptr;
        if (*ptr == 0)
        {
            if (_handler)
                return _handler->loadSettings();
            return false;
        }
        break;
    }
    default:
    {
        int p = getParamID(ptr, &ptr);
        if (p == -1)
            return false;
        if (*ptr == 0)
        {
            sendParam(p);
            return true;
        }
        else if (*ptr == '=')
        {
            ++ptr;
            if (*ptr == 0 || !setParam(p, ptr))
                return false;
            return true;
        }
        else if (*ptr == '+')
        {
            if (!adjParam(p, ptr))
                return false;
            return true;
        }
        else if (*ptr == '-')
        {
            if (!adjParam(p, ptr))
                return false;
            return true;
        }
    }
    }
    return false;
}

void FrieshDecoder::sendParam(int p)
{
    if (p < 0 || p >= PARAM_LAST || _handler == nullptr || _out == nullptr)
        return;
    switch (p)
    {
    case PARAM_HOME:
        _out->printf("$%s=%.5f %.5f %.1f\n", PARAM_NAME[p], _handler->getHomeLatitude(), _handler->getHomeLongitude(), _handler->getHomeElevation());
        break;
    case PARAM_AIM:
        _out->printf("$%s=%.5f %.5f %.1f\n", PARAM_NAME[p], _handler->getAimLatitude(), _handler->getAimLongitude(), _handler->getAimElevation());
        break;
    case PARAM_PAN:
        _out->printf("$%s=%d\n", PARAM_NAME[p], _handler->getPanOffset());
        break;
    case PARAM_TILT:
        _out->printf("$%s=%d\n", PARAM_NAME[p], _handler->getTiltOffset());
        break;
    case PARAM_TRACKING:
        _out->printf("$%s=%d\n", PARAM_NAME[p], _handler->isTracking());
        break;
    default:
        _out->println("$unknown");
    }
}

void FrieshDecoder::sendSettings()
{
    for (int t = 0; t < PARAM_LAST; ++t)
    {
        sendParam(t);
    }
}


#include <Arduino.h>
bool FrieshDecoder::setParam(int p, char *ptr)
{
    if (_handler == nullptr)
        return false;

    switch (p)
    {
    case PARAM_HOME:
    {
        float lat, lon, elv;
        lat = strtod(ptr, &ptr);
        lon = strtod(ptr, &ptr);
        elv = strtod(ptr, &ptr);
        if (*ptr != 0)
            return false;
        _handler->setHome(lat, lon, elv);
        break;
    }
    case PARAM_AIM:
    {
        float lat, lon, elv;
        lat = strtod(ptr, &ptr);
        lon = strtod(ptr, &ptr);
        elv = strtod(ptr, &ptr);
        if (*ptr != 0)
            return false;
        _handler->setAim(lat, lon, elv);
        break;
    }
    case PARAM_PAN:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        // noInterrupts();
        // g_settings.panOffset=val;
        // interrupts();
        _handler->setPanOffset(val);
        break;
    }
    case PARAM_TILT:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        // noInterrupts();
        // g_settings.tiltOffset=val;
        // interrupts();
        _handler->setTiltOffset(val);
        break;
    }
    case PARAM_TRACKING:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        // noInterrupts();
        // g_tracking = (val!=0);
        // interrupts();
        _handler->setTracking(val != 0);
        break;
    }
    default:
        return false;
    }
    return true;
}

bool FrieshDecoder::adjParam(int p, char *ptr)
{
    if (_handler == nullptr)
        return false;

    switch (p)
    {
    case PARAM_PAN:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        // noInterrupts();
        // g_settings.panOffset+=val;
        // interrupts();
        _handler->adjPanOffset(val);
        break;
    }
    case PARAM_TILT:
    {
        int32_t val = strtol(ptr, &ptr, 10);
        if (*ptr != 0)
            return false;
        // noInterrupts();
        // g_settings.tiltOffset+=val;
        // interrupts();
        _handler->adjTiltOffset(val);
        break;
    }
    default:
        return false;
    }
    return true;
}

/**
 * Get Parameter ID based on its name.
 */
int getParamID(const char *str, char **endptr)
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
    r = PARAM_LAST;
    for (t = 0; t < PARAM_LAST; ++t)
    {
        if (strncmp(PARAM_NAME[t], str, l) == 0)
        {
            if (r != PARAM_LAST)
            {
                r = PARAM_LAST;
                break;
            }
            else
            {
                r = t;
            }
        }
    }
    if (r == PARAM_LAST)
        return -1;
    return r;
}
