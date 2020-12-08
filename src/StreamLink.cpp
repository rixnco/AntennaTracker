#include <Arduino.h>
#include "StreamLink.h"


StreamLink::StreamLink() : _stream(nullptr)
{
};

StreamLink::~StreamLink()
{
}

void StreamLink::setStream(Stream* stream)
{
    _stream= stream;
}


bool StreamLink::available() {
    if(_stream==nullptr) return false;
    return _stream->available();

}

void StreamLink::process()
{
    if(_stream==nullptr) return;
    if(_stream->available()) fireDataReceivedEvent(_stream->read());
}

