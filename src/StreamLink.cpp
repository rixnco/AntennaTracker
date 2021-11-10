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


int StreamLink::available() {
    if(_stream==nullptr) return 0;
    return _stream->available();

}

void StreamLink::process()
{
    if(_stream==nullptr) return;
    if(_stream->available()) fireDataReceivedEvent(_stream->read());
}

