
#include "DataSource.h"


DataListener::~DataListener() {
}



DataSource::DataSource() : _listener(nullptr) 
{ 
}

DataSource::~DataSource()
{
    _listener = nullptr;
}


void DataSource::setDataListener(DataListener* listener) {
    _listener = listener;
}

void DataSource::doNotify(const uint8_t* data, size_t length) {
    if(_listener==nullptr) return;

    for(int t=0; t<length; ++t) {
        _listener->onDataReceived(*data++);
    }
}
