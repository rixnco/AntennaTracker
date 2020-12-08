#include "DataLink.h"



LinkListener::~LinkListener() {}
void LinkListener::onLinkConnected(DataLink* link) {}
void LinkListener::onLinkDisconnected(DataLink* link) {}


DataLink::DataLink() : _listener(nullptr) {
}

DataLink::~DataLink() {
}

void DataLink::setLinkListener(LinkListener* plistener) {
    _listener= plistener;
}
void DataLink::fireLinkConnectedEvent() {
    if(_listener==nullptr) return;
    _listener->onLinkConnected(this);
}
void DataLink::fireLinkDisconnectedEvent() {
    if(_listener==nullptr) return;
    _listener->onLinkDisconnected(this);
}
void DataLink::fireDataReceivedEvent(uint8_t data) {
    if(_listener==nullptr) return;
    _listener->onDataReceived(this, data);
}
void DataLink::fireDataReceivedEvent(const uint8_t *pData, size_t len) {
    if(_listener==nullptr) return;
    while(len-->0) {
        _listener->onDataReceived(this, *pData++);
    }
}



DataHandler::DataHandler() 
{
}

DataHandler::DataHandler(size_t size) 
{ 
    _decoders.reserve(size); 
}

DataHandler::~DataHandler()
{

}

void DataHandler::addDecoder(DataDecoder* decoder) 
{
    if(decoder==nullptr) return;
    _decoders.push_back(decoder);
}

void DataHandler::onDataReceived(DataLink* link, uint8_t data) 
{
    for(auto decoder : _decoders) {
        decoder->process(data);
    }
}