#include "DataLink.h"



LinkListener::~LinkListener() {}

void LinkListener::onLinkConnected(DataLink* link) {}
void LinkListener::onLinkDisconnected(DataLink* link) {}
void LinkListener::onDataReceived(DataLink* link, uint8_t data) {}

DataLink::DataLink() : _listener(nullptr) {
}

DataLink::~DataLink() {
}

void DataLink::setLinkListener(LinkListener* plistener) {
    _listener= plistener;
}
void DataLink::fireConnectEvent() {
    if(_listener==nullptr) return;
    _listener->onLinkConnected(this);
}

void DataLink::fireDisconnectEvent() {
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

