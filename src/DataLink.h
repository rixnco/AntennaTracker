#ifndef __DATALINK_H__
#define __DATALINK_H__

#include <stdint.h>
#include <stddef.h>

class DataLink;

class LinkListener {
public:
    virtual ~LinkListener();
    virtual void onLinkConnected(DataLink* link);
    virtual void onLinkDisconnected(DataLink* link);
    virtual void onDataReceived(DataLink* link, uint8_t data);
};


class DataLink {
public:
    DataLink();
    virtual ~DataLink();

    virtual void close() = 0;
    virtual bool isConnected() = 0;

    virtual void setLinkListener(LinkListener* plistener);
protected:
    virtual void fireConnectEvent();
    virtual void fireDisconnectEvent();
    virtual void fireDataReceivedEvent(uint8_t data);
    virtual void fireDataReceivedEvent(const uint8_t *pData, size_t len);
private:
    LinkListener *_listener;
};






#endif // __DATALINK_H__