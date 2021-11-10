#ifndef __DATALINK_H__
#define __DATALINK_H__

#include <vector>
#include <stdint.h>
#include <stddef.h>
#include <ProtocolDecoder.h>


class DataLink;

class LinkListener {
public:
    virtual ~LinkListener();
    virtual void onLinkConnected(DataLink* link);
    virtual void onLinkDisconnected(DataLink* link);
    virtual void onDataReceived(DataLink* link, uint8_t data) = 0;
};


class DataLink {
public:
    DataLink();
    virtual ~DataLink();

    virtual void setLinkListener(LinkListener* plistener);
protected:
    virtual void fireLinkConnectedEvent();
    virtual void fireLinkDisconnectedEvent();
    virtual void fireDataReceivedEvent(uint8_t data);
    virtual void fireDataReceivedEvent(const uint8_t *pData, size_t len);
private:
    LinkListener *_listener;
};


class DataHandler : public  LinkListener {
public:
    DataHandler();
    DataHandler(size_t size);
    virtual ~DataHandler();

    void addDecoder(ProtocolDecoder* decoder);

    virtual void onDataReceived(DataLink* link, uint8_t data);

private:
    std::vector<ProtocolDecoder*> _decoders;
};





#endif // __DATALINK_H__