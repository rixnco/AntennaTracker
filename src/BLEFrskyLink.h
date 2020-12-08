#ifndef __BLE_FRSKY_DATA_SOURCE_H__
#define __BLE_FRSKY_DATA_SOURCE_H__

#include "DataLink.h"
#include <BLEDevice.h>





// The remote service we wish to connect to. (Frsky)
extern const BLEUUID FRSKY_SERVICE_UUID;
// Characteristics
extern const BLEUUID FRSKY_CHARACTERISTIC_UUID;


class BLEFrskyLink : public BLEClientCallbacks, public DataLink {
public:
    BLEFrskyLink();
    ~BLEFrskyLink();

    bool connect(uint64_t address);
    bool connect(BLEAdvertisedDevice *device);
    
    void close();
    inline bool isConnected()
    {
        return _client != nullptr;
    };

    virtual void setLinkListener(LinkListener* plistener) override;

    void onConnect(BLEClient *pclient) override;
    void onDisconnect(BLEClient *pclient) override;

protected:
    static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify, void *param);

    void received(const uint8_t *pData, size_t len);

private:
    BLEClient *_client;
    BLERemoteService *_frskyService;
    BLERemoteCharacteristic *_frskyCharacteristic; 
    LinkListener *_listener;
};

#endif // __BLE_FRSKY_DATA_SOURCE_H__