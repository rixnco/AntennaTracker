#ifndef __BLE_FRSKY_DATA_SOURCE_H__
#define __BLE_FRSKY_DATA_SOURCE_H__

#include <BLEDevice.h>
#include "DataSource.h"


// The remote service we wish to connect to. (Frsky)
extern const BLEUUID FRSKY_SERVICE_UUID;
// Characteristics
extern const BLEUUID FRSKY_CHARACTERISTIC_UUID;


class BLEFrskyDataSource : public BLEClientCallbacks, public DataSource
{
public:
    BLEFrskyDataSource();
    ~BLEFrskyDataSource();

    bool connect(BLEAdvertisedDevice *device);
    virtual void close();

    inline bool isConnected()
    {
        return _client != nullptr;
    };

    void onConnect(BLEClient *pclient);
    void onDisconnect(BLEClient *pclient);

protected:
    friend void blefrskydatasource_notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify, void *param);

    void received(const uint8_t *pData, size_t len);

private:
    BLEClient *_client;
    BLERemoteService *_frskyService;
    BLERemoteCharacteristic *_frskyCharacteristic; 
};

#endif // __BLE_FRSKY_DATA_SOURCE_H__