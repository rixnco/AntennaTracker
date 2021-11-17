#ifndef __BLE_REMOTE_FRSKY_STREAM_H__
#define __BLE_REMOTE_FRSKY_STREAM_H__

#include <Stream.h>
#include <BLEDevice.h>



// The remote service we wish to connect to. (Frsky)
extern const BLEUUID FRSKY_STREAM_SERVICE_UUID;
// Characteristics
extern const BLEUUID FRSKY_STREAM_RX_CHARACTERISTIC_UUID;

#define BLE_FRSKY_STREAM_RX_BUFFER_SIZE    64


class BLERemoteFrskyStream : public Stream {
public:
    BLERemoteFrskyStream();
    ~BLERemoteFrskyStream();

    bool setFrskyService(BLERemoteService *pService);

    // Stream
    inline int available() { return _rxlen; };
    int read() override;
    int peek() override;
    void flush() override;

    // Print  (NOT IMPLEMENTED/SUPPORTED)
    inline size_t write(uint8_t) { return 0; };

protected:
    static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify, void *param);

    void received(const uint8_t *pData, size_t len);

private:
    BLERemoteService        *_frskyService;
    BLERemoteCharacteristic *_frskyCharacteristic;

    FreeRTOS::Semaphore _rxSemaphore   = FreeRTOS::Semaphore("RX");
    uint8_t  _rxbuffer[BLE_FRSKY_STREAM_RX_BUFFER_SIZE];
    uint16_t _rxhead;
    uint16_t _rxtail;
    uint16_t _rxlen;
};

#endif //__BLE_REMOTE_FRSKY_STREAM_H__