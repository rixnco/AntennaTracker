#ifndef __BLE_STREAM_H__
#define __BLE_STREAM_H__

#include <Arduino.h>
#include <BLEDevice.h>


#define FRIESH_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define FRIESH_RX_CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define FRIESH_TX_CHARACTERISTIC_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define FRIESH_MTU       20
#define RX_BUFFER_LEN 256


class BLEStream : public Stream, BLECharacteristicCallbacks
{
protected:
    uint8_t rxbuffer[RX_BUFFER_LEN];
    uint16_t rxhead;
    uint16_t rxtail;
    uint16_t rxlen;
    BLEService *pService;
    BLECharacteristic *pRXChar;
    BLECharacteristic *pTXChar;
    FreeRTOS::Semaphore semaphoreRead   = FreeRTOS::Semaphore("Read");
    FreeRTOS::Semaphore semaphoreWrite  = FreeRTOS::Semaphore("Write");
public:
    BLEStream(BLEServer *pServer) ;
    virtual ~BLEStream() ;

    int available() override;
    int read() override;
    int peek() override;
    void flush() override;

    size_t write(uint8_t c) override;

    size_t write(const uint8_t *buffer, size_t len) override;

    void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) override;

    void onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code) override;
};





#endif