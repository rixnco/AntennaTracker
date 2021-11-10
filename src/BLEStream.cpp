#include <BLE2902.h>

#include "BLEStream.h"

BLEStream::BLEStream(BLEServer *pServer) : rxhead(0), rxtail(0), rxlen(0), pService(nullptr), pRXChar(nullptr), pTXChar(nullptr)
{
  // Create the BLE Stream Service
  pService = pServer->createService(FRIESH_SERVICE_UUID);

  // Create the BLE Stream RX Characteristic
  pRXChar = pService->createCharacteristic(
      FRIESH_RX_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);

  // Create the BLE Stream TX Characteristic
  pTXChar = pService->createCharacteristic(
      FRIESH_TX_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pTXChar->addDescriptor(new BLE2902());

  semaphoreWrite.give();

  pRXChar->setCallbacks(this);
  pTXChar->setCallbacks(this);

  // Start the service
  pService->start();
}

BLEStream::~BLEStream() {
  // TODO clean resources !!!
}

int BLEStream::available()
{
  return rxlen;
}
int BLEStream::read()
{
  int c = -1;
  semaphoreRead.take();
  if (rxlen)
  {
    c = rxbuffer[rxtail];
    rxlen -= 1;
    rxtail = (rxtail + 1) % RX_BUFFER_LEN;
  }
  semaphoreRead.give();
  return c;
}

int BLEStream::peek()
{
  int c = -1;
  semaphoreRead.take();
  if (rxlen)
  {
    c = rxbuffer[rxtail];
  }
  semaphoreRead.give();
  return c;
}
void BLEStream::flush()
{
}

size_t BLEStream::write(uint8_t c)
{
  return write(&c, 1);
}

size_t BLEStream::write(const uint8_t *buffer, size_t len)
{
  int offset = 0;
  int mtu = FRIESH_MTU;
  while (len > 0)
  {
    // Serial.println("waiting sem");
    semaphoreWrite.take();
    int l = len > mtu ? mtu : len;
    pTXChar->setValue((uint8_t *)&buffer[offset], l);
    pTXChar->notify(true);
    offset += l;
    len -= l;
  }
  return len;
}

void BLEStream::onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param)
{

  uint16_t len = param->write.len;
  uint8_t *pvalue = param->write.value;
  //    Serial.println((char*)pvalue);
  semaphoreRead.take();
  for (int t = 0; t < len; ++t)
  {
    rxbuffer[rxhead] = pvalue[t];
    rxhead = (rxhead + 1) % RX_BUFFER_LEN;
    if (rxlen >= RX_BUFFER_LEN)
    {
      // overflow !!!
      rxtail = (rxtail + 1) % RX_BUFFER_LEN;
    }
    else
    {
      rxlen += 1;
    }
  }
  semaphoreRead.give();
}

void BLEStream::onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code)
{
  // Will be called on notification success or error.
  // Serial.println("giving sem");
  semaphoreWrite.give();
}
