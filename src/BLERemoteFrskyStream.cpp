#include <Arduino.h>
#include "BLERemoteFrskyStream.h"

// The remote service we wish to connect to. (Frsky)
const BLEUUID FRSKY_STREAM_SERVICE_UUID((uint16_t)0xFFF0);
// Characteristics
const BLEUUID FRSKY_STREAM_RX_CHARACTERISTIC_UUID((uint16_t)0xFFF6);

BLERemoteFrskyStream::BLERemoteFrskyStream() : 
    _frskyService(nullptr),
    _frskyCharacteristic(nullptr),
    _rxhead(0),
    _rxtail(0),
    _rxlen(0)
{
    memset((void*)_rxbuffer, 0, BLE_FRSKY_STREAM_RX_BUFFER_SIZE);
};

BLERemoteFrskyStream::~BLERemoteFrskyStream()
{
    if(_frskyCharacteristic) _frskyCharacteristic->registerForNotify(nullptr);
};

bool BLERemoteFrskyStream::setFrskyService(BLERemoteService *pService)
{

    if (pService == nullptr)
    {
        // Disconnecting from client
        Serial.println("Unregistering notifications");
        if(_frskyCharacteristic && _frskyCharacteristic->getRemoteService()->getClient()->isConnected()) _frskyCharacteristic->registerForNotify(nullptr);
        _frskyCharacteristic = nullptr;
        _frskyService = nullptr;
        return true;
    }
    Serial.println("Registering for notifications");
    _frskyService = pService;
    // Obtain a reference to the characteristics in the service of the remote BLE server.
    _frskyCharacteristic = nullptr;
    // Serial.print("Looking up frsky characteristic");
    for (auto &pair : *_frskyService->getCharacteristicsByHandle())
    {
        BLERemoteCharacteristic *pChar = pair.second;
        //Serial.println(pChar->getUUID().toString().c_str());
        if (pChar->getUUID().equals(FRSKY_STREAM_RX_CHARACTERISTIC_UUID))
        {
            Serial.println("Found frsky RX characteristic");
            _frskyCharacteristic = pChar;
            break;
        }
    }

    bool res = _frskyCharacteristic != nullptr && _frskyCharacteristic->canNotify();
 
    if(!res) 
    {
        _frskyService = nullptr;
        _frskyCharacteristic = nullptr;
    } 
    else 
    {
        Serial.println("registered for notifications");
        flush();
        _frskyCharacteristic->registerForNotify(notifyCallback, true, true, this);
    }

    return res;
};



int BLERemoteFrskyStream::read()
{
  int c = -1;
  _rxSemaphore.take();
  if (_rxlen)
  {
    c = _rxbuffer[_rxtail];
    _rxlen -= 1;
    _rxtail = (_rxtail + 1) % BLE_FRSKY_STREAM_RX_BUFFER_SIZE;
  }
  _rxSemaphore.give();
  return c;
}

int BLERemoteFrskyStream::peek()
{
  int c = -1;
  _rxSemaphore.take();
  if (_rxlen)
  {
    c = _rxbuffer[_rxtail];
  }
  _rxSemaphore.give();
  return c;
}

void BLERemoteFrskyStream::flush() 
{ 
    _rxhead=_rxtail=_rxlen=0; 
}


void BLERemoteFrskyStream::received(const uint8_t *pData, size_t len)
{
  //    Serial.println((char*)pvalue);
  _rxSemaphore.take();
  for (int t = 0; t < len; ++t)
  {
    _rxbuffer[_rxhead] = pData[t];
    _rxhead = (_rxhead + 1) % BLE_FRSKY_STREAM_RX_BUFFER_SIZE;
    if (_rxlen >= BLE_FRSKY_STREAM_RX_BUFFER_SIZE)
    {
      // overflow !!!
      _rxtail = (_rxtail + 1) % BLE_FRSKY_STREAM_RX_BUFFER_SIZE;
    }
    else
    {
      _rxlen += 1;
    }
  }
  _rxSemaphore.give();  
};

void BLERemoteFrskyStream::notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify,
    void *param)
{
    if (param != nullptr)
    {
        ((BLERemoteFrskyStream *)param)->received(pData, length);
    }
}
