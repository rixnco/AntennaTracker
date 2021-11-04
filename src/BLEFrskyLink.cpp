#include <Arduino.h>
#include "BLEFrskyLink.h"

// The remote service we wish to connect to. (Frsky)
const BLEUUID FRSKY_SERVICE_UUID((uint16_t)0xFFF0);
// Characteristics
const BLEUUID FRSKY_CHARACTERISTIC_UUID((uint16_t)0xFFF6);

BLEFrskyLink::BLEFrskyLink() : _client(nullptr),
                                   _frskyService(nullptr),
                                   _frskyCharacteristic(nullptr),
                                   _listener(nullptr)
{
};

BLEFrskyLink::~BLEFrskyLink()
{
};

bool BLEFrskyLink::connect(BLEAdvertisedDevice *device)
{
    uint64_t addr = (*(uint64_t*)device->getAddress().getNative()) & 0x0000FFFFFFFFFFFF;
    return connect(addr);
}
bool BLEFrskyLink::connect(uint64_t address)
{
    if (_client != nullptr)
    {
        // Serial.println("already connected ...");
        return false;
    }

    // Serial.print("BLEFrskyProvider(");
    // Serial.print(device->getAddress().toString().c_str());
    // Serial.println(")");

    BLEAddress addr = BLEAddress((uint8_t*)&address);
    _client = new BLEClient(); //BLEDevice::createClient();
    _client->setClientCallbacks(this);
    // Connect to the remove BLE Server.
    if (!_client->connect(addr))
    {
        close();
        // Serial.println("Connection... Failed");
        return false;
    }
    // Serial.println("Connection...OK");

    // Obtain a reference to the service we are after in the remote BLE server.

    // Serial.print("Looking up Frsky service");
    _frskyService = _client->getService(FRSKY_SERVICE_UUID);
    if (_frskyService == nullptr)
    {
        // Serial.println("...Failed");
        close();
        return false;
    }
    // Serial.println("...OK");

    // Obtain a reference to the characteristics in the service of the remote BLE server.
    _frskyCharacteristic = nullptr;
    // Serial.print("Looking up frsky characteristic");
    for (auto &pair : *_frskyService->getCharacteristicsByHandle())
    {
        BLERemoteCharacteristic *pChar = pair.second;
        //Serial.println(pChar->getUUID().toString().c_str());
        if (pChar->getUUID().equals(FRSKY_CHARACTERISTIC_UUID))
        {
            _frskyCharacteristic = pChar;
            break;
        }
    }

    if (_frskyCharacteristic != nullptr)
    {
        // Serial.println("...OK");
        // Serial.print("Registering for notifications");
        if (!_frskyCharacteristic->canNotify())
        {
            // Serial.println("...Failed");
            close();
            return false;
        }
        else
        {
            _frskyCharacteristic->registerForNotify(notifyCallback, true, true, this);
            // Serial.println("...OK");
        }
    }
    else
    {
        // Serial.println("...Failed");
        close();
        return false;
    }

    return true;
};

void BLEFrskyLink::close()
{
    if (_client == nullptr)
        return;

    if (_frskyCharacteristic != nullptr)
    {
        // Unregister notification's callback.
        //_notifyCharacteristic->registerForNotify(nullptr);
    }
    _frskyService = nullptr;
    _frskyCharacteristic = nullptr;

    BLEClient *tmp = _client;
    _client = nullptr;
    delete tmp;
};


void BLEFrskyLink::setLinkListener(LinkListener* plistener) 
{
    DataLink::setLinkListener(plistener);
    if(isConnected())
    {
        fireLinkConnectedEvent();
    }
};


void BLEFrskyLink::onConnect(BLEClient *pclient)
{
    fireLinkConnectedEvent();
};

void BLEFrskyLink::onDisconnect(BLEClient *pclient)
{
    if (_client == nullptr)
        return;
    close();
    fireLinkDisconnectedEvent();
};

void BLEFrskyLink::received(const uint8_t *pData, size_t len)
{
    fireDataReceivedEvent(pData, len);
};

void BLEFrskyLink::notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify,
    void *param)
{

    if (param != nullptr)
    {
        ((BLEFrskyLink *)param)->received(pData, length);
    }
}
