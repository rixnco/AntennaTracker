#include "Arduino.h"

#include "BLEFrskyDataSource.h"

// The remote service we wish to connect to. (Frsky)
const BLEUUID FRSKY_SERVICE_UUID((uint16_t)0xFFF0);
// Characteristics
const BLEUUID FRSKY_CHARACTERISTIC_UUID((uint16_t)0xFFF6);

void blefrskydatasource_notifyCallback(BLERemoteCharacteristic *, uint8_t *, size_t, bool, void *);

BLEFrskyDataSource::BLEFrskyDataSource() : _client(nullptr),
                                   _frskyService(nullptr),
                                   _frskyCharacteristic(nullptr)
{
};

BLEFrskyDataSource::~BLEFrskyDataSource()
{
};

bool BLEFrskyDataSource::connect(BLEAdvertisedDevice *device)
{
    if (_client != nullptr)
    {
        Serial.println("already connected ...");
        return false;
    }

    Serial.print("BLEFrskyProvider(");
    Serial.print(device->getAddress().toString().c_str());
    Serial.println(")");

    _client = new BLEClient(); //BLEDevice::createClient();
    _client->setClientCallbacks(this);
    // Connect to the remove BLE Server.
    if (!_client->connect(device))
    {
        close();
        Serial.println("Connection... Failed");
        return false;
    }
    Serial.println("Connection...OK");

    // Obtain a reference to the service we are after in the remote BLE server.

    Serial.print("Looking up Frsky service");
    _frskyService = _client->getService(FRSKY_SERVICE_UUID);
    if (_frskyService == nullptr)
    {
        Serial.println("...Failed");
        close();
        return false;
    }
    Serial.println("...OK");

    // Obtain a reference to the characteristics in the service of the remote BLE server.
    _frskyCharacteristic = nullptr;
    Serial.print("Looking up frsky characteristic");
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
        Serial.println("...OK");
        Serial.print("Registering for notifications");
        if (!_frskyCharacteristic->canNotify())
        {
            Serial.println("...Failed");
            close();
            return false;
        }
        else
        {
            _frskyCharacteristic->registerForNotify(blefrskydatasource_notifyCallback, true, this);
            Serial.println("...OK");
        }
    }
    else
    {
        Serial.println("...Failed");
        close();
        return false;
    }
    return true;
};

void BLEFrskyDataSource::close()
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

    Serial.println("closed");
};

void BLEFrskyDataSource::onConnect(BLEClient *pclient)
{
    Serial.println("- onConnect");
}

void BLEFrskyDataSource::onDisconnect(BLEClient *pclient)
{
    if (_client == nullptr)
        return;
    Serial.println("- onDisconnect");
    close();
}

void BLEFrskyDataSource::received(const uint8_t *pData, size_t len)
{
    doNotify(pData, len);
    // for (int t = 0; t < len; ++t)
    // {
    //     Serial.print(*pData++, HEX);
    //     Serial.print(' ');
    // }
    // Serial.println();
};

void blefrskydatasource_notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify,
    void *param)
{

    if (param != nullptr)
    {
        ((BLEFrskyDataSource *)param)->received(pData, length);
    }
}
