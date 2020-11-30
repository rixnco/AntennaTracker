#include <Arduino.h>

#include "BLEFrskyLink.h"
#include <SPortDecoder.h>
#include <CRSFDecoder.h>



//--------------------------------------
//              DEFINES
//--------------------------------------



//--------------------------------------
//              Types
//--------------------------------------

class TelemetryHandler : public TelemetryListener, public LinkListener {
public:
    TelemetryHandler() {
        reset();
    }

    void setLink(DataLink* dataLink) {
        if(_link != nullptr) {
            _link->setLinkListener(nullptr);
        }
        reset();
        _link = dataLink;
        if(_link!= nullptr) {
            _link->setLinkListener(this);
            if(_link->isConnected()) onLinkConnected(_link);
        }
    }

    bool isProtocolDetected() {
        return _decoder!= nullptr;
    }

    virtual void onLinkConnected(DataLink* link) {
        reset();
        Serial.println("Link connected...\nDetecting protocol...");
    }
    virtual void onLinkDisconnected(DataLink* link) {
        Serial.println("Link disconnected...");
    }

    virtual void onDataReceived(DataLink* link, uint8_t data) {
        if(_decoder != nullptr) {
            _decoder->process(data);
        } else {
            for(auto it : _counter) {
                it.first->process(data);
                if(it.second>=10) {
                    _decoder = it.first;
                    Serial.print("Protocol detected: ");
                    Serial.println(_decoder->getName().c_str());
                    break;
                }
            }
        }
    }

    virtual void onFrameDecoded(TelemetryDecoder* decoder, uint32_t id)                         { if(_decoder==nullptr) _counter[decoder]+=1; }
    virtual void onFrameError(TelemetryDecoder* decoder, TelemetryError error, uint32_t param)  { if(_decoder!=decoder) return; Serial.printf("Frame error: %d - 0x%X\n", error, param); }
    virtual void onFuelData(TelemetryDecoder* decoder, int fuel)                                { if(_decoder!=decoder) return; Serial.printf("Fuel       : %d\n", fuel); }
    virtual void onGPSData(TelemetryDecoder* decoder, double latitude, double longitude)        { if(_decoder!=decoder) return; Serial.printf("GPS        : %lf, %lf\n", latitude, longitude); }
    virtual void onVBATData(TelemetryDecoder* decoder, float voltage)                           { if(_decoder!=decoder) return; Serial.printf("VBat       : %.2fV\n", voltage); }
    virtual void onCellVoltageData(TelemetryDecoder* decoder, float voltage)                    { if(_decoder!=decoder) return; Serial.printf("Cell       : %.2fV\n", voltage); }
    virtual void onCurrentData(TelemetryDecoder* decoder, float current)                        { if(_decoder!=decoder) return; Serial.printf("Current    : %.2fA\n", current); }
    virtual void onHeadingData(TelemetryDecoder* decoder, float heading)                        { if(_decoder!=decoder) return; Serial.printf("Heading    : %.2f°\n", heading); }
    virtual void onRSSIData(TelemetryDecoder* decoder, int rssi)                                { if(_decoder!=decoder) return; Serial.printf("Rssi       : %ddB\n", rssi); }
    virtual void onRxBtData(TelemetryDecoder* decoder, float voltage)                           { if(_decoder!=decoder) return; Serial.printf("RxBt       : %.2fV\n", voltage); }
    virtual void onGPSStateData(TelemetryDecoder* decoder, int satellites, bool gpsFix)         { if(_decoder!=decoder) return; Serial.printf("GPSState   : %d, %d\n", satellites, gpsFix); }
    virtual void onVSpeedData(TelemetryDecoder* decoder, float vspeed)                          { if(_decoder!=decoder) return; Serial.printf("VSpeed     : %.2fm/s\n", vspeed); }
    virtual void onAltitudeData(TelemetryDecoder* decoder, float altitude)                      { if(_decoder!=decoder) return; Serial.printf("Altitude   : %.2fm\n", altitude); }
    virtual void onGPSAltitudeData(TelemetryDecoder* decoder, float altitude)                   { if(_decoder!=decoder) return; Serial.printf("GAlt       : %.2fm\n", altitude); }
    virtual void onDistanceData(TelemetryDecoder* decoder, int distance)                        { if(_decoder!=decoder) return; Serial.printf("Distance   : %.2dm\n", distance); }
    virtual void onRollData(TelemetryDecoder* decoder, float rollAngle)                         { if(_decoder!=decoder) return; Serial.printf("Roll       : %.2f°\n", rollAngle); }
    virtual void onPitchData(TelemetryDecoder* decoder, float pitchAngle)                       { if(_decoder!=decoder) return; Serial.printf("Pitch      : %.2f°\n", pitchAngle); }
    virtual void onGSpeedData(TelemetryDecoder* decoder, float speed)                           { if(_decoder!=decoder) return; Serial.printf("GSpeed     : %.2fm/s\n", speed); }
    virtual void onAirSpeedData(TelemetryDecoder* decoder, float speed)                         { if(_decoder!=decoder) return; Serial.printf("AirSpeed   : %.2fm/s\n", speed); }

private:
    void reset() {
        _decoder = nullptr;
        _counter[&_sportDecoder]= 0;
        _counter[&_crsfDecoder] = 0;

        _crsfDecoder.reset();
        _crsfDecoder.setTelemetryListener(this);
        _sportDecoder.reset();
        _sportDecoder.setTelemetryListener(this);
    }



    DataLink                        *_link;
    TelemetryDecoder                *_decoder;

    CRSFDecoder                     _crsfDecoder;
    SPortDecoder                    _sportDecoder;
    std::map<TelemetryDecoder*,int> _counter;
};



//--------------------------------------
//            Constants
//--------------------------------------

//--------------------------------------
//            Prototypes
//--------------------------------------

//--------------------------------------
//            Variables
//--------------------------------------

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

bool scanning;
bool doConnect;
BLEAdvertisedDevice target;

BLEFrskyLink                dataLink;
TelemetryHandler            telemetryHandler;






/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.isAdvertisingService(FRSKY_SERVICE_UUID)) {
        Serial.println("BLEFrskyClient found");
        target= advertisedDevice;
        doConnect=true;
    }

  } // onResult
}; // MyAdvertisedDeviceCallbacks





void setup() {
    
  Serial.begin(115200);
  Serial.println("Starting Antenna Tracker");

  doConnect=false;
  scanning=false;

    telemetryHandler.setLink(&dataLink);
//    telemetryHandler.setDecoder(&sportDecoder);
    // telemetryHandler.setDecoder(&crsfDecoder);

  BLEDevice::init("");

//   if(digitalRead(BTN_PIN)==0) {
//     int dev_num = esp_ble_get_bond_device_num();
//     Serial.print("Removing bond data: "); Serial.println(dev_num);
//     esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
//     esp_ble_get_bond_device_list(&dev_num, dev_list);
//     for (int i = 0; i < dev_num; i++) {
//         esp_ble_remove_bond_device(dev_list[i].bd_addr);
//     }
//     free(dev_list);    
//   }

//   BLESecurity *pSecurity = new BLESecurity();
//   pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
//   pSecurity->setCapability(ESP_IO_CAP_NONE);
//   pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

//   BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);



  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);

  Serial.println("Started...");
  
} // End of setup.

void loop() {

  if(doConnect) {
    doConnect=false;
    scanning=false;
    BLEDevice::getScan()->stop();    
    if(!dataLink.isConnected()) {
      Serial.println("Connecting to BLEFrskyClient ");
      if(!dataLink.connect(&target)) {
        Serial.println("Connection Failed");
      }
    }
  }

  if (!dataLink.isConnected() ) {
    if(!scanning) {
      scanning= true;
      Serial.println("Start scanning...");
      BLEDevice::getScan()->start(-1,nullptr, false);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
  }

  delay(10); // Delay between loops.
} // End of loop

