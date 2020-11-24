#include <Arduino.h>
#include "BLEFrskyDataSource.h"
#include "SPortDecoder.h"

//--------------------------------------
//              DEFINES
//--------------------------------------



//--------------------------------------
//              Types
//--------------------------------------

class Telemetry : public DecoderListener, DataListener {
public:
    void setDataSource(DataSource* dataSource) {
        if(_dataSource != nullptr) {
            _dataSource->setDataListener(nullptr);
        }
        _dataSource = dataSource;
        if(_dataSource!= nullptr) {
            _dataSource->setDataListener(this);
        }
    }

    void setDecoder(Decoder* decoder) {
        if(_decoder!= nullptr) {
            _decoder->setDecoderListener(nullptr);
        }
        _decoder = decoder;
        if(_decoder != nullptr) {
            _decoder->setDecoderListener(this);
        }
    }

     virtual void onDataReceived(uint8_t data) {
        //  Serial.print(data, HEX);
        //  Serial.print(' ');
         if(_decoder != nullptr) {
             _decoder->process(data);
         }
     }


   virtual void onFuelData(int fuel)                            { Serial.printf("Fuel       : %d\n", fuel); }
    virtual void onGPSData(double latitude, double longitude)   { Serial.printf("GPS        : %lf, %lf\n", latitude, longitude); }
    virtual void onVBATData(float voltage)                      { Serial.printf("VBat       : %.2fV\n", voltage); }
    virtual void onCellVoltageData(float voltage)               { Serial.printf("Cell       : %.2fV\n", voltage); }
    virtual void onCurrentData(float current)                   { Serial.printf("Current    : %.2fA\n", current); }
    virtual void onHeadingData(float heading)                   { Serial.printf("Heading    : %.2f°\n", heading); }
    virtual void onRSSIData(int rssi)                           { Serial.printf("Rssi       : %ddB\n", rssi); }
    virtual void onGPSStateData(int satellites, bool gpsFix)    { Serial.printf("GPSState   : %d, %d\n", satellites, gpsFix); }
    virtual void onVSpeedData(float vspeed)                     { Serial.printf("VSpeed     : %.2fm/s\n", vspeed); }
    virtual void onAltitudeData(float altitude)                 { Serial.printf("Altitude   : %.2fm\n", altitude); }
    virtual void onGPSAltitudeData(float altitude)              { Serial.printf("GAlt       : %.2fm\n", altitude); }
    virtual void onDistanceData(int distance)                   { Serial.printf("Distance   : %.2dm\n", distance); }
    virtual void onRollData(float rollAngle)                    { Serial.printf("Roll       : %.2f°\n", rollAngle); }
    virtual void onPitchData(float pitchAngle)                  { Serial.printf("Pitch      : %.2f°\n", pitchAngle); }
    virtual void onGSpeedData(float speed)                      { Serial.printf("GSpeed     : %.2fm/s\n", speed); }
    virtual void onAirSpeedData(float speed)                    { Serial.printf("AirSpeed   : %.2fm/s\n", speed); }

private:
    DataSource* _dataSource;
    Decoder*    _decoder;
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

BLEFrskyDataSource  dataSource;
SPortDecoder        sportDecoder;
Telemetry           telemetry;




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

    telemetry.setDataSource(&dataSource);
    telemetry.setDecoder(&sportDecoder);

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
    if(!dataSource.isConnected()) {
      Serial.println("Connecting to BLEFrskyClient ");
      if(dataSource.connect(&target)) {
        Serial.println("Connected");
        sportDecoder.reset();

      } else {
        Serial.println("Connection Failed");
      }
    }
  }

  if (!dataSource.isConnected() ) {
    if(!scanning) {
      scanning= true;
      Serial.println("Start scanning...");
      BLEDevice::getScan()->start(-1,nullptr, false);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
  }

  delay(10); // Delay between loops.
} // End of loop

