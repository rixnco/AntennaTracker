#include <Arduino.h>
#include "BLEDevice.h"

//--------------------------------------
//              DEFINES
//--------------------------------------

#define LED_PIN     2


#define UP_PIN      15
#define DOWN_PIN    21
#define LEFT_PIN    4
#define RIGHT_PIN   16
#define FIRE2_PIN   17
#define FIRE1_PIN   5

#define JOY_OFF_VALUE ((1<<UP_PIN) | (1<<DOWN_PIN) | (1<<LEFT_PIN) | (1<<RIGHT_PIN) | (1<<FIRE2_PIN) | (1<<FIRE1_PIN))

#define COM1_PIN    18
#define COM2_PIN    19

#define LED1_PIN    25
#define LED2_PIN    12

#define COLOR_OFF     3
#define COLOR_RED     1
#define COLOR_GREEN   2
#define COLOR_ORANGE  3

#define BTN_PIN     23

#define BTN_PRESSED  0
#define BTN_RELEASED 1


//--------------------------------------
//            Constants
//--------------------------------------

// The remote service we wish to connect to. (HID)
static BLEUUID HIDServiceUUID((uint16_t)0x1812);
// Characteristics
static BLEUUID mapCharUUID((uint16_t)0x2A4B);
static BLEUUID reportCharUUID((uint16_t)0x2A4D);
// descriptors
static BLEUUID refDescUUID((uint16_t)0x2908);


//--------------------------------------
//            Prototypes
//--------------------------------------

void IRAM_ATTR joy1Interrupt();
void IRAM_ATTR joy2Interrupt();
void IRAM_ATTR btnInterrupt();
static void notifyCallback(BLERemoteCharacteristic*,uint8_t*,size_t,bool,void*);
class Gamepad;

//--------------------------------------
//            Variables
//--------------------------------------


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


//--------------------------------------
//              Types
//--------------------------------------

class Led {
  public:
    Led(uint32_t pin, uint8_t color) :
      _pin(pin),
      _color(color) {
      };

  uint8_t getColor() {
    return _color;
  }

  void show(bool b=true) {
    GPIO.out_w1tc = (_color << _pin);
    if(b) GPIO.out_w1ts = (_color << _pin);
  }

  private:
    uint32_t _pin;
    uint8_t  _color;
};


class Gamepad : public BLEClientCallbacks {
  public:
    Gamepad(uint32_t pin_up, uint32_t pin_down, uint32_t pin_left, uint32_t pin_right, uint32_t pin_f1, uint32_t pin_f2, Led *pLed, std::string name):
    _client(nullptr),
    _hidService(nullptr),
    _reportMapCharacteristic(nullptr),
    _gamepadReportCharacteristic(nullptr),
    _ledReportCharacteristic(nullptr),
    _pin_up(pin_up),
    _pin_down(pin_down),
    _pin_left(pin_left),
    _pin_right(pin_right),
    _pin_f1(pin_f1),
    _pin_f2(pin_f2),
    _pLed(pLed),
    _name(name)
    {
      _disabled_mask  = (1<<_pin_up);
      _disabled_mask |= (1<<_pin_down);
      _disabled_mask |= (1<<_pin_left);
      _disabled_mask |= (1<<_pin_right);
      _disabled_mask |= (1<<_pin_f1);
      _disabled_mask |= (1<<_pin_f2);

      _enabled_mask= 0;
    };
    ~Gamepad() {};

    inline void enable() {
        if(!isConnected()) return;
        GPIO.out_w1tc= _enabled_mask;
    };

    inline void disable() {
        GPIO.out_w1ts= _disabled_mask;
    };

    void update(bool up, bool down, bool left, bool right, bool f1, bool f2) {
      _enabled_mask  = (up<<_pin_up);
      _enabled_mask |= (down<<_pin_down);
      _enabled_mask |= (left<<_pin_left);
      _enabled_mask |= (right<<_pin_right);
      _enabled_mask |= (f1<<_pin_f1);
      _enabled_mask |= (f2<<_pin_f2);
    };

    void setLed(Led *pLed) {
      _pLed= pLed;
      if(isConnected()) {
        Serial.print(_name.c_str()); Serial.print(": setting color: "); Serial.println(pLed->getColor());
        if(_ledReportCharacteristic!= nullptr) {
          _ledReportCharacteristic->writeValue(_pLed->getColor(), true);
        }        
      }
      show();
    };
    void show(bool b=true) {
      _pLed->show(b && isConnected());
    };

    bool connect(BLEAdvertisedDevice* device) {
      if(_client!=nullptr) {
        Serial.print(_name.c_str()); Serial.println(" already connected ...");
        return false;
      }

      Serial.print(_name.c_str()); Serial.print(" connecting to ");
      Serial.println(device->getAddress().toString().c_str());

      _client= new BLEClient(); //BLEDevice::createClient();
      _client->setClientCallbacks(this);
      // Connect to the remove BLE Server.
      if(!_client->connect(device)) {
        disconnect();
        Serial.println("Connection... Failed");
        return false;
      } 
      Serial.println("Connection...OK");

      // Obtain a reference to the service we are after in the remote BLE server.
      
      Serial.print("Looking up HID service");
      _hidService = _client->getService(HIDServiceUUID);
      if (_hidService == nullptr) {
        Serial.println("...Failed");
        disconnect();
        return false;
      }
      Serial.println("...OK");

      // Obtain a reference to the characteristics in the service of the remote BLE server.
      _gamepadReportCharacteristic= nullptr;
      _ledReportCharacteristic= nullptr;
      for(auto &pair : *_hidService->getCharacteristicsByHandle()) {
        BLERemoteCharacteristic* pChar= pair.second;
        if( pChar->getUUID().equals(mapCharUUID)) {
          Serial.print("- Found reportMap characteristic: ");
          std::string val= pChar->readValue();
          for(char c:val) {
            Serial.print((uint8_t)c, HEX); Serial.print(" ");
          }
          Serial.println();
        }
        if( pChar->getUUID().equals(reportCharUUID)) {
          // Serial.println("- Found report characteristic");
          BLERemoteDescriptor* pDesc= pChar->getDescriptor(refDescUUID);
          if (pDesc != nullptr) {
            // std::string val= pDesc->readValue();
            // Serial.println(val.length());
            uint16_t val= pDesc->readUInt16();
            switch(val) {
              case 0x0101:
                _gamepadReportCharacteristic= pChar;
                Serial.println("- Found gamepad input Report characteristic");
                break;
              case 0x0202:
                _ledReportCharacteristic= pChar;
                Serial.println("- Found led output Report characteristic");
            }
          } else {
            // Serial.println("-- missing report reference descriptor ! ignoring");
          }
        }
      }

      if(_gamepadReportCharacteristic!= nullptr) {
        Serial.print("- Registering for gamepad notifications");
        if(!_gamepadReportCharacteristic->canNotify()) {
          Serial.print("...Failed");
          disconnect();
          return false;
        } else {
          _gamepadReportCharacteristic->registerForNotify(notifyCallback, true, this);
          Serial.println("...OK");
        }
      } else {
        Serial.println("- Missing gamepad input report characteristic");
        disconnect();
        return false;
      }
      if(_ledReportCharacteristic!= nullptr) {
        Serial.print("- Setting led color");
        if(!_ledReportCharacteristic->canWrite()) {
          Serial.print("...Failed");
        } else {
          _ledReportCharacteristic->writeValue(_pLed->getColor(), true);
          Serial.println("...OK");
        }
      } else {
        Serial.println("- Missing led output report characteristic");
      }

      _pLed->show();
      Serial.print(_name.c_str()); Serial.println(" connected");
      return true;
    };
    void disconnect() {
      if(_client==nullptr) return;

      if(_gamepadReportCharacteristic!=nullptr) {
        // Unregister notification's callback.
        //_gamepadReportCharacteristic->registerForNotify(nullptr);
      }
      _hidService= nullptr;
      _reportMapCharacteristic= nullptr;
      _gamepadReportCharacteristic= nullptr;
      _ledReportCharacteristic= nullptr;

      delete _client;
      _client= nullptr;

      _pLed->show(false);
      Serial.print(_name.c_str()); Serial.println(" disconnected");
    };

    inline bool isConnected() {
      return _client!=nullptr;
    };


  void onConnect(BLEClient* pclient) {
    Serial.print(_name.c_str()); Serial.println(" - onConnect");
  }

  void onDisconnect(BLEClient* pclient) {
    Serial.print(_name.c_str()); Serial.println("- onDisconnect");
    disconnect();
  }


  private:
    BLEClient* _client;
    BLERemoteService* _hidService;
    BLERemoteCharacteristic* _reportMapCharacteristic;      
    BLERemoteCharacteristic* _gamepadReportCharacteristic;  // report reference descriptor: report ID = 1, type = Output
    BLERemoteCharacteristic* _ledReportCharacteristic;      // report reference descriptor: report ID = 1, type = Output

    uint32_t _pin_up;
    uint32_t _pin_down;
    uint32_t _pin_left;
    uint32_t _pin_right;
    uint32_t _pin_f1;
    uint32_t _pin_f2;
    Led* _pLed;
    std::string _name;
    uint32_t _disabled_mask;
    uint32_t _enabled_mask;
};








static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify,
  void* param) {
  
  int8_t x= (int8_t)pData[0];
  int8_t y= (int8_t)pData[1];

  bool up   = (y<0);
  bool down = (y>0);
  bool left = (x<0);
  bool right= (x>0);
  bool f1   = (pData[2]&0x01);
  bool f2   = ((pData[2]>>1)&0x01);
  
  // Serial.print(pData[0], HEX); Serial.print(" "); Serial.print(pData[1], HEX); Serial.print(" "); Serial.println(pData[2], HEX);

  if(param!=nullptr)  {
    portENTER_CRITICAL(&mux);
    ((Gamepad*)param)->update(up,down,left,right,f1,f2);
    ((Gamepad*)param)->show(false);
    portEXIT_CRITICAL(&mux);
  }
}


Led RED(LED2_PIN, COLOR_RED);
Led GREEN(LED1_PIN, COLOR_GREEN);

Gamepad joyA(UP_PIN, DOWN_PIN, LEFT_PIN, RIGHT_PIN, FIRE1_PIN, FIRE2_PIN, &GREEN, "GamepadA");
Gamepad joyB(UP_PIN, DOWN_PIN, LEFT_PIN, RIGHT_PIN, FIRE1_PIN, FIRE2_PIN, &RED, "GamepadB");

Gamepad* pJoy1;
Gamepad* pJoy2;

volatile int led_state;
volatile int btn_state;
int last_btn_state;

bool scanning;
bool doConnect;
BLEAdvertisedDevice target;








/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.getName()=="HIDGamepad") {
      target= advertisedDevice;
      doConnect=true;
    }

  } // onResult
}; // MyAdvertisedDeviceCallbacks





void setup() {
    
  Serial.begin(115200);
  Serial.println("Starting HIDGamepad Controller...");

  led_state=1;
  digitalWrite(LED_PIN, led_state);
  pinMode(LED_PIN, OUTPUT);

  GPIO.out_w1tc= COLOR_OFF<<LED1_PIN;
  GPIO.out_w1tc= COLOR_OFF<<LED2_PIN;
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED1_PIN+1, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED2_PIN+1, OUTPUT);

  pJoy1= &joyA;
  pJoy2= &joyB;

  GPIO.out_w1ts= JOY_OFF_VALUE;
  
  pinMode(UP_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(DOWN_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(LEFT_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(RIGHT_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(FIRE2_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(FIRE1_PIN, OUTPUT_OPEN_DRAIN);

  pinMode(COM1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COM1_PIN), joy1Interrupt, CHANGE);
  pinMode(COM2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COM2_PIN), joy2Interrupt, CHANGE);

  last_btn_state= btn_state=BTN_RELEASED;
  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnInterrupt, CHANGE);


  doConnect=false;
  scanning=false;

  BLEDevice::init("");

  if(digitalRead(BTN_PIN)==0) {
    int dev_num = esp_ble_get_bond_device_num();
    Serial.print("Removing bond data: "); Serial.println(dev_num);
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }
    free(dev_list);    
  }

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

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

int loop_counter=0;
// This is the Arduino main loop function.
void loop() {

  if(last_btn_state!=btn_state && btn_state==BTN_RELEASED) {
    Serial.println("Swaping gamepads");
    if(pJoy1->isConnected() || pJoy2->isConnected()) {
      portENTER_CRITICAL(&mux);
      Gamepad* pTemp= pJoy1;
      pJoy1=pJoy2;
      pJoy2=pTemp;
      portEXIT_CRITICAL(&mux);
      pJoy1->setLed(&GREEN);
      pJoy2->setLed(&RED);
    }
  }
  last_btn_state= btn_state;

  if(doConnect) {
    doConnect=false;
    scanning=false;
    BLEDevice::getScan()->stop();    
    if(!joyA.isConnected()) {
      Serial.println("Connecting Gamepad A");
      if(joyA.connect(&target)) {
        Serial.println("Connection OK");
      } else {
        Serial.println("Connection Failed");
      }
    } else if(!joyB.isConnected()) {
      Serial.println("Connecting Gamepad B");
      if(joyB.connect(&target)) {
        Serial.println("Connection OK");
      } else {
        Serial.println("Connection Failed");
      }
    }
  }


  ++loop_counter;
  loop_counter= loop_counter%((joyA.isConnected()||joyB.isConnected())?30:15);
  if(loop_counter%5==0) {
    joyA.show();
    joyB.show();
  }

  if (!joyA.isConnected() || !joyB.isConnected() ) {
    if(loop_counter==0) {
      led_state=!led_state;
    }
    if(!scanning) {
      scanning= true;
      Serial.println("Start scanning...");
      BLEDevice::getScan()->start(-1,nullptr, false);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
  }


  digitalWrite(LED_PIN, led_state);
  delay(10); // Delay a second between loops.
} // End of loop

void IRAM_ATTR btnInterrupt() {
  static unsigned long last_interrupt_time=0;
  unsigned long interrupt_time= millis();
  if (interrupt_time - last_interrupt_time > 10) 
  {
    btn_state= digitalRead(BTN_PIN);
  }
  last_interrupt_time = interrupt_time;
  
}

void IRAM_ATTR joy1Interrupt() {
  if(digitalRead(COM1_PIN)) {
    pJoy1->disable();
  } else {
    portENTER_CRITICAL_ISR(&mux);
    pJoy1->enable();
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR joy2Interrupt() {
  if(digitalRead(COM2_PIN)) {
    pJoy2->disable();
  } else {
    portENTER_CRITICAL_ISR(&mux);
    pJoy2->enable();
    portEXIT_CRITICAL_ISR(&mux);
  }
}