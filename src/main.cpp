#include <Arduino.h>


#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <ESP32Servo.h>
#include <TelemetryDecoder.h>
#include <SPortDecoder.h>
#include <CRSFDecoder.h>
#include <ESP32Stepper.h>
#include <GeoUtils.h>
#include <QMC5883LCompass.h>
#include <HMC5883LCompass.h>
#include <FrieshDecoder.h>

#include "DataLink.h"
#include "BLEFrskyLink.h"
#include "StreamLink.h"

//--------------------------------------
//              DEFINES
//--------------------------------------


#define BTN_PIN                 4
#define LED1_PIN                2

#define SERVO_PIN               25
#define STEPPER_DIR_PIN         26
#define STEPPER_STEP_PIN        27
#define STEPPER_ENA_PIN         14
#define STEPPER_STEP_PER_REV    (32 * 64 * 8)

#define EEPROM_SIZE             256

//--------------------------------------
//              Types
//--------------------------------------

#define SETTINGS_MAGIC          0xDEADBEEF
#define SETTINGS_VERSION        100

typedef struct {
    uint32_t    magic;
    uint32_t    version;
    bool        compassCalibrated;
    int32_t     compassMinX; 
    int32_t     compassMaxX; 
    int32_t     compassMinY; 
    int32_t     compassMaxY; 
    uint64_t    bleRemoteAddress;
    bool        homed;
    float       homeLattitude;
    float       homeLongitude;
    float       homeElevation;
} settings_t;

//-----------------------------------------------------------------------------------------
// Telemetry stuff
//-----------------------------------------------------------------------------------------
class TelemetryHandler : public TelemetryListener
{
    // GPS Telemetry ...
    virtual void onGPSData(TelemetryDecoder *decoder, float latitude, float longitude) override;
    virtual void onGPSAltitudeData(TelemetryDecoder *decoder, float altitude) override;
    virtual void onGPSStateData(TelemetryDecoder *decoder, int satellites, bool gpsFix) override;
    virtual void onGSpeedData(TelemetryDecoder *decoder, float speed) override;

    // Optional ...
    virtual void onFrameDecoded(TelemetryDecoder *decoder, uint32_t id) override;
    virtual void onFrameError(TelemetryDecoder *decoder, TelemetryError error, uint32_t param) override;
    virtual void onFuelData(TelemetryDecoder *decoder, int fuel) override;
    virtual void onVBATData(TelemetryDecoder *decoder, float voltage) override;
    virtual void onCellVoltageData(TelemetryDecoder *decoder, float voltage) override;
    virtual void onCurrentData(TelemetryDecoder *decoder, float current) override;
    virtual void onHeadingData(TelemetryDecoder *decoder, float heading) override;
    virtual void onRSSIData(TelemetryDecoder *decoder, int rssi) override;
    virtual void onRxBtData(TelemetryDecoder *decoder, float voltage) override;
    virtual void onVSpeedData(TelemetryDecoder *decoder, float vspeed) override;
    virtual void onAltitudeData(TelemetryDecoder *decoder, float altitude) override;
    virtual void onDistanceData(TelemetryDecoder *decoder, int distance) override;
    virtual void onRollData(TelemetryDecoder *decoder, float rollAngle) override;
    virtual void onPitchData(TelemetryDecoder *decoder, float pitchAngle) override;
    virtual void onAirSpeedData(TelemetryDecoder *decoder, float speed) override;
};

//-----------------------------------------------------------------------------------------
// Friesh protocol stuff
//-----------------------------------------------------------------------------------------
class FrieshHandler : public FrieshListener
{
public:
    virtual void onFrameError(FrieshDecoder *decoder, FrieshError error, uint32_t param) override;
    virtual void onCalibrateCompass(FrieshDecoder *decoder) override;
    virtual void onSetHomeLocation(FrieshDecoder *decoder) override;
    virtual void onStartTracking(FrieshDecoder *decoder) override;
    virtual void onStopTracking(FrieshDecoder *decoder) override;
};


//-----------------------------------------------------------------------------------------
// BLEDataHandler stuff
//-----------------------------------------------------------------------------------------

class BLEDataHandler : public DataHandler {
public:
    BLEDataHandler();
    BLEDataHandler(size_t size);
    virtual void onLinkConnected(DataLink* link) override;
    virtual void onLinkDisconnected(DataLink* link) override;
};


//-----------------------------------------------------------------------------------------
// BLE stuff
//-----------------------------------------------------------------------------------------

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    /**
   * Called for each advertising BLE server.
   */
    void onResult(BLEAdvertisedDevice advertisedDevice) override;
};    // MyAdvertisedDeviceCallbacks



//-----------------------------------------------------------------------------------------
// State machine stuff
//-----------------------------------------------------------------------------------------

class State
{
public:
    virtual ~State() {}
    virtual void enter(){}
    virtual State *run() = 0;
    virtual void exit(){}
};


class StartupState : public State {
public:
    virtual void enter() override;
    virtual State *run() override;
private:
    bool _pressed;
    bool _long_press;
};

class IdleState : public State {
public:
    virtual void enter() override;
    virtual State *run() override;
private:
};


class CalibrationState : public State {
public:
    virtual void enter() override;
    virtual State *run() override;
private:
    int      _loops;
    float    _speed;
    float    _angle;
    uint64_t _lastMeasureTime;
};

class ConnectionState : public State {
public:
    virtual void enter() override;
    virtual State *run() override;
    virtual void exit() override;
private:
};

class HomeState : public State {
public:
    virtual void enter() override;
    virtual State *run() override;
private:
    uint64_t _lastProcessTime;
};

class TrackingState : public State {
public:
    virtual void enter() override;
    virtual State *run() override;
    virtual void exit() override;
private:
    uint64_t _lastProcessTime;
};


StartupState        startupState;
IdleState           idleState;
CalibrationState    calibrationState;
ConnectionState     connectionState;
HomeState           homeState;
TrackingState       trackingState;

State*      _state = &startupState;
State*      _lastState = nullptr;;



//--------------------------------------
//            Constants
//--------------------------------------

//--------------------------------------
//            Prototypes
//--------------------------------------

float getHeadingError(float current_heading, float target_heading);
bool wire_ping(uint8_t addr);



settings_t g_settings;

void storeSettings();
void resetSettings();
bool loadSettings();


//--------------------------------------
//            Variables
//--------------------------------------



uint64_t g_remoteAddress = 0x0000000000000000;
bool    g_connected = false;;
GeoPt   g_homeLocation;

bool    g_gpsFix = false;
int     g_gpsSatellites;
GeoPt   g_gpsTarget;






CRSFDecoder         bleCRSFDecoder;
SPortDecoder        bleSPortDecoder;
BLEDataHandler      bleDataHandler(2);
BLEFrskyLink        bleLink;

FrieshDecoder       serialFrieshDecoder;
CRSFDecoder         serialCRSFDecoder;
SPortDecoder        serialSPortDecoder;
DataHandler         serialDataHandler(3);
StreamLink          serialLink;

TelemetryHandler    telemetryHandler;
FrieshHandler       frieshHandler;

ESP32Stepper        stepper;
Servo               tiltServo;
QMC5883LCompass     qmc5883;
HMC5883LCompass     hmc5883;
Compass             *compass;
//LiquidCrystal_I2C   lcd(0x27); 
LiquidCrystal_I2C   lcd(PCF8574_ADDR_A21_A11_A01);


// BUTTON Stuff
#define BTN_LONG_PRESS_TIME   3000
#define BTN_DEBOUNCE_MS         40

void btnInit();
void IRAM_ATTR  btnChangedISR();
bool            isButtonPressed();
uint64_t        getButtonPressTime();
void            resetButtonPressTime();

portMUX_TYPE      _btnMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t _btn_state_time;
volatile bool     _btn_state;


// LED Stuff
void ledInit();
void IRAM_ATTR onLedTimer();
void ledState(bool state);
void ledBlink(uint16_t* profile, size_t len);

hw_timer_t *        _led_timer;
portMUX_TYPE        _led_timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile size_t     _led_profile_idx;
volatile size_t     _led_profile_len;
volatile uint16_t   *_led_profile;

#define LED_ON   (0x8000)
#define LED_OFF  (0x0000)
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

uint16_t    g_startup_profile[] = { 
    LED_ON  |  100,
    LED_OFF | 1400
};

uint16_t    g_connection_profile[] = { 
    LED_ON  |  100,
    LED_OFF |  100,
    LED_ON  |  100,
    LED_OFF | 1200
};

uint16_t    g_fix_profile[] = { 
    LED_ON  |  100,
    LED_OFF |  100,
    LED_ON  | 1200,
    LED_OFF |  100
};

uint16_t    g_tracking_profile[] = { 
    LED_ON  |  300,
    LED_OFF |  300,
};



void setup()
{
    EEPROM.begin(EEPROM_SIZE);

    Serial.begin(115200);
    Serial.println("Starting Antenna Tracker");

    btnInit();
    ledInit();

    pinMode(STEPPER_ENA_PIN, OUTPUT);
    digitalWrite(STEPPER_ENA_PIN, LOW);

    if(loadSettings())
    {
        Serial.println("settings restored...");
    } 
    else 
    {
        Serial.println("Using default settings...");
    }

    Serial.print("Configuring BLE link");
    bleCRSFDecoder.setTelemetryListener(&telemetryHandler);
    bleSPortDecoder.setTelemetryListener(&telemetryHandler);
    bleDataHandler.addDecoder(&bleCRSFDecoder);
    bleDataHandler.addDecoder(&bleSPortDecoder);
    bleLink.setLinkListener(&bleDataHandler);
    Serial.println("...OK");

    Serial.print("Configuring serial link");
    serialFrieshDecoder.setCheckCRC(false);
    serialFrieshDecoder.setFrieshListener(&frieshHandler);
    serialCRSFDecoder.setTelemetryListener(&telemetryHandler);
    serialSPortDecoder.setTelemetryListener(&telemetryHandler);
    serialDataHandler.addDecoder(&serialFrieshDecoder);
    serialDataHandler.addDecoder(&serialCRSFDecoder);
    serialDataHandler.addDecoder(&serialSPortDecoder);
    serialLink.setLinkListener(&serialDataHandler);
    serialLink.setStream(&Serial2);
    Serial2.begin(115200);
    Serial.println("...OK");

    Serial.print("Configuring BLE scanner");
    BLEDevice::init("");

    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device.  Specify that we want active scanning and start the
    // scan to run for 5 seconds.
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

    g_connected= false;
    Serial.println("...OK");


    Serial.print("Configuring stepper...");
    if (!stepper.attach(STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_STEP_PER_REV))
    {
        Serial.println("...FAIL");
        while (true)
            ;
    }
    Serial.println("...OK");

    Serial.print("Configuring servo...");
    if (!tiltServo.attach(SERVO_PIN))
    {
        Serial.println("...FAIL");
        while (true)
            ;
    }
    Serial.println("...OK");
    tiltServo.write(20);

    Serial.println("Checking compass...");
    Wire.begin();

    if (wire_ping(0x1E))
    {
        Serial.println("Found HMC5883");
        hmc5883.init();
        hmc5883.setMode(HMC5883_MODE_CONTINUOUS, HMC5883_ODR_75_HZ, HMC5883_RNG_1_30_GA, HMC5883_OSR_8);
        compass = &hmc5883;
    }
    else
    {
        if (wire_ping(0x0D))
        {
            Serial.println("Found QMC5883");
            qmc5883.init();
            qmc5883.setMode(QMC5883_MODE_CONTINUOUS, QMC5883_ODR_10_HZ, QMC5883_RNG_2_GA, QMC5883_OSR_64);
            compass = &qmc5883;
        }
        else
        {
            Serial.println("Initializing Compass...FAIL");
            while (true)
                ;
        }
    }
    if(g_settings.compassCalibrated) {
        compass->setCalibration(g_settings.compassMinX, g_settings.compassMaxX, g_settings.compassMinY, g_settings.compassMaxY, -2000, 2000);
        Serial.println("Restored compass calibration");
    }

    Serial.println("Initializing Compass...OK");

    if(g_settings.bleRemoteAddress!=0) {
        Serial.println("Restored BLE Remote address");
    }

    if(g_settings.homed) {
        Serial.println("Restored home location");
        g_homeLocation= GeoPt(g_settings.homeLattitude, g_settings.homeLongitude, g_settings.homeElevation);        
    }


    // Scan I2C bus
    // for(int t=1; t<127; ++t) {
    //     if(wire_ping(t)) Serial.printf("Found device at address: %02x\n", t);
    // }

    lcd.begin(16,2);               // initialize the lcd 
    lcd.setCursor(0, 0);
    lcd.print("AntennaTracker");

    Serial.println("Started...");
} // End of setup.

void loop()
{
    if(_state==nullptr) {
        Serial.println("State machine terminated !!!");
        while(true);
    }
    if(_state != _lastState) {
        if(_lastState!=nullptr) _lastState->exit();
        _state->enter();
        _lastState = _state;
    }
    _state = _state->run();
    
    serialLink.process();
}

// Settings Stuff
void resetSettings() {
    memset(&g_settings, 0, sizeof(g_settings));
    g_settings.magic   = SETTINGS_MAGIC;
    g_settings.version = SETTINGS_VERSION;
    storeSettings();
}

void storeSettings() {
    EEPROM.put<settings_t>(0, g_settings);
    EEPROM.commit();
    Serial.println(">>> settings stored");
}

bool loadSettings() {
    uint32_t magic;
    EEPROM.get<uint32_t>(0, magic);
    if(magic== SETTINGS_MAGIC) 
    {
        // We got some settings stored in EEPROM
        uint32_t version;
        EEPROM.get<uint32_t>(4, version);
        if(version==SETTINGS_VERSION)
        {
            // Just read settings
            EEPROM.get<settings_t>(0, g_settings);
            return true;
        }
        else 
        {
            // Need to convert stored settings to our version
            // For now just reset settings to our version's default.
        }
    }
    resetSettings();
    
    return false;
}


// BUTTON Stuff
void btnInit() 
{
    pinMode(BTN_PIN, INPUT_PULLUP);
    _btn_state = false;
    attachInterrupt(BTN_PIN, btnChangedISR, CHANGE);
}

void IRAM_ATTR  btnChangedISR() {
    uint64_t now= millis();

    portENTER_CRITICAL_ISR(&_btnMux);
    _btn_state_time=now;
    _btn_state= !digitalRead(BTN_PIN);
    portEXIT_CRITICAL_ISR(&_btnMux);
}

bool isButtonPressed() {
    bool res;
    portENTER_CRITICAL(&_btnMux);
    res = _btn_state && (millis()-_btn_state_time>BTN_DEBOUNCE_MS);
    portEXIT_CRITICAL(&_btnMux);
     return res; 
}

uint64_t getButtonPressTime() 
{
    uint64_t time=0;
    portENTER_CRITICAL(&_btnMux);
    if(_btn_state) time= millis()-_btn_state_time;
    portEXIT_CRITICAL(&_btnMux);
    return time;
}

void resetButtonPressTime()
{
    portENTER_CRITICAL(&_btnMux);
    if(_btn_state) _btn_state_time= millis();
    portEXIT_CRITICAL(&_btnMux);

}


void ledInit()
{
    pinMode(LED1_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    _led_timer = timerBegin(0, 80, true);
    /* Attach onTimer function to our timer */
    timerAttachInterrupt(_led_timer, &onLedTimer, true);
}

void IRAM_ATTR onLedTimer() {
    portENTER_CRITICAL_ISR(&_led_timerMux);
    _led_profile_idx = (_led_profile_idx+1) % _led_profile_len;
    digitalWrite(LED1_PIN, (_led_profile[_led_profile_idx]&LED_ON) != 0);
    timerAlarmWrite(_led_timer, 1000 * (_led_profile[_led_profile_idx]&(~LED_ON)), true);
    portEXIT_CRITICAL_ISR(&_led_timerMux);    
}

void ledState(bool state) 
{
    portENTER_CRITICAL(&_led_timerMux);
    timerAlarmDisable(_led_timer);
    timerStop(_led_timer);
    digitalWrite(LED1_PIN, state);
    portEXIT_CRITICAL(&_led_timerMux);    
}

void ledBlink(uint16_t* profile, size_t len)
{
    portENTER_CRITICAL(&_led_timerMux);
    timerStop(_led_timer);
    _led_profile= profile;
    _led_profile_len= len;
    _led_profile_idx = 0;
    digitalWrite(LED1_PIN, (_led_profile[_led_profile_idx]&LED_ON) != 0);
    timerAlarmWrite(_led_timer, 1000 * (_led_profile[_led_profile_idx]&(~LED_ON)), true);
    timerWrite(_led_timer, 0);
    timerAlarmEnable(_led_timer);
    timerStart(_led_timer);
    portEXIT_CRITICAL(&_led_timerMux);    
}


void StartupState::enter()
{
    // AUTO Calibration
    Serial.println("StartupState::enter");

    resetButtonPressTime();
    _pressed    = false;
    _long_press = false;    

    ledBlink(g_startup_profile, ARRAY_LEN(g_startup_profile));
    lcd.clear();
    lcd.print("Click to start");
    lcd.setCursor(0,1);
    lcd.print("Hold to reset");
}

State *StartupState::run()
{
    static uint64_t lastTime=0;

    if(millis() - lastTime < 30) return this;
    lastTime= millis();


    if(!isButtonPressed()) 
    {
        if(_long_press) {
            resetSettings();
        }
        if(_pressed) return &idleState;
        return this;
    }
    _pressed = true;
    if(getButtonPressTime()>BTN_LONG_PRESS_TIME) {
        ledState(LED_ON);
        _long_press = true;
    }

    return this;
}


void IdleState::enter()
{
    // AUTO Calibration
    Serial.println("IdleState::enter");
    ledState(LED_OFF);
}
State *IdleState::run()
{
    if(!g_settings.compassCalibrated) {
        return &calibrationState;
    }
    if(!g_connected) {
        return &connectionState;
    }
    if(!g_settings.homed) {
        return &homeState;
    }
    return &trackingState;
}

void CalibrationState::enter()
{
    Serial.println("Calibration::enter");
    g_settings.compassCalibrated = false;
    g_settings.compassMinX = 999999;
    g_settings.compassMaxX = -999999;
    g_settings.compassMinY = 999999;
    g_settings.compassMaxY = -999999;

    _loops = 2;
    _speed = 45;
    _angle = -390;
    _lastMeasureTime = 0;

    ledState(LED_OFF);
    lcd.setCursor(0,0);
    lcd.print("Calibrating");
    lcd.setCursor(0,1);
    lcd.print("...please wait");
}

State *CalibrationState::run()
{
    if(!stepper.isMoving()) {
        if(_loops==0) 
        {
            // Now we have finish spinning
            Serial.println("Done");
            Serial.print("compass.setCalibration(");
            Serial.print(g_settings.compassMinX);
            Serial.print(", ");
            Serial.print(g_settings.compassMaxX);
            Serial.print(", ");
            Serial.print(g_settings.compassMinY);
            Serial.print(", ");
            Serial.print(g_settings.compassMaxY);
            Serial.println(");");

            compass->setCalibration(g_settings.compassMinX, g_settings.compassMaxX, g_settings.compassMinY, g_settings.compassMaxY, -2000, 2000);
            g_settings.compassCalibrated = true;
            storeSettings();
            return &idleState;
        }
        --_loops;
        _angle = -_angle;
        stepper.move(_speed, _angle);
    }
    if(millis()-_lastMeasureTime < 100) return this;
    _lastMeasureTime = millis();

    compass->read();
    bool changed = false;
    // Return XYZ readings
    float x = compass->getX();
    float y = compass->getY();
    if (x < g_settings.compassMinX)
    {
        g_settings.compassMinX = x;
        changed = true;
    }
    if (x > g_settings.compassMaxX)
    {
        g_settings.compassMaxX = x;
        changed = true;
    }

    if (y < g_settings.compassMinY)
    {
        g_settings.compassMinY = y;
        changed = true;
    }
    if (y > g_settings.compassMaxY)
    {
        g_settings.compassMaxY = y;
        changed = true;
    }

    if (changed)
    {
        Serial.println("CALIBRATING...");
    }

    return this;

}


void ConnectionState::enter()
{
    Serial.println("ConnectionState::enter");
    // if(g_settings.bleRemoteAddress==0) {
        Serial.println("Start scanning...");
        g_remoteAddress=0;
        BLEDevice::getScan()->start(-1, nullptr, false); // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    // }

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connection...");
    ledBlink(g_connection_profile,ARRAY_LEN(g_connection_profile));

}

State *ConnectionState::run()
{
    if(g_connected)
    {
        Serial.println("ConnectionState::run -> connected...");
        return &idleState;
    }
    if(g_remoteAddress!=0) 
    {
        BLEDevice::getScan()->stop();

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Connecting");
        lcd.setCursor(0,1);
        lcd.printf("> %02X%02X%02X%02X%02X%02X", 
            (uint8_t)(g_remoteAddress>>40),
            (uint8_t)(g_remoteAddress>>32),
            (uint8_t)(g_remoteAddress>>24),
            (uint8_t)(g_remoteAddress>>16),
            (uint8_t)(g_remoteAddress>> 8),
            (uint8_t)(g_remoteAddress    ));
        Serial.printf("connecting to %02X:%02X:%02X:%02X:%02X:%02X\n", 
            (uint8_t)(g_remoteAddress>>40),
            (uint8_t)(g_remoteAddress>>32),
            (uint8_t)(g_remoteAddress>>24),
            (uint8_t)(g_remoteAddress>>16),
            (uint8_t)(g_remoteAddress>> 8),
            (uint8_t)(g_remoteAddress    ));
        
        
        if(bleLink.connect(g_remoteAddress)) {
            g_connected = true;
            Serial.println("Connection...OK");
            g_settings.bleRemoteAddress= g_remoteAddress;
            storeSettings();
        } else {
            Serial.println("Connection...FAIL");
            g_remoteAddress = 0;
            Serial.println("Start scanning...");
            BLEDevice::getScan()->start(-1, nullptr, false); // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
        }

    }

    return this;
}

void ConnectionState::exit() {
    BLEDevice::getScan()->stop();
}

void HomeState::enter() 
{
    Serial.println("HomeState::enter");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Home position");
    lcd.setCursor(0,1);
    lcd.print("Waiting fix...");
    ledBlink(g_fix_profile, ARRAY_LEN(g_fix_profile));
}

State *HomeState::run() 
{
    static bool lastFix= true;
    static GeoPt lastTarget(0,0);

    if(!g_connected) return &idleState;

    if(!g_gpsFix && lastFix) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Home position");
        lcd.setCursor(0,1);
        lcd.print("Waiting fix...");
    }
    if(!g_gpsFix) {
        lastFix = g_gpsFix;
        return this;
    } 

    // We have a fix !!!
    ledState(LED_ON);

    if(lastTarget != g_gpsTarget) {
        lastTarget= g_gpsTarget;
        lcd.clear();
        lcd.print("btn to set HOME");
        lcd.setCursor(0,1);
        lcd.printf("%.4f , %.4f", lastTarget.getLatitude(), lastTarget.getLongitude());
    }

    if(isButtonPressed()) 
    {
        g_homeLocation= g_gpsTarget;
        g_settings.homed= true;
        g_settings.homeLattitude= g_homeLocation.getLatitude();
        g_settings.homeLongitude= g_homeLocation.getLongitude();
        g_settings.homeElevation= g_homeLocation.getElevation();
        storeSettings();
        return &idleState;
    }
    return this;
}


void TrackingState::enter() 
{
    Serial.println("TrackingState::enter");
    _lastProcessTime = 0;
    lcd.clear();
    lcd.print("Tracking:");
    ledBlink(g_tracking_profile, sizeof(g_tracking_profile)/sizeof(g_tracking_profile[0]));
}

State *TrackingState::run() 
{
    if(!g_settings.compassCalibrated) return &idleState;
    if(!g_settings.homed) return &idleState;
    if(!g_connected) return &idleState;
    
    if(!g_gpsFix) return this;

    static int cnt= 0;

    if(millis() - _lastProcessTime < 100) return this;
    _lastProcessTime=millis();
    
    if(g_homeLocation.distanceTo(g_gpsTarget)< 20) {
        if(stepper.isMoving()) {
            stepper.stop();
            lcd.clear();
            lcd.print("Tracking:");
            lcd.setCursor(0,1);
            lcd.print("too close...");
        }
        return this;
    }

    compass->read();

    // Return Azimuth reading
    int a = compass->getAzimuth();
    float target = g_homeLocation.azimuthTo(g_gpsTarget);
    while(target<0) target+=360;
    while(target>=360) target-=360;
    float error = getHeadingError(a, target);
    float speed = error * 2;

    if ( cnt == 0 ) {
        lcd.setCursor(0,0);
        lcd.printf("Tracking: %.1f   ", target);
        lcd.setCursor(0,1);
        lcd.printf("azimuth %d     ", a);

        Serial.printf("azimuth=%d   target=%.1f   error=%.1f  speed: %.1f\n", a, target, error, speed);
    }
    cnt = (cnt+1) % 10;

    float dir = speed >= 0 ? DIR_CW : DIR_CCW;
    speed = fabs(speed);
    if (speed > 90)
    {
        speed = 90;
    }
    stepper.move(speed, dir);

    return this;
}

void TrackingState::exit()
{
    stepper.stop();
}














bool wire_ping(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

float getHeadingError(float current_heading, float target_heading)
{
    float errorDeg;
    float rawError = target_heading - current_heading;
    if (rawError < -180.)
    {
        errorDeg = -rawError - 180;
        return errorDeg;
    }
    else if (rawError > 180.)
    {
        errorDeg = -rawError + 180;
        return errorDeg;
    }
    else
    {
        return rawError;
    }
}

// TELEMETRY HANDLING

void TelemetryHandler::onGPSData(TelemetryDecoder *decoder, float latitude, float longitude)
{
    g_gpsFix   = true;
    g_gpsTarget= GeoPt(latitude, longitude);
    Serial.printf("GPS        : %lf, %lf\n", latitude, longitude);
}
void TelemetryHandler::onGPSStateData(TelemetryDecoder *decoder, int satellites, bool gpsFix)
{
    g_gpsFix = gpsFix;
    g_gpsSatellites = satellites;
    Serial.printf("GPSState   : %d, %d\n", satellites, gpsFix);
}
void TelemetryHandler::onGPSAltitudeData(TelemetryDecoder *decoder, float altitude)
{
    Serial.printf("GAlt       : %.2fm\n", altitude);
}
void TelemetryHandler::onGSpeedData(TelemetryDecoder *decoder, float speed)
{
    Serial.printf("GSpeed     : %.2fm/s\n", speed);
}




void TelemetryHandler::onFrameDecoded(TelemetryDecoder *decoder, uint32_t id)
{
    // Do nothing
//    Serial.printf("received %02X\n", id);
}
void TelemetryHandler::onFrameError(TelemetryDecoder *decoder, TelemetryError error, uint32_t param)
{
//    Serial.printf("Frame error: %d - 0x%X\n", error, param);
}
void TelemetryHandler::onFuelData(TelemetryDecoder *decoder, int fuel)
{
    // Serial.printf("Fuel       : %d\n", fuel);
}
void TelemetryHandler::onVBATData(TelemetryDecoder *decoder, float voltage)
{
    // Serial.printf("VBat       : %.2fV\n", voltage);
}
void TelemetryHandler::onCellVoltageData(TelemetryDecoder *decoder, float voltage)
{
    // Serial.printf("Cell       : %.2fV\n", voltage);
}
void TelemetryHandler::onCurrentData(TelemetryDecoder *decoder, float current)
{
    // Serial.printf("Current    : %.2fA\n", current);
}
void TelemetryHandler::onHeadingData(TelemetryDecoder *decoder, float heading)
{
    // Serial.printf("Heading    : %.2f°\n", heading);
}
void TelemetryHandler::onRSSIData(TelemetryDecoder *decoder, int rssi)
{
    // Serial.printf("Rssi       : %ddB\n", rssi);
}
void TelemetryHandler::onRxBtData(TelemetryDecoder *decoder, float voltage)
{
    // Serial.printf("RxBt       : %.2fV\n", voltage);
}
void TelemetryHandler::onVSpeedData(TelemetryDecoder *decoder, float vspeed)
{
    // Serial.printf("VSpeed     : %.2fm/s\n", vspeed);
}
void TelemetryHandler::onAltitudeData(TelemetryDecoder *decoder, float altitude)
{
    // Serial.printf("Altitude   : %.2fm\n", altitude);
}
void TelemetryHandler::onDistanceData(TelemetryDecoder *decoder, int distance)
{
    // Serial.printf("Distance   : %.2dm\n", distance);
}
void TelemetryHandler::onRollData(TelemetryDecoder *decoder, float rollAngle)
{
    // Serial.printf("Roll       : %.2f°\n", rollAngle);
}
void TelemetryHandler::onPitchData(TelemetryDecoder *decoder, float pitchAngle)
{
    // Serial.printf("Pitch      : %.2f°\n", pitchAngle);
}
void TelemetryHandler::onAirSpeedData(TelemetryDecoder *decoder, float speed)
{
    // Serial.printf("AirSpeed   : %.2fm/s\n", speed);
}


// FRIESH PROTOCOL


void FrieshHandler::onFrameError(FrieshDecoder *decoder, FrieshError error, uint32_t param)
{
    Serial.printf("FRIESH: Frame Error: 0x%02X [%d]\n", error, param);
}
void FrieshHandler::onCalibrateCompass(FrieshDecoder *decoder)
{
    Serial.printf("FRIESH: calibrate compass...");
}
void FrieshHandler::onSetHomeLocation(FrieshDecoder *decoder)
{
    Serial.printf("FRIESH: set Home location...");
}
void FrieshHandler::onStartTracking(FrieshDecoder *decoder)
{
    Serial.printf("FRIESH: start tracking...");
}
void FrieshHandler::onStopTracking(FrieshDecoder *decoder)
{
    Serial.printf("FRIESH: stop tracking...");
}


//-----------------------------------------------------------------------------------------
// BLE DataHandler stuff
//-----------------------------------------------------------------------------------------

BLEDataHandler::BLEDataHandler() : DataHandler() {}
BLEDataHandler::BLEDataHandler(size_t size) : DataHandler(size) {}

void BLEDataHandler::onLinkConnected(DataLink* link) 
{
}

void BLEDataHandler::onLinkDisconnected(DataLink* link)
{
    g_connected = false;
}



/// BLE

/**
 * Called for each advertising BLE server.
 */
void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice)
{
    Serial.print("BLE Advertised Device: ");
    Serial.println(advertisedDevice.toString().c_str());
    uint64_t addr = *(uint64_t*)(advertisedDevice.getAddress().getNative()) & 0x0000FFFFFFFFFFFF;

    // We have found a device, let us now see if it contains the service we are looking for.
    if ( (g_settings.bleRemoteAddress==addr) ||
         ((g_settings.bleRemoteAddress==0) && advertisedDevice.isAdvertisingService(FRSKY_SERVICE_UUID)))
    {
        Serial.println("Binding to BLEFrskyClient");
        g_remoteAddress= addr;
    }
}







