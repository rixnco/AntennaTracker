
#include <Arduino.h>

#include <ESP32Servo.h>
#include <TelemetryDecoder.h>
#include <SPortDecoder.h>
#include <CrsfDecoder.h>
// #include <ESP32Stepper.h>
#include <ServoStepper.h>
#include <GeoUtils.h>
#include <FrieshDecoder.h>

#include "bsp.h"
#include "Display.h"
#include "DataLink.h"
#include "BLERemoteFrskyStream.h"
#include "StreamLink.h"
#include "Settings.h"
#include "BLEFrieshStream.h"
#include "AS5600.h"
#include "PID_v1.h"
//--------------------------------------
//              CONFIG
//--------------------------------------

//#define FRIESH_SERVER_ENABLED



//--------------------------------------
//              DEFINES
//--------------------------------------


#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))



//--------------------------------------
//              Types
//--------------------------------------







//-----------------------------------------------------------------------------------------
// Telemetry stuff
//-----------------------------------------------------------------------------------------



class TelemetryManager : public TelemetryProvider, public TelemetryListener
{
public:
    virtual ~TelemetryManager() {};

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

    // Metrics
    inline uint32_t getDecodedFrames() { return _decodedFrames; }
    inline uint32_t getDecodingError() { return _decodingErrors; }

    // Telemetry
    inline GeoPt    getGPS() override { return _gps; }
    inline uint16_t getSatellites() override { return _satellites; }
    inline bool     hasFix() override { return _fix; }
    inline float    getSpeed() override  { return _speed; }
    inline int      getFuel() override { return _fuel; }
    inline float    getVBat() override { return _vbat; }
    inline float    getVCell() override { return _vcell; }
    inline float    getCurrent() override { return _current; }
    inline float    getHeading() override { return _heading; }
    inline int      getRSSI() override { return _rssi; }
    inline float    getRxBt() override { return _rxbt; }
    inline float    getVSpeed() override { return _vspeed; }
    inline float    getAltitude() override { return _altitude; }
    inline int      getDistance() override { return _distance; }
    inline float    getRoll() override { return _roll; }
    inline float    getPitch() override { return _pitch; }
    inline float    getAirSpeed() override { return _airspeed; }

private:
    uint32_t    _decodedFrames;
    uint32_t    _decodingErrors;
    GeoPt       _gps;
    uint16_t    _satellites; 
    bool        _fix;
    float       _galt;
    float       _speed;
    int         _fuel;
    float       _vbat;
    float       _vcell;
    float       _current;
    float       _heading;
    int         _rssi;
    float       _rxbt;
    float       _vspeed;
    float       _altitude;
    int         _distance;
    float       _roll;
    float       _pitch;
    float       _airspeed;

};

//-----------------------------------------------------------------------------------------
// Friesh protocol stuff
//-----------------------------------------------------------------------------------------
// class MyFrieshHandler : public FrieshHandler {
// public:
//     MyFrieshHandler(Telemetry* pTelemetry);
//     virtual ~MyFrieshHandler();

//     virtual void  setHome(float lat, float lon, float elv) override;
//     virtual float getHomeLatitude() override;
//     virtual float getHomeLongitude() override;
//     virtual float getHomeElevation() override;

//     virtual void  setCalib(float lat, float lon, float elv) override;
//     virtual float getCalibLatitude() override;
//     virtual float getCalibLongitude() override;
//     virtual float getCalibElevation() override;

//     virtual void setTracking(bool tracking) override;
//     virtual bool isTracking() override;

//     virtual void setPanOffset(int pan) override;
//     virtual void adjPanOffset(int16_t delta) override;
//     virtual int  getPanOffset() override;

//     virtual void setTiltOffset(int pan) override;
//     virtual void adjTiltOffset(int16_t delta) override;
//     virtual int  getTiltOffset() override;

//     virtual bool storeSettings() override;
//     virtual bool loadSettings() override;
// private:
//     Telemetry *_pTelemetry;
// };

//-----------------------------------------------------------------------------------------
// BLEDataHandler stuff
//-----------------------------------------------------------------------------------------

class BLEDataHandler : public DataHandler
{
public:
    BLEDataHandler();
    BLEDataHandler(size_t size);
    virtual void onLinkConnected(DataLink *link) override;
    virtual void onLinkDisconnected(DataLink *link) override;
};

//-----------------------------------------------------------------------------------------
// BLE Client / Server stuff
//-----------------------------------------------------------------------------------------


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class BLEFrskyConnection : public BLEAdvertisedDeviceCallbacks, BLEClientCallbacks, protected BLEClient
{
public:
    BLEFrskyConnection();

    void onResult(BLEAdvertisedDevice advertisedDevice) override;

    void process();

	void onConnect(BLEClient *pClient) override;
	void onDisconnect(BLEClient *pClient) override;

    inline bool isConnected() { return _connected; }
protected:
    uint64_t   _address;
    bool       _connected;
}; // BLEFrskyClient

#ifdef FRIESH_SERVER_ENABLED
class BLEFrieshConnection : public BLEServerCallbacks
{
public:
    BLEFrieshConnection();

    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override;
    void onDisconnect(BLEServer *pServer) override;

    bool isConnected();
protected:
    bool _connected;
};
#endif


//-----------------------------------------------------------------------------------------
// State machine stuff
//-----------------------------------------------------------------------------------------

class State
{
public:
    virtual ~State() {}
    virtual void enter() {}
    virtual State *run() = 0;
    virtual void exit() {}
};

class StartupState : public State
{
public:
    virtual void enter() override;
    virtual State *run() override;

private:
    bool _pressed;
    bool _long_press;
};

class IdleState : public State
{
public:
    virtual void enter() override;
    virtual State *run() override;

private:
};

class ConfigurationState : public State
{
public:
    virtual void enter() override;
    virtual State *run() override;

private:
};

class TrackingState : public State
{
public:
    virtual void enter() override;
    virtual State *run() override;
    virtual void exit() override;

private:
    uint64_t _lastProcessTime;
};

StartupState startupState;
IdleState idleState;
ConfigurationState configurationState;
TrackingState trackingState;

State *_state = &startupState;
State *_lastState = nullptr;


//--------------------------------------
//            Constants
//--------------------------------------

//--------------------------------------
//            Prototypes
//--------------------------------------

static float getTilt(float distance, float elevation);
static float getAzimuth();
static float getHeadingError(float current_heading, float target_heading);


bool wire_ping(uint8_t addr);


//--------------------------------------
//            Variables
//--------------------------------------

#ifdef FRIESH_SERVER_ENABLED
BLEServer               *pFrieshServer;
BLEFrieshConnection     frieshConnection;
bool                    g_frieshClientConnected = false;
#endif
Stream                  *pFrieshStream;



BLEFrskyConnection      frskyConnection;
bool                    g_frskyConnected = false;
BLERemoteFrskyStream    *pFrskyStream;





TelemetryManager        telemetry;

CRSFDecoder             frskyCRSFDecoder;
SPortDecoder            frskySPortDecoder;
BLEDataHandler          frskyDataHandler(2);
StreamLink              frskyLink;

FrieshDecoder           serialFrieshDecoder(&Settings, &telemetry);
CRSFDecoder             serialCRSFDecoder;
SPortDecoder            serialSPortDecoder;
DataHandler             serialDataHandler(3);
StreamLink              serialLink;

SPortDecoder            frieshSPortDecoder;
CRSFDecoder             frieshCRSFDecoder;
FrieshDecoder           frieshFrieshDecoder(&Settings, &telemetry);
DataHandler             frieshDataHandler(3);
StreamLink              frieshLink;


ServoStepper stepper;
Servo tiltServo;

LCD_Display lcd(0x27);
OLED_Display oled(0x3c, 5);
DisplayProxy display;


double panPidSetpoint, panPidInput, panPidOutput;
double Kp=10., Ki=0., Kd=0.;
double consKp=1., consKi=0., consKd=0.;
PID PanPID = PID(&panPidInput, &panPidOutput, &panPidSetpoint, Kp, Ki, Kd, DIRECT);

AS5600 RotaryEncoder = AS5600();

uint16_t g_startup_profile[] = {
    LED_ON | 100,
    LED_OFF | 1400};

uint16_t g_connection_profile[] = {
    LED_ON | 100,
    LED_OFF | 100,
    LED_ON | 100,
    LED_OFF | 1200};

uint16_t g_fix_profile[] = {
    LED_ON | 100,
    LED_OFF | 100,
    LED_ON | 1200,
    LED_OFF | 100};

uint16_t g_tracking_profile[] = {
    LED_ON | 300,
    LED_OFF | 300,
};


void setup()
{


    Serial.begin(115200);
    Serial.println("Starting Antenna Tracker");

    BLEDevice::init("Friesh");

    btnInit();
    ledInit();
    Settings.init();
    if(!Settings.load())
    {
        Serial.println("Using default settings...");
    }

    Serial.print("Configuring the telemetry link");
    pFrskyStream = new BLERemoteFrskyStream();
   
    frskyCRSFDecoder.setTelemetryListener(&telemetry);
    frskySPortDecoder.setTelemetryListener(&telemetry);
   
    frskyDataHandler.addDecoder(&frskyCRSFDecoder);
    frskyDataHandler.addDecoder(&frskySPortDecoder);
   
    frskyLink.setLinkListener(&frskyDataHandler);
    frskyLink.setStream(pFrskyStream);
    Serial.println("...OK");

    Serial.print("Configuring the serial link");
    serialFrieshDecoder.setOutputStream(&Serial);
    serialCRSFDecoder.setTelemetryListener(&telemetry);
    serialSPortDecoder.setTelemetryListener(&telemetry);

    serialDataHandler.addDecoder(&serialFrieshDecoder);
    serialDataHandler.addDecoder(&serialCRSFDecoder);
    serialDataHandler.addDecoder(&serialSPortDecoder);

    serialLink.setLinkListener(&serialDataHandler);
    serialLink.setStream(&Serial);
    Serial.println("...OK");

    Serial.print("Configuring the friesh link");
    #ifdef FRIESH_SERVER_ENABLED
    // Initializes the Friesh Stream server
    pFrieshServer = BLEDevice::createServer();
    pFrieshServer->setCallbacks(&frieshConnection);
    pFrieshStream= new BLEFrieshStream(pFrieshServer);
    #else
    Serial2.begin(115200);
    pFrieshStream = &Serial2;
    #endif

    frieshFrieshDecoder.setOutputStream(pFrieshStream);
    frieshCRSFDecoder.setTelemetryListener(&telemetry);
    frieshSPortDecoder.setTelemetryListener(&telemetry);
 
    frieshDataHandler.addDecoder(&frieshFrieshDecoder);
    frieshDataHandler.addDecoder(&serialCRSFDecoder);
    frieshDataHandler.addDecoder(&serialSPortDecoder);

    frieshLink.setLinkListener(&frieshDataHandler);
    frieshLink.setStream(pFrieshStream);
    Serial.println("...OK");

    pinMode(SERVO_PIN, OUTPUT);
    Serial.print("Configuring servo...");
    if (!tiltServo.attach(SERVO_PIN))
    {
        Serial.println("...FAIL");
        while (true)
            ;
    }
    Serial.println(tiltServo.attached());
    Serial.println("...OK");

    tiltServo.write(SERVO_ZERO_OFFSET + (SERVO_DIRECTION * 0));

    Serial.print("Configuring stepper...");
    motorPinsInit();
    stepper.init(SERVO_STEPPER_PWM_PIN, SERVO_STEPPER_DIR_PIN, 30000, 3, 8, 32);
    Serial.println("...OK");
    stepper.stop();

    // Scan I2C bus
    Wire.begin();
    Serial.println("Scanning I2C bus...");
    for (int t = 1; t < 127; ++t)
    {
        if (wire_ping(t))
            Serial.printf("Found device at address: %02x\n", t);
    }

    Serial.print("Initializing rotary encoder");
    if(!RotaryEncoder.init())
    {
        Serial.println("...Failed");
    }
    else
    {
        Serial.println("...OK");
    }

    if (wire_ping(0x3c))
    {
        Serial.println("Configuring OLED Display...");
        display.setDisplay(&oled);
    }
    else if (wire_ping(0x27))
    {
        Serial.println("Configuring LCD Display...");
        display.setDisplay(&lcd);
    }
    display.init();
    display.clear();
    display.setCursor(0, 0);
    display.printf("AntennaTracker");
    // display.setCursor(0,1);
    // display.printf("%3d\xF8", getAzimuth());
    display.show();

    // Initialize panPID
    panPidSetpoint = 0;
    panPidInput = 0;
    PanPID.SetOutputLimits(-255.0, 255.0);
    PanPID.SetSampleTime(20);
    PanPID.SetMode(AUTOMATIC);

    // Start advertising
    #ifdef FRIESH_SERVER_ENABLED
    g_frieshClientConnected = false;
    delay(500);
    Serial.print("Advertising the friesh Server");
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(FRIESH_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("...OK");
    #endif

    delay(200);
    g_frskyConnected = false;
    Serial.println("Scanning for a Frsky telemetry connection");
    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device.  Specify that we want active scanning and start the
    // scan to run for 5 seconds.
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&frskyConnection);
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(-1, nullptr, false);

    Serial.println("AntennaTracker started...");

} // End of setup.

void loop()
{
    #ifdef FRIESH_SERVER_ENABLED
    // disconnecting
    if (g_frieshClientConnected && !frieshConnection.isConnected())
    {
        Serial.println("start advertising");
        g_frieshClientConnected = false;
        pFrieshServer->startAdvertising(); // restart advertising
    }
    // connecting
    if (!g_frieshClientConnected && frieshConnection.isConnected())
    {
        Serial.println("Stop advertising");
        // do stuff here on connecting
        BLEDevice::getAdvertising()->stop();
        g_frieshClientConnected = true;
    }
    #endif

    frskyConnection.process();

    frskyLink.process();
    serialLink.process();
    frieshLink.process();



    if (_state == nullptr)
    {
        Serial.println("State machine terminated !!!");
        while (true)
            ;
    }
    if (_state != _lastState)
    {
        if (_lastState != nullptr)
            _lastState->exit();
        _state->enter();
        _lastState = _state;
    }
    _state = _state->run();

}


void StartupState::enter()
{
    // AUTO Calibration
    Serial.println("StartupState::enter");

    resetButtonPressTime();
    _pressed = false;
    _long_press = false;

    ledBlink(g_startup_profile, ARRAY_LEN(g_startup_profile));

    display.clear();
    display.setCursor(0, 0);
    display.print("Click to start");
    display.setCursor(0, 1);
    display.print("Hold to reset");
    display.show();
}

State *StartupState::run()
{
    static uint64_t lastTime = 0;

    if (millis() - lastTime < 30)
        return this;
    lastTime = millis();

    if (!isButtonPressed())
    {
        if (_long_press)
        {
            Settings.reset();
        }
        if (_pressed)
            return &idleState;
        return this;
    }
    _pressed = true;
    if (getButtonPressTime() > BTN_LONG_PRESS_TIME)
    {
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

    return &trackingState;
}


GeoPt getHome() { return Settings.getHome(); }
GeoPt getTarget() { return Settings.getTrackerMode()==MODE_TRACK?telemetry.getGPS():Settings.getCalib(); }


void TrackingState::enter()
{
    Serial.println("TrackingState::enter");
    _lastProcessTime = 0;
    display.clear();
    display.setCursor(0, 0);
    GeoPt home = getHome();
    GeoPt target = getTarget();
    float d = home.distanceTo(target);
    if (d < 20)
    {
        display.print("T: ---.-\xF8 --.--km");
    }
    else
    {
        display.printf("T:% 3.1f\xF8 %2.2fkm", home.azimuthTo(target), d);
    }
    display.setCursor(0, 1);
    display.printf("A:% 3.1f\xF8", getAzimuth());
    display.show();
    ledBlink(g_tracking_profile, sizeof(g_tracking_profile) / sizeof(g_tracking_profile[0]));
}

State *TrackingState::run()
{


    //    if(!g_gpsFix) return this;

    static uint64_t lastDebugTime = 0;
    static uint64_t lastDisplayTime = 0;
    static uint64_t lastADCTime = 0;
    static uint64_t lastButtonTime = 0;
    static uint8_t  screenMode = 0;
    uint64_t now = millis();

    if (isButtonPressed() && now - lastButtonTime > 500)
    {
        screenMode = !screenMode;
        lastButtonTime = now;
    }
    
    if (now - _lastProcessTime < 20)
        return this;

    _lastProcessTime = now;
    GeoPt home = getHome();
    GeoPt target = getTarget();
    
    float target_d = home.distanceTo(target);
    float current_a = getAzimuth();
    static float batt_volts = 0;

    if (now - lastADCTime > 5000)
    {
        int batt_raw = analogRead(ADC_BATT_PIN);
        batt_volts = (float)batt_raw / Settings.getAdcBattFactor();
        lastADCTime = now;
    }
    

    if (target_d < 20)
    {
        if (stepper.isMoving())
        {
            stepper.stop();
            lastDisplayTime = 0;
        }
        if (now - lastDisplayTime >= 200)
        {
            lastDisplayTime = now;
            display.clear();
            display.setCursor(0, 0);
            if (screenMode == 0)
            {
                if (Settings.getTrackerMode() == MODE_CALIB)
                {
                    display.print("cT: ---.-\xF8 --.--km");
                } else
                {
                    display.print("T: ---.-\xF8 --.--km");
                }
                display.setCursor(0, 1);
                display.printf("A:% 3.1f\xF8 --.-\xF8", current_a);
            } else
            {
                display.setCursor(0, 0);
                display.printf("Batt:% 3.1f V", batt_volts);
                bool c = frskyConnection.isConnected();
                if (c)
                {
                    display.setCursor(0, 1);
                    display.printf("Connected to radio");
                }
                
            }
            display.show();
        }
        return this;
    }

    float target_a = home.azimuthTo(target);
    while (target_a < 0)
        target_a += 360;
    while (target_a >= 360)
        target_a -= 360;
    float error = getHeadingError(current_a, target_a);
    if (fabsf(error) < .5)
    {
        PanPID.SetTunings(consKp, consKi, consKd);
    } else if(fabsf(error) > 0.9) {
        PanPID.SetTunings(Kp, Ki, Kd);
    }
    
    panPidInput = (double)error;
    PanPID.Compute();

    float tilt = getTilt(target_d, target.getElevation());
    if (tilt < 0)
        tilt = 0;

    if (now - lastDisplayTime >= 200)
    {
        display.clear();
        display.setCursor(0, 0);
        if (screenMode == 0)
        {
            if (Settings.getTrackerMode() == MODE_CALIB)
            {
                display.printf("cT: %3.1f\xF8 %2.2fkm", target_a, target_d * 0.001);
            } else
            {
                display.printf("T: %3.1f\xF8 %2.2fkm", target_a, target_d * 0.001);
            }
            display.setCursor(0, 1);
            display.printf("A: %3.1f\xF8 %2.1f\xF8", current_a, tilt);
        } else
        {
            display.setCursor(0, 0);
            display.printf("Batt:% 3.1fV", batt_volts);
            bool c = frskyConnection.isConnected();
            if (c)
            {
                display.setCursor(0, 1);
                display.printf("Connected to radio");
            }     
        }
        display.show();

        lastDisplayTime = now;
    }
    if (fabsf(error) < 0.1)
    {
        stepper.stop();
    } else {
        float speed = (float)panPidOutput;
        float dir = 0.;
        if(speed < 0) dir = 1.;
        else dir = 0;
        // Serial.print("error : " + String(error) + "\tspeed : " + String(speed) + "\t\t");
        // Serial.println("speed abs : " + String(speed));
        stepper.move(speed, dir);
    }

    if (tilt < SERVO_MIN)
        tilt = SERVO_MIN;
    if (tilt > SERVO_MAX)
        tilt = SERVO_MAX;
    // TODO Do we adjust the tilt angle like that, or do we do something smarter ???!!!
    tiltServo.write(SERVO_ZERO_OFFSET + (SERVO_DIRECTION * (tilt+Settings.getTiltOffset())));

    if (now - lastDebugTime > 1000)
    {
        lastDebugTime = now;
        Serial.printf("target_d=%2.2fkm  target_a=%.1f°  current_a=%.1f° error=%.1f  tilt: %.1f\n", target_d* 0.001, target_a, current_a, error, tilt);
    }

    return this;
}

void TrackingState::exit()
{
    stepper.stop();
    display.clear();
    display.show();
}

bool wire_ping(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

static float getTilt(float distance, float elevation)
{
    elevation = Settings.getAltitudeMode()==ALT_BARO?elevation:elevation-Settings.getHomeElevation();
    float tilt = atan2(elevation, distance);
    return TO_DEGF(tilt);    

}

static float getAzimuth()
{
    RotaryEncoder.read();
    float azimuth = RotaryEncoder.getAngleDegrees()+Settings.getPanOffset();
    while (azimuth < 0)
    {
        azimuth += 360.;
    }
    return fmod(azimuth, 360);
}

static float getHeadingError(float current_heading, float target_heading)
{
    float errorDeg;
    float rawError = target_heading - current_heading;
    if (rawError < -180.)
    {
        errorDeg = rawError + 360;
        return errorDeg;
    }
    else if (rawError > 180.)
    {
        errorDeg = rawError - 360;
        return errorDeg;
    }
    else
    {
        return rawError;
    }
}

// TELEMETRY HANDLING

uint32_t dbg_tlm_cnt = 0;
void TelemetryManager::onGPSData(TelemetryDecoder *decoder, float latitude, float longitude)
{
    _fix = true;
    _gps = GeoPt(latitude, longitude, _altitude); // estimated altitude (could be gps or baro)
    Serial.printf("[%04d] GPS        : %.4f, %.4f, %.1f\n", dbg_tlm_cnt, _gps.getLatitude(), _gps.getLongitude(), _gps.getElevation());
}
void TelemetryManager::onGPSStateData(TelemetryDecoder *decoder, int satellites, bool gpsFix)
{
    _fix = gpsFix;
    _satellites = satellites;

    //    Serial.printf("GPSState   : %d, %d\n", satellites, gpsFix);
}
void TelemetryManager::onGPSAltitudeData(TelemetryDecoder *decoder, float altitude)
{
    _galt = altitude;
    //    g_GAlt = altitude;
    //    Serial.printf("GAlt       : %.2fm\n", altitude);
}
void TelemetryManager::onGSpeedData(TelemetryDecoder *decoder, float speed)
{
    _speed = speed;
    //    Serial.printf("GSpeed     : %.2fm/s\n", speed);
}

void TelemetryManager::onFrameDecoded(TelemetryDecoder* decoder, uint32_t id) {
//    Serial.printf("[%s] - %d\n", decoder->getName().c_str(), id);
    ++_decodedFrames;
    ++dbg_tlm_cnt;
}

void TelemetryManager::onFrameError(TelemetryDecoder *decoder, TelemetryError error, uint32_t param)
{
    ++_decodingErrors;
//    Serial.printf("[%s] Frame error: %d - 0x%X\n", decoder->getName().c_str(), error, param);
}
void TelemetryManager::onFuelData(TelemetryDecoder *decoder, int fuel)
{
    _fuel = fuel;
    // Serial.printf("Fuel       : %d\n", fuel);
}
void TelemetryManager::onVBATData(TelemetryDecoder *decoder, float voltage)
{
    _vbat = voltage;
    // Serial.printf("VBat       : %.2fV\n", voltage);
}
void TelemetryManager::onCellVoltageData(TelemetryDecoder *decoder, float voltage)
{
    _vcell = voltage;
    // Serial.printf("Cell       : %.2fV\n", voltage);
}
void TelemetryManager::onCurrentData(TelemetryDecoder *decoder, float current)
{
    _current = current;
    // Serial.printf("Current    : %.2fA\n", current);
}
void TelemetryManager::onHeadingData(TelemetryDecoder *decoder, float heading)
{
    _heading = heading;
    // Serial.printf("Heading    : %.2f\xF8\n", heading);
}
void TelemetryManager::onRSSIData(TelemetryDecoder *decoder, int rssi)
{
    _rssi = rssi;
    // Serial.printf("Rssi       : %ddB\n", rssi);
}
void TelemetryManager::onRxBtData(TelemetryDecoder *decoder, float voltage)
{
    _rxbt = voltage;
    // Serial.printf("RxBt       : %.2fV\n", voltage);
}
void TelemetryManager::onVSpeedData(TelemetryDecoder *decoder, float vspeed)
{
    _vspeed= vspeed;
    // Serial.printf("VSpeed     : %.2fm/s\n", vspeed);
}
void TelemetryManager::onAltitudeData(TelemetryDecoder *decoder, float altitude)
{
    _altitude = altitude;
    // Serial.printf("Altitude   : %.2fm\n", altitude);
}
void TelemetryManager::onDistanceData(TelemetryDecoder *decoder, int distance)
{
    _distance = distance;
    // Serial.printf("Distance   : %.2dm\n", distance);
}
void TelemetryManager::onRollData(TelemetryDecoder *decoder, float rollAngle)
{
    _roll = rollAngle;
    // Serial.printf("Roll       : %.2f\xF8\n", rollAngle);
}
void TelemetryManager::onPitchData(TelemetryDecoder *decoder, float pitchAngle)
{
    _pitch = pitchAngle;
    // Serial.printf("Pitch      : %.2f\xF8\n", pitchAngle);
}
void TelemetryManager::onAirSpeedData(TelemetryDecoder *decoder, float speed)
{
    _airspeed = speed;
    // Serial.printf("AirSpeed   : %.2fm/s\n", speed);
}

// FRIESH PROTOCOL


//-----------------------------------------------------------------------------------------
// BLE DataHandler stuff
//-----------------------------------------------------------------------------------------

BLEDataHandler::BLEDataHandler() : DataHandler() {}
BLEDataHandler::BLEDataHandler(size_t size) : DataHandler(size) {}

void BLEDataHandler::onLinkConnected(DataLink *link)
{
}

void BLEDataHandler::onLinkDisconnected(DataLink *link)
{
    g_frskyConnected = false;
}


/// BLE Frsky Client callbacks
BLEFrskyConnection::BLEFrskyConnection() : _connected(false)
{
    setClientCallbacks(this);
}

void BLEFrskyConnection::onResult(BLEAdvertisedDevice advertisedDevice)
{
    if(_connected || _address!=0) return;

    BLEAddress addr = advertisedDevice.getAddress();
    uint64_t address = *(uint64_t *)(addr.getNative()) & 0x0000FFFFFFFFFFFF;

    if(Settings.isFrskyAddressSet() && (address != Settings.getFrskAddress())) return;

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.isAdvertisingService(FRSKY_STREAM_SERVICE_UUID))
    {

        BLEDevice::getScan()->stop();

        Serial.print("BLEFrsky Device ");
        Serial.println(advertisedDevice.toString().c_str());
        
        _address = address;

    }
}

void BLEFrskyConnection::process() 
{
    if(!_connected && _address!=0) 
    {
        // Do connect
        Serial.printf("connecting to %02X:%02X:%02X:%02X:%02X:%02X\n",
                      (uint8_t)(_address >>  0),
                      (uint8_t)(_address >>  8),
                      (uint8_t)(_address >> 16),
                      (uint8_t)(_address >> 24),
                      (uint8_t)(_address >> 32),
                      (uint8_t)(_address >> 40));
        
        if(connect(BLEAddress((uint8_t*)&_address)))
        {
            _connected = pFrskyStream->setFrskyService(this->getService(FRSKY_STREAM_SERVICE_UUID));
        }
        if(_connected) {
            Serial.println("BLEFrskyStream connected ...OK");
        } else {
            _address=0;
            Serial.println("BLEFrskyStream connection ...Failed");
        }
        return;
    }

    if(_connected && _address == 0) 
    {
        // Do disconnect
        Serial.println("BLEFrskyStream disconnected");
        pFrskyStream->setFrskyService(nullptr);
        _connected = false;
        _address=0;
        Serial.println("Start scanning...");
        BLEDevice::getScan()->start(-1, nullptr, false); // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
}


void BLEFrskyConnection::onConnect(BLEClient *pClient) 
{
}
	
void BLEFrskyConnection::onDisconnect(BLEClient *pClient) 
{
    _address = 0;
}

#ifdef FRIESH_SERVER_ENABLED
BLEFrieshConnection::BLEFrieshConnection() : _connected(false)
{
}

void BLEFrieshConnection::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param)
{
    uint64_t addr =  (*(uint64_t*)param->connect.remote_bda) & 0x0000FFFFFFFFFFFF;
    _connected = true;
    Serial.printf("FrieshClient connected: %012X\n", addr);
};

void BLEFrieshConnection::onDisconnect(BLEServer *pServer)
{
    _connected = false;
    Serial.println("FrieshClient disconnected...");
}

bool BLEFrieshConnection::isConnected() 
{
    return _connected;
}
#endif
