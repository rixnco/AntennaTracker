#include "TelemetryDecoder.h"

TelemetryListener::~TelemetryListener() {}
void TelemetryListener::onFrameDecoded(TelemetryDecoder* decoder, uint32_t id) {}
void TelemetryListener::onFrameError(TelemetryDecoder* decoder, TelemetryError error, uint32_t param) {}
void TelemetryListener::onFuelData(TelemetryDecoder* decoder, int fuel) {}
void TelemetryListener::onGPSData(TelemetryDecoder* decoder, float latitude, float longitude) {}
void TelemetryListener::onVBATData(TelemetryDecoder* decoder, float voltage) {}
void TelemetryListener::onCellVoltageData(TelemetryDecoder* decoder, float voltage) {}
void TelemetryListener::onCurrentData(TelemetryDecoder* decoder, float current) {}
void TelemetryListener::onHeadingData(TelemetryDecoder* decoder, float heading) {}
void TelemetryListener::onRSSIData(TelemetryDecoder* decoder, int rssi) {}
void TelemetryListener::onRxBtData(TelemetryDecoder* decoder, float voltage) {}
void TelemetryListener::onGPSStateData(TelemetryDecoder* decoder, int satellites, bool gpsFix) {}
void TelemetryListener::onVSpeedData(TelemetryDecoder* decoder, float vspeed) {}
void TelemetryListener::onAltitudeData(TelemetryDecoder* decoder, float altitude) {}
void TelemetryListener::onGPSAltitudeData(TelemetryDecoder* decoder, float altitude) {}
void TelemetryListener::onDistanceData(TelemetryDecoder* decoder, int distance) {}
void TelemetryListener::onRollData(TelemetryDecoder* decoder, float rollAngle) {}
void TelemetryListener::onPitchData(TelemetryDecoder* decoder, float pitchAngle) {}
void TelemetryListener::onGSpeedData(TelemetryDecoder* decoder, float speed) {}
void TelemetryListener::onAirSpeedData(TelemetryDecoder* decoder, float speed) {}



TelemetryDecoder::TelemetryDecoder(std::string name) : DataDecoder(name), _listener(nullptr) {
    
};
TelemetryDecoder::~TelemetryDecoder() {
    _listener= nullptr;
};

void TelemetryDecoder::setTelemetryListener(TelemetryListener* listener) {
    _listener = listener;
}

void TelemetryDecoder::fireFrameDecoded(uint32_t id) { 
    if(_listener!=nullptr) _listener->onFrameDecoded(this, id); 
}
void TelemetryDecoder::fireFrameError(TelemetryError error, uint32_t param) { 
    if(_listener!=nullptr) _listener->onFrameError(this, error, param); 
}
void TelemetryDecoder::fireFuelData(int fuel) { 
    if(_listener!=nullptr) _listener->onFuelData(this, fuel); 
}
void TelemetryDecoder::fireGPSData(float latitude, float longitude) { 
    if(_listener!=nullptr) _listener->onGPSData(this, latitude,longitude); 
}
void TelemetryDecoder::fireVBATData(float voltage) { 
    if(_listener!=nullptr) _listener->onVBATData(this, voltage); 
}
void TelemetryDecoder::fireCellVoltageData(float voltage)  { 
    if(_listener!=nullptr) _listener->onCellVoltageData(this, voltage); 
}
void TelemetryDecoder::fireCurrentData(float current)  { 
    if(_listener!=nullptr) _listener->onCurrentData(this, current); 
}
void TelemetryDecoder::fireHeadingData(float heading) { 
    if(_listener!=nullptr) _listener->onHeadingData(this, heading); 
}
void TelemetryDecoder::fireRSSIData(int rssi) { 
    if(_listener!=nullptr) _listener->onRSSIData(this, rssi); 
}
void TelemetryDecoder::fireRxBtData(float voltage) { 
    if(_listener!=nullptr) _listener->onRxBtData(this, voltage); 
}
void TelemetryDecoder::fireGPSStateData(int satellites, bool gpsFix) { 
    if(_listener!=nullptr) _listener->onGPSStateData(this, satellites, gpsFix); 
}
void TelemetryDecoder::fireVSpeedData(float vspeed) { 
    if(_listener!=nullptr) _listener->onVSpeedData(this, vspeed); 
}
void TelemetryDecoder::fireAltitudeData(float altitude) { 
    if(_listener!=nullptr) _listener->onAltitudeData(this, altitude); 
}
void TelemetryDecoder::fireGPSAltitudeData(float altitude) { 
    if(_listener!=nullptr) _listener->onGPSAltitudeData(this, altitude); 
}
void TelemetryDecoder::fireDistanceData(int distance) { 
    if(_listener!=nullptr) _listener->onDistanceData(this, distance); 
}
void TelemetryDecoder::fireRollData(float rollAngle) { 
    if(_listener!=nullptr) _listener->onRollData(this, rollAngle); 
}
void TelemetryDecoder::firePitchData(float pitchAngle) { 
    if(_listener!=nullptr) _listener->onPitchData(this, pitchAngle); 
}
void TelemetryDecoder::fireGSpeedData(float speed) { 
    if(_listener!=nullptr) _listener->onGSpeedData(this, speed); 
}
void TelemetryDecoder::fireAirSpeedData(float speed) { 
    if(_listener!=nullptr) _listener->onAirSpeedData(this, speed); 
}
