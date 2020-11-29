#include "TelemetryDecoder.h"

TelemetryListener::~TelemetryListener() {}
void TelemetryListener::onFuelData(int fuel) {}
void TelemetryListener::onGPSData(double latitude, double longitude) {}
void TelemetryListener::onVBATData(float voltage) {}
void TelemetryListener::onCellVoltageData(float voltage) {}
void TelemetryListener::onCurrentData(float current) {}
void TelemetryListener::onHeadingData(float heading) {}
void TelemetryListener::onRSSIData(int rssi) {}
void TelemetryListener::onGPSStateData(int satellites, bool gpsFix) {}
void TelemetryListener::onVSpeedData(float vspeed) {}
void TelemetryListener::onAltitudeData(float altitude) {}
void TelemetryListener::onGPSAltitudeData(float altitude) {}
void TelemetryListener::onDistanceData(int distance) {}
void TelemetryListener::onRollData(float rollAngle) {}
void TelemetryListener::onPitchData(float pitchAngle) {}
void TelemetryListener::onGSpeedData(float speed) {}
void TelemetryListener::onAirSpeedData(float speed) {}



TelemetryDecoder::TelemetryDecoder() : _listener(nullptr) {
    
};
TelemetryDecoder::~TelemetryDecoder() {
    _listener= nullptr;
};

void TelemetryDecoder::setTelemetryListener(TelemetryListener* listener) {
    _listener = listener;
}

void TelemetryDecoder::onFuelData(int fuel) { 
    if(_listener!=nullptr) _listener->onFuelData(fuel); 
}
void TelemetryDecoder::onGPSData(double latitude, double longitude) { 
    if(_listener!=nullptr) _listener->onGPSData(latitude,longitude); 
}
void TelemetryDecoder::onVBATData(float voltage) { 
    if(_listener!=nullptr) _listener->onVBATData(voltage); 
}
void TelemetryDecoder::onCellVoltageData(float voltage)  { 
    if(_listener!=nullptr) _listener->onCellVoltageData(voltage); 
}
void TelemetryDecoder::onCurrentData(float current)  { 
    if(_listener!=nullptr) _listener->onCurrentData(current); 
}
void TelemetryDecoder::onHeadingData(float heading) { 
    if(_listener!=nullptr) _listener->onHeadingData(heading); 
}
void TelemetryDecoder::onRSSIData(int rssi) { 
    if(_listener!=nullptr) _listener->onRSSIData(rssi); 
}
void TelemetryDecoder::onGPSStateData(int satellites, bool gpsFix) { 
    if(_listener!=nullptr) _listener->onGPSStateData(satellites, gpsFix); 
}
void TelemetryDecoder::onVSpeedData(float vspeed) { 
    if(_listener!=nullptr) _listener->onVSpeedData(vspeed); 
}
void TelemetryDecoder::onAltitudeData(float altitude) { 
    if(_listener!=nullptr) _listener->onAltitudeData(altitude); 
}
void TelemetryDecoder::onGPSAltitudeData(float altitude) { 
    if(_listener!=nullptr) _listener->onGPSAltitudeData(altitude); 
}
void TelemetryDecoder::onDistanceData(int distance) { 
    if(_listener!=nullptr) _listener->onDistanceData(distance); 
}
void TelemetryDecoder::onRollData(float rollAngle) { 
    if(_listener!=nullptr) _listener->onRollData(rollAngle); 
}
void TelemetryDecoder::onPitchData(float pitchAngle) { 
    if(_listener!=nullptr) _listener->onPitchData(pitchAngle); 
}
void TelemetryDecoder::onGSpeedData(float speed) { 
    if(_listener!=nullptr) _listener->onGSpeedData(speed); 
}
void TelemetryDecoder::onAirSpeedData(float speed) { 
    if(_listener!=nullptr) _listener->onAirSpeedData(speed); 
}
