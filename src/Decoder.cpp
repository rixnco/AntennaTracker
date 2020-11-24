#include "Decoder.h"

DecoderListener::~DecoderListener() {}
void DecoderListener::onFuelData(int fuel) {}
void DecoderListener::onGPSData(double latitude, double longitude) {}
void DecoderListener::onVBATData(float voltage) {}
void DecoderListener::onCellVoltageData(float voltage) {}
void DecoderListener::onCurrentData(float current) {}
void DecoderListener::onHeadingData(float heading) {}
void DecoderListener::onRSSIData(int rssi) {}
void DecoderListener::onGPSStateData(int satellites, bool gpsFix) {}
void DecoderListener::onVSpeedData(float vspeed) {}
void DecoderListener::onAltitudeData(float altitude) {}
void DecoderListener::onGPSAltitudeData(float altitude) {}
void DecoderListener::onDistanceData(int distance) {}
void DecoderListener::onRollData(float rollAngle) {}
void DecoderListener::onPitchData(float pitchAngle) {}
void DecoderListener::onGSpeedData(float speed) {}
void DecoderListener::onAirSpeedData(float speed) {}



Decoder::Decoder() : _listener(nullptr) {
    
};
Decoder::~Decoder() {
    _listener= nullptr;
};

void Decoder::setDecoderListener(DecoderListener* listener) {
    _listener = listener;
}

void Decoder::onFuelData(int fuel) { 
    if(_listener!=nullptr) _listener->onFuelData(fuel); 
}
void Decoder::onGPSData(double latitude, double longitude) { 
    if(_listener!=nullptr) _listener->onGPSData(latitude,longitude); 
}
void Decoder::onVBATData(float voltage) { 
    if(_listener!=nullptr) _listener->onVBATData(voltage); 
}
void Decoder::onCellVoltageData(float voltage)  { 
    if(_listener!=nullptr) _listener->onCellVoltageData(voltage); 
}
void Decoder::onCurrentData(float current)  { 
    if(_listener!=nullptr) _listener->onCurrentData(current); 
}
void Decoder::onHeadingData(float heading) { 
    if(_listener!=nullptr) _listener->onHeadingData(heading); 
}
void Decoder::onRSSIData(int rssi) { 
    if(_listener!=nullptr) _listener->onRSSIData(rssi); 
}
void Decoder::onGPSStateData(int satellites, bool gpsFix) { 
    if(_listener!=nullptr) _listener->onGPSStateData(satellites, gpsFix); 
}
void Decoder::onVSpeedData(float vspeed) { 
    if(_listener!=nullptr) _listener->onVSpeedData(vspeed); 
}
void Decoder::onAltitudeData(float altitude) { 
    if(_listener!=nullptr) _listener->onAltitudeData(altitude); 
}
void Decoder::onGPSAltitudeData(float altitude) { 
    if(_listener!=nullptr) _listener->onGPSAltitudeData(altitude); 
}
void Decoder::onDistanceData(int distance) { 
    if(_listener!=nullptr) _listener->onDistanceData(distance); 
}
void Decoder::onRollData(float rollAngle) { 
    if(_listener!=nullptr) _listener->onRollData(rollAngle); 
}
void Decoder::onPitchData(float pitchAngle) { 
    if(_listener!=nullptr) _listener->onPitchData(pitchAngle); 
}
void Decoder::onGSpeedData(float speed) { 
    if(_listener!=nullptr) _listener->onGSpeedData(speed); 
}
void Decoder::onAirSpeedData(float speed) { 
    if(_listener!=nullptr) _listener->onAirSpeedData(speed); 
}
