//
// Created by Samuel HEIDMANN on 27/02/2021.
//

#include "AS5600.h"

bool AS5600::_read_word(byte addr, byte reg, uint16_t& res) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    if(Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
    while(Wire.available() <2 );
    res = ((Wire.read()) << 8) | (Wire.read() & 0xFF);
    return true;
}

bool AS5600::_read_byte(byte addr, byte reg, uint8_t& res) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    if(Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
    while(Wire.available() < 1 );
    res = (Wire.read() & 0xFF);
    return true;
}

bool AS5600::init() {
    byte status = 0;
    byte reg;
    if(!_read_byte(AS5600_ADDR, AS5600_STAT, reg)) return false;
    reg = reg & 0b00111000u;
    if(reg!=status) {
        status = reg;
        Serial.printf("status: %02X\n", status);
    } else {
        Serial.printf("Nope\n");
    }
    return status;
}

uint16_t AS5600::getRawAngle() {
    return _reg1;
}

AS5600::AS5600() {
    _address = AS5600_ADDR;
    _reg1 = 0;
    _reg2 = 0;
    _reg3 = 0;
}

void AS5600::read() {
    _read_word(AS5600_ADDR, AS5600_ANG_HI, _reg1);
    _read_byte(AS5600_ADDR, AS5600_STAT, _reg2);
    _reg2 &= 0b00111000u;
    //_read_byte(AS5600_ADDR, AS5600_AGC, _reg3);
}

float AS5600::getAngleDegrees() {
    return _reg1 * 360. / 4096.;
}

byte AS5600::getStatus() const {
    return _reg2;
}
