//
// Created by Samuel HEIDMANN on 27/02/2021.
//

#include "AS5600.h"

uint16_t AS5600::_read_word(byte addr, byte reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, (uint8_t)2);
    while(Wire.available() <2 );
    uint16_t res = ((Wire.read()) << 8) | (Wire.read() & 0xFF);
    return res;
}

uint8_t AS5600::_read_byte(byte addr, byte reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, (uint8_t)1);
    while(Wire.available() < 1 );
    uint8_t res = (Wire.read() & 0xFF);
    return res;
}

void AS5600::init() {
    byte status = 0;
    byte reg = _read_byte(AS5600_ADDR, AS5600_STAT) & 0b00111000u;
    if(reg!=status) {
        status = reg;
        Serial.printf("status: %02X\n", status);
    } else {
        Serial.printf("Nope\n");
    }
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
    _reg1= _read_word(AS5600_ADDR, AS5600_ANG_HI);
    _reg2 = _read_byte(AS5600_ADDR, AS5600_STAT) & 0b00111000u;
    //_reg3 = _read_byte(AS5600_ADDR, AS5600_AGC);
}

float AS5600::getAngleDegrees() {
    return _reg1 * 360. / 4096.;
}

byte AS5600::getStatus() const {
    return _reg2;
}
