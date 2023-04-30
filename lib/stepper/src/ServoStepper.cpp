#include "ServoStepper.h"


ServoStepper::ServoStepper():
  _state(IDLE),
  _speed(0),
  _lastFreq(0),
  _pwmPin(0),
  _stop_us_low(1406),
  _stop_us_high(1539),
  _stop_us((_stop_us_low + _stop_us_high)/2),
  _servo()
  {}


bool ServoStepper::attach(uint8_t pwmPin) {
  int res = _servo.attach(pwmPin);
  if (res != 0)
  {
    return true;
  } else
  {
    return false;
  }
}

void ServoStepper::detach() {
  this->_servo.detach();
}

void ServoStepper::move(float speed) {
  if (speed == 0) {
    this->stop();
    return;
  }
  int offset = 0;
  offset = speed  > 0 ? this->_stop_us_high : this->_stop_us_low;
  this->_servo.writeMicroseconds(offset + speed);
  //Serial.println("servo us : " + String(offset + speed));
}

void ServoStepper::stop() {
  this->_servo.writeMicroseconds(this->_stop_us);
}

bool ServoStepper::isMoving() {
  if (this->_servo.readMicroseconds() != this->_stop_us)
  {
    return true;
  } else
  {
    return false;
  }
}

void ServoStepper::move(float speed, float direction) {
  if(direction == DIR_CW) {
    this->move(speed);
  } else
  {
    this->move(-speed);
  }
}

ServoStepper::~ServoStepper() {
  this->detach();
}