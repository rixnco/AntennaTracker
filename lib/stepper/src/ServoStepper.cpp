#include "ServoStepper.h"
#include "ESP32PWM.h"
#include <cstdint>

ServoStepper::ServoStepper():
  _state(IDLE),
  _channel(-1),
  _speed(0),
  _lastFreq(0),
  _pwmPin(0),
  _stop_us(0),
  _dirPin(0),
  _minimum_speed(0),
  _isMoving(false)
  {}


void ServoStepper::init(int pwm_pin, int dir_pin, unsigned int freq, int channel, int resolution, uint8_t min_speed) {
  this->_channel = channel;
  this->_pwmPin = pwm_pin;
  this->_dirPin = dir_pin;
  this->_minimum_speed = min_speed;
  ledcSetup(this->_channel, freq, resolution);
  ledcAttachPin(this->_pwmPin, this->_channel); 
}

void ServoStepper::stop() {
  ledcWrite(this->_channel, 0);
  this->_isMoving = false;
}

bool ServoStepper::isMoving() {
  return this->_isMoving;
}

void ServoStepper::move(float speed, float direction) {
  if (speed == 0) {
    this->stop();
  } else {
    if(direction == 0) {
      digitalWrite(this->_dirPin, HIGH);
    } else
    {
      digitalWrite(this->_dirPin, LOW);
    }
    uint32_t uspeed = (uint32_t)(fabs(speed) + this->_minimum_speed);
    if(uspeed > 255) uspeed = 255;
    ledcWrite(this->_channel, uspeed);
    this->_isMoving = true;
  }
}

ServoStepper::~ServoStepper() {
  // TODO
}