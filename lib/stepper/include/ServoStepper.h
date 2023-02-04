#ifndef __SERVO_TEPPER_H__
#define __SERVO_TEPPER_H__

#include <stdint.h>
#include <limits>
#include <ESP32Servo.h>

class ServoStepper;


#define DIR_CW   (std::numeric_limits<float>::infinity())
#define DIR_CCW  (-std::numeric_limits<float>::infinity())

class ServoStepper {
public:
    ServoStepper();
    virtual ~ServoStepper();
    
    bool attach(uint8_t pwmPin);
    void detach();
    void move(float speed, float direction);
    void move(float speed);
    bool isMoving();
    void stop();  

private:
    enum State { IDLE, MOVING, STOPPING };

    Servo                _servo;
    State                _state;
    volatile float       _speed;
    int                  _stop_us;
    volatile uint32_t    _lastFreq;

    uint8_t     _pwmPin;
    float       _maxSpeed;
};

#endif