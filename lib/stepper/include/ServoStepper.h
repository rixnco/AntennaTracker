#ifndef __SERVO_TEPPER_H__
#define __SERVO_TEPPER_H__

#include <stdint.h>
#include <limits>


class ServoStepper {
public:
    ServoStepper();
    virtual ~ServoStepper();
    
    void init(int pwm_pin, int dir_pin, unsigned int freq, int channel, int resolution, uint8_t min_speed);
    void move(float speed, float direction);
    bool isMoving();
    void stop();

private:
    enum State { IDLE, MOVING, STOPPING };

    State                _state;
    int                  _channel;
    volatile float       _speed;
    int                  _stop_us;
    volatile uint32_t    _lastFreq;
    bool                 _isMoving;
    uint8_t              _minimum_speed;

    uint8_t     _pwmPin;
    uint8_t     _dirPin;
    float       _maxSpeed;
};

#endif