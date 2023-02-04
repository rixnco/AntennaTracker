#ifndef __SERVO_TEPPER_H__
#define __SERVO_TEPPER_H__

#include <stdint.h>
#include <limits>

class ServoStepper;


#define DIR_CW   (std::numeric_limits<float>::infinity())
#define DIR_CCW  (-std::numeric_limits<float>::infinity())

class ServoStepper {
public:
    ServoStepper();
    virtual ~ServoStepper();
    
    bool attach(uint8_t pwmPin);
    void detach();
    void move(float speed, float angle=DIR_CW);
    bool isMoving();
    void stop();

protected:
    void onStep();

    static void onStepISR(void*);

private:
    enum State { IDLE, MOVING, STOPPING };

    State                _state;
    volatile float       _speed;
    volatile uint32_t    _lastFreq;

    uint8_t     _pwmPin;
    uint8_t     _dirPin;
    uint8_t     _enablePin;
    uint32_t    _stepPerRev;
    float       _maxSpeed;
    bool        _reverseDir;
};

#endif