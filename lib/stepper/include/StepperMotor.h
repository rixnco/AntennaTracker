#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <driver/mcpwm.h>
#include <limits>

class StepperMotor;

typedef void (*stepper_callback_t)(StepperMotor*);


#define DIR_CW   (std::numeric_limits<float>::infinity())
#define DIR_CCW  (-std::numeric_limits<float>::infinity())


class StepperMotor {
public:

    StepperMotor(mcpwm_unit_t unit, mcpwm_timer_t timer);

    bool init(uint8_t stepPin, uint8_t dirPin, uint32_t stepPerRev, bool reverseDir=false);

    inline void setReverseDir(bool reverseDir) { _reverseDir = reverseDir; };
    inline bool getReverseDir() { return _reverseDir; };
    inline void setMaxSpeed(float speed) { _maxSpeed = speed; };
    inline float getMaxSpeed() { return _maxSpeed; };
    inline void setMinSpeed(float speed) { _minSpeed = speed; };
    inline float getMinSpeed() { return _minSpeed; };

    void setStepCallback(stepper_callback_t callback);
    void setStopCallback(stepper_callback_t callback);

    void move(float speed, float angle=DIR_CW);
    bool isMoving();
    void stop();

protected:
    void onStep();

    static void onStepISR(void*);

private:
    volatile float       _speed;
    volatile uint32_t    _remainingSteps;

    uint8_t     _stepPin;
    uint8_t     _dirPin;
    uint32_t    _stepPerRev;
    bool        _reverseDir;
    float       _minSpeed;
    float       _maxSpeed;


    volatile mcpwm_unit_t    _unit;
    volatile mcpwm_timer_t   _timer;
    volatile stepper_callback_t _stepCallback;
    volatile stepper_callback_t _stopCallback;
};

#define Stepper Stepper1
extern StepperMotor Stepper1;
extern StepperMotor Stepper2;
extern StepperMotor Stepper3;



#endif // __MCSTEPPER_H__