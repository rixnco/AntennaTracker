#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <driver/mcpwm.h>
#include <limits>

class ESP32Stepper;

typedef void (*stepper_callback_t)(ESP32Stepper*);


#define DIR_CW   (std::numeric_limits<float>::infinity())
#define DIR_CCW  (-std::numeric_limits<float>::infinity())



class ESP32Stepper {
public:
    ESP32Stepper();
    virtual ~ESP32Stepper();
    
    bool attach(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint32_t stepPerRev, float maxSpeed, bool reverseDir=false);
    void detach();
    inline bool isAttached() { return _instance!=nullptr; }

    void setStepCallback(stepper_callback_t callback);
    void setStopCallback(stepper_callback_t callback);

    void move(float speed, float angle=DIR_CW);
    bool isMoving();
    void stop();

protected:
    void onStep();

    static void onStepISR(void*);

private:
    typedef struct {
        ESP32Stepper*       stepper;
        mcpwm_unit_t        unit;
        mcpwm_timer_t       timer;
        mcpwm_io_signals_t  pwm;
    } MCPWMInstance; 

    enum State { IDLE, MOVING, STOPPING };

    State                _state;
    volatile float       _speed;
    volatile uint32_t    _remainingSteps;
    volatile uint32_t    _lastFreq;

    uint8_t     _stepPin;
    uint8_t     _dirPin;
    uint8_t     _enablePin;
    uint32_t    _stepPerRev;
    float       _maxSpeed;
    bool        _reverseDir;

    stepper_callback_t  _stepCallback;
    stepper_callback_t  _stopCallback;

    MCPWMInstance*      _instance;

    #define MCPWM_COUNT             (6)
    static MCPWMInstance _instances[MCPWM_COUNT];
};




#endif // __MCSTEPPER_H__