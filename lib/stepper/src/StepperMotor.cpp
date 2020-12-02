#include "StepperMotor.h"
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <math.h>

#define TIMER_TEP_INT_BIT(t) (MCPWM_TIMER0_TEP_INT_ENA << t)

#define STOPPING    (0)
#define STOPPED     (-1)


void IRAM_ATTR onStepISR(void *arg);

static mcpwm_dev_t *__MCPWM[2] = {&MCPWM0, &MCPWM1};

StepperMotor Stepper1(MCPWM_UNIT_0, MCPWM_TIMER_0);
StepperMotor Stepper2(MCPWM_UNIT_0, MCPWM_TIMER_1);
StepperMotor Stepper3(MCPWM_UNIT_0, MCPWM_TIMER_2);

StepperMotor::StepperMotor(mcpwm_unit_t unit, mcpwm_timer_t timer) :  _speed(STOPPED), _remainingSteps(0), _minSpeed(0),  _maxSpeed(std::numeric_limits<float>::max()), _unit(unit), _timer(timer), _stepCallback(nullptr), _stopCallback(nullptr)
{
}

bool StepperMotor::init(uint8_t stepPin, uint8_t dirPin, uint32_t stepPerRev, bool reverseDir)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _stepPerRev = stepPerRev;
    _reverseDir = reverseDir;

    gpio_config_t conf;
    conf.pin_bit_mask = 1 << _dirPin;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;    

    gpio_config(&conf);
    gpio_set_level((gpio_num_t)_dirPin, 0);
    // pinMode(_dirPin, OUTPUT);
    // digitalWrite(_dirPin, LOW);

    mcpwm_gpio_init(_unit, MCPWM0A, _stepPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(_unit, _timer, &pwm_config);
    mcpwm_stop(_unit, _timer);
    mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_A);

    __MCPWM[_unit]->int_ena.val = TIMER_TEP_INT_BIT(_timer);
    mcpwm_isr_register(_unit, onStepISR, (void *)this, ESP_INTR_FLAG_LEVEL1, NULL);

    return true;
}

void StepperMotor::setStepCallback(stepper_callback_t callback) { 
    _stepCallback = callback; 
};
void StepperMotor::setStopCallback(stepper_callback_t callback) { 
    _stopCallback = callback; 
};


void StepperMotor::move(float speed, float angle)
{
    stop();
    if (speed <= 0 )
        return;
    bool dir= !_reverseDir;
    dir= angle<0?!dir:dir;
    
    angle= fabs(angle);

    if(speed<_minSpeed) speed = _minSpeed;
    if(speed>_maxSpeed) speed = _maxSpeed;

    if(angle>DIR_CCW && angle<DIR_CW) {
        _remainingSteps = (uint32_t)(angle * _stepPerRev / 360.f);
        if (angle>0 && _remainingSteps == 0)
            return;
    } else {
        _remainingSteps=0;
    }

    gpio_set_level((gpio_num_t)_dirPin, dir);

    uint32_t freq = (uint32_t)(speed * _stepPerRev / 360.f);
    _speed = speed;
    mcpwm_set_frequency(_unit, _timer, freq);
    mcpwm_set_duty_in_us(_unit, _timer, MCPWM_OPR_A, 5);
    mcpwm_set_duty_type(_unit, _timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_start(_unit, _timer);
}

bool StepperMotor::isMoving() { return _speed>=STOPPING; }


void StepperMotor::stop()
{
    if(_speed<=STOPPING) return;
    mcpwm_stop(_unit, _timer);
    mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_A);
    _remainingSteps = 1;
    _speed = STOPPING;
}

void StepperMotor::onStep()
{
     uint32_t status;
    status = __MCPWM[_unit]->int_st.val; //Read interrupt status
    if (status & (TIMER_TEP_INT_BIT(_timer))) 
    {
        if (_remainingSteps >= 1)
        {
            --_remainingSteps;
            if (_remainingSteps == 0)
            {
                mcpwm_set_signal_low(_unit, _timer, MCPWM_OPR_A);
                mcpwm_stop(_unit, _timer);
                _speed = STOPPED;
                if(_stopCallback != nullptr) _stopCallback(this);
            } else  {
                if(_stepCallback != NULL) {
                    _stepCallback(this); 
                }
            }
        } else {
            if(_stepCallback != NULL) {
                _stepCallback(this); 
            }
        }

    }
    __MCPWM[_unit]->int_clr.val = status;

}

void StepperMotor::onStepISR(void *arg)
{
    if(arg!=NULL) {
        ((StepperMotor*)arg)->onStep();
    }
}
