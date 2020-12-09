#include "ESP32Stepper.h"
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <math.h>



//#define DEBUG




#ifdef DEBUG

#ifdef ARDUINO
#include <Arduino.h>
#define dbg_print(_fmt)         do { Serial.printf(_fmt); } while(0)
#define dbg_printf(_fmt,...)    do { Serial.printf(_fmt, ##__VA_ARGS__); } while(0)

#else
#define dbg_print(_fmt)         do { printf(_fmt); } while(0)
#define dbg_printf(_fmt,...)    do { printf(_fmt, ##__VA_ARGS__); } while(0)
#endif

#else

#define dbg_print(_fmt)         ((void)0)
#define dbg_printf(_fmt,...)    ((void)0)

#endif









#define TIMER_TEP_INT_BIT(t) (MCPWM_TIMER0_TEP_INT_ENA << t)

void IRAM_ATTR onStepISR(void *arg);

static mcpwm_dev_t    *__MCPWM[2] = {&MCPWM0, &MCPWM1};
static intr_handle_t   __INTR_HANDLE[2] = { NULL, NULL};


ESP32Stepper::MCPWMInstance ESP32Stepper::_instances[MCPWM_COUNT] = {
    { .stepper= nullptr, .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_0, .pwm = MCPWM0A },
    { .stepper= nullptr, .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_1, .pwm = MCPWM1A },
    { .stepper= nullptr, .unit = MCPWM_UNIT_0, .timer = MCPWM_TIMER_2, .pwm = MCPWM2A },
    { .stepper= nullptr, .unit = MCPWM_UNIT_1, .timer = MCPWM_TIMER_0, .pwm = MCPWM0A },
    { .stepper= nullptr, .unit = MCPWM_UNIT_1, .timer = MCPWM_TIMER_1, .pwm = MCPWM1A },
    { .stepper= nullptr, .unit = MCPWM_UNIT_1, .timer = MCPWM_TIMER_2, .pwm = MCPWM2A },
};



ESP32Stepper::ESP32Stepper() :  
    _state(IDLE), 
    _speed(0), 
    _remainingSteps(0), 
    _stepPin(0), 
    _dirPin(0),
    _enablePin(0),
    _stepPerRev(0), 
    _reverseDir(false), 
    _stepCallback(nullptr), 
    _stopCallback(nullptr), 
    _instance(nullptr) 
{
}

ESP32Stepper::~ESP32Stepper() {
    detach();
}

bool ESP32Stepper::attach(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint32_t stepPerRev, bool reverseDir)
{
    if(_instance!=nullptr) {
        // Already attached.
        return false;
    }

    for(int t=0; t<MCPWM_COUNT; ++t) {
        if(_instances[t].stepper==nullptr) {
            _instances[t].stepper= this;
            _instance= &_instances[t];
            break;
        }
    }

    if(_instance==nullptr) {
        // no more mcpwm instance available.
        return false;
    }

    _stepPin = stepPin;
    _dirPin = dirPin;
    _enablePin = enablePin;
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

    gpio_config_t conf_enbale_pin;
    conf_enbale_pin.pin_bit_mask = 1 << _enablePin;
    conf_enbale_pin.mode = GPIO_MODE_OUTPUT;
    conf_enbale_pin.pull_up_en = GPIO_PULLUP_DISABLE;
    conf_enbale_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf_enbale_pin.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&conf_enbale_pin);
    gpio_set_level((gpio_num_t)_enablePin, 1);

    mcpwm_gpio_init(_instance->unit, _instance->pwm, _stepPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(_instance->unit, _instance->timer, &pwm_config);
    mcpwm_stop(_instance->unit, _instance->timer);
    mcpwm_set_signal_low(_instance->unit, _instance->timer, MCPWM_OPR_A);

    dbg_printf("unit=%d timer=%d\n", _instance->unit, _instance->timer);

    // Clear pending Timer TEP interrupt if any
    __MCPWM[_instance->unit]->int_clr.val  = TIMER_TEP_INT_BIT(_instance->timer);
    // Enable Timer TEP interrupt
    __MCPWM[_instance->unit]->int_ena.val |= TIMER_TEP_INT_BIT(_instance->timer);

    dbg_printf("int_ena=%04X\n", __MCPWM[_instance->unit]->int_ena.val);

    if(__INTR_HANDLE[_instance->unit] == nullptr) {
        dbg_printf("Registering isr for unit %d\n", _instance->unit);
        mcpwm_isr_register(_instance->unit, onStepISR, (void *)(&_instance->unit), ESP_INTR_FLAG_LEVEL1, &__INTR_HANDLE[_instance->unit]);
    }

    return true;
}

void ESP32Stepper::detach() {
    if(_instance==nullptr) return;
    stop();
    // Disable Timer TEP interrupt
    __MCPWM[_instance->unit]->int_ena.val &= ~TIMER_TEP_INT_BIT(_instance->timer);
    // Clear pending Timer TEP interrupt if any
    __MCPWM[_instance->unit]->int_clr.val  = TIMER_TEP_INT_BIT(_instance->timer);

    _instance->stepper = nullptr;

    gpio_reset_pin((gpio_num_t)_stepPin);
    gpio_reset_pin((gpio_num_t)_dirPin);

    _state= IDLE;
    _speed =0;
    _remainingSteps = 0;
    _stepPin =0;
    _dirPin = 0;
    _enablePin = 0;
    _stepPerRev = 0;
    _reverseDir = false;
    _stepCallback = nullptr;
    _stopCallback = nullptr;

    _instance = nullptr;
}

void ESP32Stepper::setStepCallback(stepper_callback_t callback) { 
    _stepCallback = callback; 
};
void ESP32Stepper::setStopCallback(stepper_callback_t callback) { 
    _stopCallback = callback; 
};


void ESP32Stepper::move(float speed, float angle)
{
    if(_instance==nullptr) return;
    
    stop();
    if (speed <= 0 ) {
        return;
    }
    gpio_set_level((gpio_num_t)_enablePin, 0);
    bool dir= !_reverseDir;
    dir= angle<0?!dir:dir;
    angle= fabs(angle);

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
    mcpwm_set_frequency(_instance->unit, _instance->timer, freq);
    mcpwm_set_duty_in_us(_instance->unit, _instance->timer, MCPWM_OPR_A, 5);
    mcpwm_set_duty_type(_instance->unit, _instance->timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    _state = MOVING;
    mcpwm_start(_instance->unit, _instance->timer);
}

bool ESP32Stepper::isMoving() { return _state == MOVING; }


void ESP32Stepper::stop()
{
    if(_instance==nullptr) return;

    if(_state != MOVING) return;
    mcpwm_stop(_instance->unit, _instance->timer);
    mcpwm_set_signal_low(_instance->unit, _instance->timer, MCPWM_OPR_A);
    _state = STOPPING;
}

void ESP32Stepper::onStep()
{
    if (_remainingSteps >= 1)
    {
        --_remainingSteps;
        if (_remainingSteps == 0)
        {
            _state = STOPPING;
        }
    } 
    if(_state == STOPPING) {
        mcpwm_set_signal_low(_instance->unit, _instance->timer, MCPWM_OPR_A);
        mcpwm_stop(_instance->unit, _instance->timer);
        _state = IDLE;
        _speed =0;
        gpio_set_level((gpio_num_t)_enablePin, 1);
        if(_stopCallback != nullptr) _stopCallback(this);
    } else  {
        if(_stepCallback != NULL) {
            _stepCallback(this); 
        }
    }
}

void ESP32Stepper::onStepISR(void* arg)
{
    mcpwm_unit_t unit = * reinterpret_cast<mcpwm_unit_t*>(arg);
    ESP32Stepper* stepper= nullptr;

    uint32_t status = __MCPWM[unit]->int_st.val; //Read interrupt status
    for(int t=0; t<3; ++t) {
        stepper=_instances[unit*3+t].stepper;
        if(stepper && (status & TIMER_TEP_INT_BIT(t))) {
            stepper->onStep();
        }
    }
    __MCPWM[unit]->int_clr.val = status;

}
