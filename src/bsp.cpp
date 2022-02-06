#include "bsp.h"



static portMUX_TYPE      _btnMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t _btn_state_time;
static volatile bool     _btn_state;

static hw_timer_t *        _led_timer;
static portMUX_TYPE        _led_timerMux = portMUX_INITIALIZER_UNLOCKED;
static volatile size_t     _led_profile_idx;
static volatile size_t     _led_profile_len;
static volatile uint16_t   *_led_profile;




// BUTTON Stuff
void btnInit() 
{
    pinMode(BTN_PIN, INPUT_PULLUP);
    _btn_state = false;
    attachInterrupt(BTN_PIN, btnChangedISR, CHANGE);
}

void IRAM_ATTR  btnChangedISR() {
    uint64_t now= millis();

    portENTER_CRITICAL_ISR(&_btnMux);
    _btn_state_time=now;
    _btn_state= !digitalRead(BTN_PIN);
    portEXIT_CRITICAL_ISR(&_btnMux);
}

bool isButtonPressed() {
    bool res;
    portENTER_CRITICAL(&_btnMux);
    res = _btn_state && (millis()-_btn_state_time>BTN_DEBOUNCE_MS);
    portEXIT_CRITICAL(&_btnMux);
    return res; 
}

uint64_t getButtonPressTime() 
{
    uint64_t time=0;
    portENTER_CRITICAL(&_btnMux);
    if(_btn_state) time= millis()-_btn_state_time;
    portEXIT_CRITICAL(&_btnMux);
    return time;
}

void resetButtonPressTime()
{
    portENTER_CRITICAL(&_btnMux);
    if(_btn_state) _btn_state_time= millis();
    portEXIT_CRITICAL(&_btnMux);

}


void ledInit()
{
    pinMode(LED1_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    _led_timer = timerBegin(0, 80, true);
    /* Attach onTimer function to our timer */
    timerAttachInterrupt(_led_timer, &onLedTimer, true);
}

void IRAM_ATTR onLedTimer() {
    portENTER_CRITICAL_ISR(&_led_timerMux);
    _led_profile_idx = (_led_profile_idx+1) % _led_profile_len;
    digitalWrite(LED1_PIN, (_led_profile[_led_profile_idx]&LED_ON) != 0);
    timerAlarmWrite(_led_timer, 1000 * (_led_profile[_led_profile_idx]&(~LED_ON)), true);
    portEXIT_CRITICAL_ISR(&_led_timerMux);    
}

void ledState(bool state) 
{
    portENTER_CRITICAL(&_led_timerMux);
    timerAlarmDisable(_led_timer);
    timerStop(_led_timer);
    digitalWrite(LED1_PIN, state);
    portEXIT_CRITICAL(&_led_timerMux);    
}

void ledBlink(uint16_t* profile, size_t len)
{
    portENTER_CRITICAL(&_led_timerMux);
    timerStop(_led_timer);
    _led_profile= profile;
    _led_profile_len= len;
    _led_profile_idx = 0;
    digitalWrite(LED1_PIN, (_led_profile[_led_profile_idx]&LED_ON) != 0);
    timerAlarmWrite(_led_timer, 1000 * (_led_profile[_led_profile_idx]&(~LED_ON)), true);
    timerWrite(_led_timer, 0);
    timerAlarmEnable(_led_timer);
    timerStart(_led_timer);
    portEXIT_CRITICAL(&_led_timerMux);    
}

