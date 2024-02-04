#ifndef __BSP_H__
#define __BSP_H__

#include <Arduino.h>
#include <stdint.h>

#ifdef SAMBI

#define ADC_BATT_PIN            35
#define BTN_PIN                 4
#define LED1_PIN                2

// Servo as Stepper
#define SERVO_STEPPER_PIN       33
#define SERVO_PIN               25
#define STEPPER_DIR_PIN         27
#define STEPPER_STEP_PIN        26
#define STEPPER_ENA_PIN         14

#define SERVO_ZERO_OFFSET       90
#define SERVO_DIRECTION         -1       // or -1
#define SERVO_MIN               -30
#define SERVO_MAX               90

#else

#define ADC_BATT_PIN            35
#define BTN_PIN                 23
#define LED1_PIN                2

#define SERVO_PIN               33
#define STEPPER_DIR_PIN         26
#define STEPPER_STEP_PIN        27
#define STEPPER_ENA_PIN         14

#define SERVO_ZERO_OFFSET       50
#define SERVO_DIRECTION         1   // or -1
#define SERVO_MIN               0   //-30
#define SERVO_MAX               90
#endif


#define STEPPER_STEP_PER_REV    (4076 * 5)


// BUTTON Stuff
#define BTN_LONG_PRESS_TIME   3000
#define BTN_DEBOUNCE_MS         40

void btnInit();
void IRAM_ATTR  btnChangedISR();
bool            isButtonPressed();
uint64_t        getButtonPressTime();
void            resetButtonPressTime();



// LED Stuff
void ledInit();
void IRAM_ATTR onLedTimer();
void ledState(bool state);
void ledBlink(uint16_t* profile, size_t len);


#define LED_ON   (0x8000)
#define LED_OFF  (0x0000)


// Motor Stuff
void motorPinsInit();



#endif // __BSP_H__