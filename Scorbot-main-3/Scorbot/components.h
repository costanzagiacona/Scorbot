#ifndef COMPONENTS_H
#define COMPONENTS_H


//STM32
#if defined(ARDUINO_ARCH_STM32) 
#define STM32
#endif


#define PIN_CONTROL_EXTRA_FEATURES
#define PIN_CONTROL_STORE_VALUES
#define PIN_MEASURE_EXTRA_FEATURES
#define PIN_MEASURE_STORE_VALUES


#include <Arduino.h>
#include <math.h>
#include <stm32f4xx_hal.h>
#include <HardwareTimer.h>    /****************/
#include "Pid.h"
#include "control.h"
#include "util.h"

// PIN CONTROL //

class PinControl {
public:
  PinControl(uint8_t pin);
  PinControl(uint8_t pin, float v1, float v2);

  uint8_t getPin();

  void setLimits(float v1, float v2);

  void set(bool state);
  void pwm(uint8_t pwm);
  void control(float value);

  #if defined(PIN_CONTROL_EXTRA_FEATURES)
  void feedback(float error);
  void feedback();

  void setPID(PID *pid);
  PID* getPID();
  #endif

  #if defined(PIN_CONTROL_STORE_VALUES)
  bool last_set();
  uint8_t last_pwm();
  float last_control();
  #endif

public:
  uint8_t pin;
  float v1;
  float v2;

  #if defined(PIN_CONTROL_EXTRA_FEATURES)
  PID *pid;
  #endif

  #if defined(PIN_CONTROL_STORE_VALUES)
  bool set_;
  uint8_t pwm_;
  #endif
};


class PinMeasure {
public:
  PinMeasure(uint8_t pin, bool pullup = false);
  PinMeasure(uint8_t pin, float v1, float v2, bool pullup = false);
  
  uint8_t getPin();
  
  void setLimits(float v1, float v2);

  bool state();
  uint16_t value();
  float measure();

  #if defined(PIN_MEASURE_EXTRA_FEATURES)
  float filter(bool readonly);
  float filter();

  void setFilter(Filter *filter);
  Filter* getFilter();
  #endif

  #if defined(PIN_MEASURE_STORE_VALUES)
  bool last_state();
  uint16_t last_value();
  float last_measure();
  #endif

private:
  uint8_t pin;
  float v1;
  float v2;

  #if defined(PIN_MEASURE_EXTRA_FEATURES)
  Filter* fil;
  #endif

  #if defined(PIN_MEASURE_STORE_VALUES)
  bool state_;
  uint16_t value_;
  #endif
};


// DC Motor with Encoder
class Motor{
public:
  Motor(PinControl &INA, PinControl &INB, PinControl &PWM, PinMeasure &CHA, PinMeasure &CHB, PinMeasure &END);
  ~Motor();

  // Motor operating modes
  enum class OperatingMode {
    BRAKE_GND,
    SPIN_CCW,
    SPIN_CW,
    BRAKE_VCC
  };

  void invertEncoder(bool invert);  // Invert encoder ticks counting direction
  long getEncoder();                // Return encoder ticks
  void setEncoder(long value);      // Set/reset encoder ticks
  void readEncoder();               // Read encoder pins
  void updateEncoder();             // Update encoder ticks

  void invertMotor(bool invert);    // Invert physical spin direction of the motor
  void driveMotor(int16_t spwm);      // Assign pwm with sign for spin direction

  bool isInEndStop();               // Check if motor is at endstop

public:
  PinControl &pin_INA;          // A pin
  PinControl &pin_INB;          // B pin
  PinControl &pin_PWM;          // pwm pin
  PinMeasure &pin_CHA;          // encoder channel A
  PinMeasure &pin_CHB;          // encoder channel B
  PinMeasure &pin_END;          // endstop switch

  bool enc_A = false;           // Encoder channel A state
  bool enc_B = false;           // Encoder channel B state

  bool encoder_invert = false;  // If invert encoder counting direction
  long encoder = 0;             // Encoder ticks count

  bool motor_invert = false;    // If invert motor physical spin direction
};


#endif  // COMPONENTS_H