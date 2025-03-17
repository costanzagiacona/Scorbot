#ifndef COMPONENTS_H
#define COMPONENTS_H

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif

//#define PIN_CONTROL_EXTRA_FEATURES
//#define PIN_CONTROL_STORE_VALUES
//#define PIN_MEASURE_EXTRA_FEATURES
//#define PIN_MEASURE_STORE_VALUES


#include <Arduino.h>
#include <math.h>
#include "utils.h"
#include "control.h"


#if defined(UNO) || defined(MEGA)
class PWMfreq {
public:
  PWMfreq() = delete;
  ~PWMfreq() = delete;

#if defined(UNO)
  enum class UnoTimer0 : unsigned char{   // D5 & D6
    FREQ_62500_00 = 0b00000001, // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    FREQ_7812_50  = 0b00000010, // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    FREQ_976_56   = 0b00000011, // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (DEFAULT)
    FREQ_244_14   = 0b00000100, // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    FREQ_61_04    = 0b00000101  // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  };

  enum class UnoTimer1 : unsigned char{   // D9 & D10 
    FREQ_31372_55 = 0b00000001, // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class UnoTimer2 : unsigned char{   // D3 & D11
    FREQ_31372_55 = 0b00000001, // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_980_39   = 0b00000011, // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
    FREQ_490_20   = 0b00000100, // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_245_10   = 0b00000101, // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    FREQ_122_55   = 0b00000110, // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000111  // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  };
#endif

#if defined(MEGA)
  enum class MegaTimer0 : unsigned char{   // D4 & D13
    FREQ_62500_00 = 0b00000001, // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
    FREQ_7812_50  = 0b00000010, // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    FREQ_976_56   = 0b00000011, // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (DEFAULT)
    FREQ_244_14   = 0b00000100, // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
    FREQ_61_04    = 0b00000101  // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
  };

  enum class MegaTimer1 : unsigned char{   // D11 & D12 
    FREQ_31372_55 = 0b00000001, // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer2 : unsigned char{   // D9 & D10
    FREQ_31372_55 = 0b00000001, // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_980_39   = 0b00000011, // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
    FREQ_490_20   = 0b00000100, // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_245_10   = 0b00000101, // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
    FREQ_122_55   = 0b00000110, // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000111  // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer3 : unsigned char{   // D2, D3 & D5 
    FREQ_31372_55 = 0b00000001, // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz
  };  

  enum class MegaTimer4 : unsigned char{   // D6, D7 & D8 
    FREQ_31372_55 = 0b00000001, // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz
  };

  enum class MegaTimer5 : unsigned char{   // D44, D45 & D46 
    FREQ_31372_55 = 0b00000001, // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
    FREQ_3921_16  = 0b00000010, // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
    FREQ_490_20   = 0b00000011, // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz (DEFAULT)
    FREQ_122_55   = 0b00000100, // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
    FREQ_30_64    = 0b00000101  // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz
  };
#endif

  //NOTE: Changing timer 0 affects millis() and delay!

#if defined(UNO)
  static void set(UnoTimer0 freq);
  static void set(UnoTimer1 freq);
  static void set(UnoTimer2 freq);
#endif

#if defined(MEGA)
  static void set(MegaTimer0 freq);
  static void set(MegaTimer1 freq);
  static void set(MegaTimer2 freq);
  static void set(MegaTimer3 freq);
  static void set(MegaTimer4 freq);
  static void set(MegaTimer5 freq);
#endif
};
#endif


#if defined(UNO) || defined(MEGA)
class SerialComm {
public:
  SerialComm() = delete;
  ~SerialComm() = delete;

  static HardwareSerial* port(uint8_t channel);

  static void start(HardwareSerial *hwserial, uint32_t baudrate, uint8_t config = SERIAL_8N1);
  static void start(uint8_t channel, uint32_t baudrate, uint8_t config = SERIAL_8N1);

  static void close(HardwareSerial *hwserial);
  static void close(uint8_t channel);
};
#endif


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

private:
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

private:
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


// 1-8 DoF Robot with DC Motors
class Robot {
public:
  Robot(PinControl &enable, uint8_t size, uint32_t ts_us);
  ~Robot();

  // Operating modes
  enum class Status : uint8_t{
    Idle = 0,   // All the motor are stopped
    Pwm  = 1,   // The robot act as a DAQ (pwm direct control)
    Ref  = 2    // The robot track encoders setpoint
  };
  
  int getSize();                                      // Return number of motors

  Status getStatus();                                 // Return robot status
  bool setStatus(Status status, bool reset = false);  // Set robot internal status

  uint32_t getTimeSampling();                     // Get robot time sampling (in microseconds)
  void setTimeSampling(uint32_t ts_us);           // Set robot time sampling (in microseconds)

  void setMotor(uint8_t index, Motor &motor);     // Set robot's motor specified by index
  void invertMotor(uint8_t index, bool inv);      // Invert motor's spin direction specified by index

  void initPID(uint8_t index, float sat, float pole);                     // Set PID properties for motor specified by index
  void setupPID(uint8_t index, float div, float kp, float ki, float kd);  // Set PID gains for motor specified by index
  void resetPID(uint8_t index, float xi, float xd);                       // Set PID state for motor specified by index
  void resetPID(uint8_t index);                                           // Reset PID state for motor specified by index
  void resetPIDs();                                                       // Reset all PIDs state
  
  void updateEncoder(uint8_t index);            // Update encoder value for motor specified by index
  void updateEncoders();                        // Update all motors' encoders
  void invertEncoder(uint8_t index, bool inv);  // Invert encoder count direction
  long getEncoder(uint8_t index);               // Get encoder value for motor specified by index 
  void setEncoder(uint8_t index, long value);   // Set encoder value for motor specified by index 
  void resetEncoder(uint8_t index);             // Reset encoder value for motor specified by index
  void resetEncoders();                         // Reset all motors' encoders to zero
  
  int16_t getPwm(uint8_t index);                // Get PWM value for motor specified by index 
  void setPwm(uint8_t index, int16_t pwm);      // Set PWM value for motor specified by index 
  void resetPwm(uint8_t index);                 // Reset all motors' PWM values to zero
  void resetPwms();                             // Reset all motors' PWM values to zero

  long getTarget(uint8_t index);                // Get encoder target value for motor specified by index 
  void setTarget(uint8_t index, long value);    // Set encoder target value for motor specified by index 
  void resetTarget(uint8_t index);              // Reset encoder target value for motor specified by index
  void resetTargets();                          // Reset all motors' encoder target to zero

  void updateEndStop(uint8_t index);            // Update endstop value for motor specified by index
  void updateEndStops();                        // Update all motors' endstop value
  bool getEndStop(uint8_t index);               // Get endstop value for motor specified by index

  int16_t getAction(uint8_t index);             // Get Action value for motor specified by index
  void setAction(uint8_t index, int16_t act);   // Set Action value for motor specified by index
  void resetAction(uint8_t index);              // Reset all motors' Action values to zero
  void resetActions();                          // Reset all motors' Action values to zero

  void enableMotors();                          // Enable all motors
  void disableMotors();                         // Disable all motors
  
  void update();                  // Update robot sensors readings
  void compute();                 // compute actions (final pwms values) according to robot status
  void actuate();                 // Apply computed actions (final pwms values)
  void reset();                   // Reset internal variables
  
private:
  PinControl &enable;     // Pin for motors enabling/disabling

  uint8_t size;           // Number of motors
  uint32_t ts;            // Time sampling in microseconds
  Motor **motors;         // Pointers array to Motors
  PID *pids;              // Pointer to motors' PIDs

  Status status;          // Robot status

  float *pids_div;        // Encoders error divider
  float *pids_kp;         // PID's P coefficient
  float *pids_ki;         // PID's I coefficient
  float *pids_kd;         // PID's D coefficient
  float *pids_sat;        // PID integral saturation
  float *pids_pole;       // PID derivative filter pole
  
  long    *mot_encs;      // Motors' encoders values
  int16_t *mot_pwms;      // Motors' PWMs current values
  long    *mot_refs;      // Motors' target encoders values
  bool    *mot_ends;      // Motors' endstop values
  int16_t *mot_acts;      // Motors' PWMs actuation values
};


#endif  // COMPONENTS_H