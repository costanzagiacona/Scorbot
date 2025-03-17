#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif


#include <Arduino.h>
#include <math.h>
#include "utils.h"
#include "components.h"


#if defined(MEGA)
//#define DEBUG_COMMUNICATION   // Enable serial communication debugging
#define DEBUG_CHANNEL 1       // Choice serial channel for debugging
#define DEBUG_LOW             // Debug low level data exchange
#define DEBUG_HIGH            // Debug high level data exchange
#define DEBUG_DATA            // Debug content of data exchange

#if DEBUG_CHANNEL == 0
#define DEBUG_SERIAL Serial
#elif DEBUG_CHANNEL == 1
#define DEBUG_SERIAL Serial1
#elif DEBUG_CHANNEL == 2
#define DEBUG_SERIAL Serial2
#elif DEBUG_CHANNEL == 3
#define DEBUG_SERIAL Serial3
#endif

#endif


// Serial Communication Protocol (static)
class Communication {
  static HardwareSerial *hwserial;

public:
  Communication() = delete;
  ~Communication() = delete;
  
  static void channel(uint8_t index);
  static void flush(bool input = true);
  
  // Codes
  enum class Code : uint8_t{
    IDLE  = 0b00001,   // 1
    PWM   = 0b00010,   // 2
    REF   = 0b00011,   // 3
    ROBOT = 0b10000,   // 16
    MOTOR = 0b10001,   // 17
    PID   = 0b10010,   // 18
    ACKC  = 0b11000,   // 24
    ACKS  = 0b11001,   // 25
    ERROR = 0b11111    // 31
  };

  static bool convert(uint8_t value, Code &code);
  static bool isCtrl(Code code);
  static bool isSetup(Code code);
  static bool isAck(Code code);
  static bool isError(Code code);

  struct Header{
    Header(Code code, uint8_t num);
    Header() : Header(Code::IDLE, 0) {}

    Code getCode();
    uint8_t getNum();

    bool setCode(Code code);
    bool setNum(uint8_t num);

    bool parse(uint8_t byte);
    uint8_t byte();

    uint8_t size();
    uint8_t from(uint8_t *buffer);
    uint8_t fill(uint8_t *buffer);

  private:
    Code code;
    uint8_t num;
  };

  
  struct Message{
    Message(Code code, uint8_t num);
    Message(Code code) : Message(code, 0) {}

    Code getCode();
    uint8_t getNum();

    bool setNum(uint8_t num);

    uint8_t size();
    uint8_t from(uint8_t *buffer);
    uint8_t fill(uint8_t *buffer);

  protected:
    virtual uint8_t size_payload();
    virtual uint8_t from_payload(uint8_t *buffer);
    virtual uint8_t fill_payload(uint8_t *buffer);

  private:
    Header header;

    uint8_t size_header();
    uint8_t from_header(uint8_t *buffer);
    uint8_t fill_header(uint8_t *buffer);
  };


  struct MsgIDLE : public Message{
    MsgIDLE() : Message(Code::IDLE) {}
    MsgIDLE(uint8_t num) : Message(Code::IDLE, num) {}

    uint8_t getCount();
    
    bool setCount(uint8_t count);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  struct MsgPWM : public Message{
    MsgPWM() : Message(Code::PWM) {}
    MsgPWM(uint8_t num) : Message(Code::PWM, num) {}

    uint8_t getCount();
    int16_t getPwm(uint8_t index);

    bool setCount(uint8_t count);
    bool setPwm(uint8_t index, int16_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
    
  private:
    int16_t pwms[8];
  };


  struct MsgREF : public Message{
    MsgREF() : Message(Code::REF) {}
    MsgREF(uint8_t num) : Message(Code::REF, num) {}

    uint8_t getCount();
    int16_t getDeltaEnc(uint8_t index);

    bool setCount(uint8_t count);
    bool setDeltaEnc(uint8_t index, int16_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    int16_t deltas[8];
  };


  struct MsgROBOT : public Message{
    MsgROBOT() : Message(Code::ROBOT) {}
    MsgROBOT(uint8_t num) : Message(Code::ROBOT, num) {}

    uint8_t getCount();
    uint32_t getTimeSampling();
    uint8_t getAllowedTicks();

    bool setCount(uint8_t count);
    bool setTimeSampling(uint32_t value);
    bool setAllowedTicks(uint8_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint32_t timesampling_us;
    uint8_t allowed_ticks;
  };


  struct MsgMOTOR : public Message{
    MsgMOTOR() : Message(Code::MOTOR) {}
    MsgMOTOR(uint8_t num) : Message(Code::MOTOR, num) {}
    
    uint8_t getIndex();
    bool getChangeEncoder();
    bool getInvertSpinDir();
    bool getChangeSpinDir();
    int8_t getSpinDirection();
    bool getInvertEncDir();
    bool getChangeEncDir();
    int8_t getEncDirection();
    int32_t getEncoderValue();

    bool setIndex(uint8_t index);
    bool setChangeEncoder(bool value);
    bool setInvertSpinDir(bool value);
    bool setChangeSpinDir(bool value);
    bool setSpinDirection(int8_t dir);
    bool setInvertEncDir(bool value);
    bool setChangeEncDir(bool value);
    bool setEncDirection(int8_t dir);
    bool setEncoderValue(int32_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint8_t flags;
    int32_t encoder;
  };


  struct MsgPID : public Message{
    MsgPID() : Message(Code::PID) {}
    MsgPID(uint8_t num) : Message(Code::PID, num) {}
    
    uint8_t getIndex();
    float getPidDiv();
    float getPidKp();
    float getPidKi();
    float getPidKd();
    float getPidSat();
    float getPidPole();

    bool setIndex(uint8_t index);
    bool setPidDiv(float value);
    bool setPidKp(float value);
    bool setPidKi(float value);
    bool setPidKd(float value);
    bool setPidSat(float value);
    bool setPidPole(float value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    float div;
    float kp;
    float ki;
    float kd;
    float sat;
    float pole;
  };


  struct MsgACKC : public Message{
    MsgACKC() : Message(Code::ACKC) {}
    MsgACKC(uint8_t num) : Message(Code::ACKC, num) {}

    uint8_t getCount();
    bool getEndStop(uint8_t index);
    int16_t getDeltaEnc(uint8_t index);
    
    bool setCount(uint8_t count);
    bool setEndStop(uint8_t index, bool value);
    bool setDeltaEnc(uint8_t index, int16_t value);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);

  private:
    uint8_t endstops;
    int16_t deltas[8];
  };


  struct MsgACKS : public Message{
    MsgACKS() : Message(Code::ACKS) {}
    MsgACKS(uint8_t num) : Message(Code::ACKS, num) {}

    uint8_t getCount();
    uint8_t getIndex();
    
    bool setCount(uint8_t count);
    bool setIndex(uint8_t index);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  struct MsgERROR : public Message{
    MsgERROR() : Message(Code::ERROR) {}
    MsgERROR(uint8_t num) : Message(Code::ERROR, num) {}

    uint8_t getCount();
    uint8_t getIndex();
    
    bool setCount(uint8_t count);
    bool setIndex(uint8_t index);

  protected:
    uint8_t size_payload();
    uint8_t from_payload(uint8_t *buffer);
    uint8_t fill_payload(uint8_t *buffer);
  };


  static bool peek(Header *hdr, Code target, Timer *timeout_us = NULL);
  static bool peek(Header *hdr, Timer *timeout_us = NULL);
  static bool rcv(Message *msg, Timer *timeout_us = NULL);
  static bool snd(Message *msg);

  static bool transmit(Message *sndMsg, Message *rcvMsg, Timer *timeout = NULL);
};


class RobotComm {
public:
  RobotComm(Robot &robot, uint8_t channel);
  ~RobotComm();

  void setPinComm(PinControl *pin);
  void setPinCtrl(PinControl *pin);

  void cycle(uint32_t time_us);
  void cycle();

private:
  Robot &robot;           // Robot controlled by this serial communication
  uint8_t channel;        // Serial channel used for communication
  PinControl *pin_comm;   // Debug pin for communication timing
  PinControl *pin_ctrl;   // Debug pin for control timing

  Timer timer;            // Internal timer for serial communication during control operations
  Timer timeout;          // Internal timer for serial communication receiving timeouts
  long *encoders_rcv;     // Cumulative encoders values received
  long *encoders_snd;     // Cumulative encoders values sent
  uint8_t ticks_allowed;  // Allowed ticks in control mode without a new control
  uint8_t ticks_used;     // Used ticks in control mode without a new control
};


#if defined(DEBUG_COMMUNICATION)
class DebugComm {
  static String indent(uint8_t level, uint8_t size);

public:
  DebugComm() = delete;
  ~DebugComm() = delete;

  static void print(Communication::Header      *header, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::Message    *message, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgIDLE   *msg_idle, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgPWM     *msg_pwm, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgREF     *msg_ref, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgROBOT *msg_robot, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgMOTOR *msg_motor, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgPID     *msg_pid, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgACKC   *msg_ackc, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgACKS   *msg_acks, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
  static void print(Communication::MsgERROR *msg_error, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);
};
#endif

#endif  // COMMUNICATION_H