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

//STM32
#if defined(ARDUINO_ARCH_STM32) 
#define STM32
#endif


#include <Arduino.h>
#include <math.h>
#include "util.h"
#include "components.h"


//#if defined(MEGA)
#if defined(STM32)
#define DEBUG_COMMUNICATION   // Enable serial communication debugging
#define DEBUG_CHANNEL 0       // Choice serial channel for debugging
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
  #if defined(STM32)
    static Stream *hwserial;
  #else
    static HardwareSerial *hwserial;
  #endif

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

  
  struct Message {
    Message(Code code, uint8_t num);     // Costruttore principale
    Message(Code code) : Message(code, 0) {}  // Costruttore con num = 0

    Code getCode();        // Restituisce il codice del messaggio
    uint8_t getNum();      // Restituisce il numero associato
    bool setNum(uint8_t num);  // Imposta il numero e restituisce true se valido

    uint8_t size();        // Dimensione totale del messaggio
    uint8_t from(uint8_t *buffer);  // Decodifica da buffer
    uint8_t fill(uint8_t *buffer);  // Codifica in buffer

  protected:
    virtual uint8_t size_payload();        // Dimensione del payload
    virtual uint8_t from_payload(uint8_t *buffer); // Decodifica il payload
    virtual uint8_t fill_payload(uint8_t *buffer); // Codifica il payload

  private:
    Header header;       // Header del messaggio

    uint8_t size_header();       // Dimensione dell'header
    uint8_t from_header(uint8_t *buffer);  // Decodifica l'header
    uint8_t fill_header(uint8_t *buffer);  // Codifica l'header
  };



  struct MsgIDLE : public Message {
    MsgIDLE() : Message(Code::IDLE) {}           // Costruttore predefinito
    MsgIDLE(uint8_t num) : Message(Code::IDLE, num) {} // Costruttore con num

    uint8_t getCount();      // Restituisce il numero di elementi
    bool setCount(uint8_t count); // Imposta il numero di elementi

  protected:
    uint8_t size_payload();          // Dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica payload
    uint8_t fill_payload(uint8_t *buffer); // Codifica payload
  };


  struct MsgPWM : public Message {
    MsgPWM() : Message(Code::PWM) {}           // Costruttore predefinito
    MsgPWM(uint8_t num) : Message(Code::PWM, num) {} // Costruttore con num

    uint8_t getCount();     // Restituisce il numero di canali PWM
    int16_t getPwm(uint8_t index); // Restituisce il valore PWM di un canale

    bool setCount(uint8_t count);      // Imposta il numero di canali PWM
    bool setPwm(uint8_t index, int16_t value); // Imposta il valore PWM di un canale

  protected:
    uint8_t size_payload();          // Dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica payload
    uint8_t fill_payload(uint8_t *buffer); // Codifica payload

  private:
    int16_t pwms[8];  // Array per memorizzare i valori PWM (max 8 canali)
  };


  struct MsgREF : public Message {
    MsgREF() : Message(Code::REF) {}            // Costruttore predefinito
    MsgREF(uint8_t num) : Message(Code::REF, num) {} // Costruttore con numero identificativo

    uint8_t getCount();      // Restituisce il numero di encoder monitorati
    int16_t getDeltaEnc(uint8_t index); // Restituisce la variazione dell'encoder all'indice specificato

    bool setCount(uint8_t count);      // Imposta il numero di encoder monitorati
    bool setDeltaEnc(uint8_t index, int16_t value); // Imposta la variazione dell'encoder all'indice specificato

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer

  private:
    int16_t deltas[8];  // Array che memorizza le variazioni degli encoder (max 8 valori)
  };



  struct MsgROBOT : public Message {
    MsgROBOT() : Message(Code::ROBOT) {}            // Costruttore predefinito
    MsgROBOT(uint8_t num) : Message(Code::ROBOT, num) {} // Costruttore con numero identificativo

    uint8_t getCount();         // Restituisce il numero di elementi nel messaggio
    uint32_t getTimeSampling(); // Restituisce il tempo di campionamento in microsecondi
    uint8_t getAllowedTicks();  // Restituisce il numero massimo di tick consentiti

    bool setCount(uint8_t count);          // Imposta il numero di elementi nel messaggio
    bool setTimeSampling(uint32_t value);  // Imposta il tempo di campionamento
    bool setAllowedTicks(uint8_t value);   // Imposta il numero massimo di tick consentiti

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer

  private:
    uint32_t timesampling_us; // Tempo di campionamento in microsecondi
    uint8_t allowed_ticks;    // Numero massimo di tick consentiti
  };



  struct MsgMOTOR : public Message {
    MsgMOTOR() : Message(Code::MOTOR) {}            // Costruttore predefinito
    MsgMOTOR(uint8_t num) : Message(Code::MOTOR, num) {} // Costruttore con numero identificativo
    
    uint8_t getIndex();            // Restituisce l'indice del motore
    bool getChangeEncoder();        // Verifica se c'è stato un cambio di encoder
    bool getInvertSpinDir();        // Verifica se la direzione di rotazione è invertita
    bool getChangeSpinDir();        // Verifica se la direzione di rotazione è cambiata
    int8_t getSpinDirection();      // Restituisce la direzione di rotazione
    bool getInvertEncDir();         // Verifica se la direzione dell'encoder è invertita
    bool getChangeEncDir();         // Verifica se la direzione dell'encoder è cambiata
    int8_t getEncDirection();       // Restituisce la direzione dell'encoder
    int32_t getEncoderValue();      // Restituisce il valore dell'encoder

    bool setIndex(uint8_t index);        // Imposta l'indice del motore
    bool setChangeEncoder(bool value);   // Imposta il cambio di encoder
    bool setInvertSpinDir(bool value);   // Imposta l'inversione della direzione di rotazione
    bool setChangeSpinDir(bool value);   // Imposta il cambio della direzione di rotazione
    bool setSpinDirection(int8_t dir);   // Imposta la direzione di rotazione
    bool setInvertEncDir(bool value);    // Imposta l'inversione della direzione dell'encoder
    bool setChangeEncDir(bool value);    // Imposta il cambio della direzione dell'encoder
    bool setEncDirection(int8_t dir);    // Imposta la direzione dell'encoder
    bool setEncoderValue(int32_t value); // Imposta il valore dell'encoder

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer

  private:
    uint8_t flags;   // Byte di flag per indicare stati e modifiche
    int32_t encoder; // Valore dell'encoder associato al motore
  };



  struct MsgPID : public Message {
    MsgPID() : Message(Code::PID) {}            // Costruttore predefinito
    MsgPID(uint8_t num) : Message(Code::PID, num) {} // Costruttore con numero identificativo
    
    uint8_t getIndex();        // Restituisce l'indice del PID
    float getPidDiv();         // Restituisce il valore di divisione del PID
    float getPidKp();          // Restituisce il guadagno proporzionale (Kp)
    float getPidKi();          // Restituisce il guadagno integrale (Ki)
    float getPidKd();          // Restituisce il guadagno derivativo (Kd)
    float getPidSat();         // Restituisce il valore di saturazione
    float getPidPole();        // Restituisce il valore del polo

    bool setIndex(uint8_t index);   // Imposta l'indice del PID
    bool setPidDiv(float value);    // Imposta il valore di divisione del PID
    bool setPidKp(float value);     // Imposta il guadagno proporzionale (Kp)
    bool setPidKi(float value);     // Imposta il guadagno integrale (Ki)
    bool setPidKd(float value);     // Imposta il guadagno derivativo (Kd)
    bool setPidSat(float value);    // Imposta il valore di saturazione
    bool setPidPole(float value);   // Imposta il valore del polo

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer

  private:
    float div;    // Valore di divisione del PID
    float kp;     // Guadagno proporzionale (Kp)
    float ki;     // Guadagno integrale (Ki)
    float kd;     // Guadagno derivativo (Kd)
    float sat;    // Valore di saturazione
    float pole;   // Valore del polo
  };



  struct MsgACKC : public Message {
    MsgACKC() : Message(Code::ACKC) {}            // Costruttore predefinito
    MsgACKC(uint8_t num) : Message(Code::ACKC, num) {} // Costruttore con numero identificativo

    uint8_t getCount();           // Restituisce il numero di encoder monitorati
    bool getEndStop(uint8_t index); // Restituisce lo stato dell'interruttore di fine corsa per l'indice specificato
    int16_t getDeltaEnc(uint8_t index); // Restituisce la variazione dell'encoder per l'indice specificato
    
    bool setCount(uint8_t count);          // Imposta il numero di encoder monitorati
    bool setEndStop(uint8_t index, bool value); // Imposta lo stato dell'interruttore di fine corsa per l'indice specificato
    bool setDeltaEnc(uint8_t index, int16_t value); // Imposta la variazione dell'encoder per l'indice specificato

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer

  private:
    uint8_t endstops;    // Byte che memorizza lo stato degli interruttori di fine corsa
    int16_t deltas[8];   // Array che memorizza le variazioni degli encoder (max 8 valori)
  };



  struct MsgACKS : public Message {
    MsgACKS() : Message(Code::ACKS) {}            // Costruttore predefinito
    MsgACKS(uint8_t num) : Message(Code::ACKS, num) {} // Costruttore con numero identificativo

    uint8_t getCount();       // Restituisce il numero di elementi monitorati
    uint8_t getIndex();       // Restituisce l'indice specifico dell'elemento

    bool setCount(uint8_t count);   // Imposta il numero di elementi monitorati
    bool setIndex(uint8_t index);   // Imposta l'indice specifico dell'elemento

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer
  };



  struct MsgERROR : public Message {
    MsgERROR() : Message(Code::ERROR) {}            // Costruttore predefinito
    MsgERROR(uint8_t num) : Message(Code::ERROR, num) {} // Costruttore con numero identificativo

    uint8_t getCount();       // Restituisce il numero di errori monitorati
    uint8_t getIndex();       // Restituisce l'indice specifico dell'errore

    bool setCount(uint8_t count);   // Imposta il numero di errori monitorati
    bool setIndex(uint8_t index);   // Imposta l'indice specifico dell'errore

  protected:
    uint8_t size_payload();          // Restituisce la dimensione del payload
    uint8_t from_payload(uint8_t *buffer); // Decodifica il payload dal buffer
    uint8_t fill_payload(uint8_t *buffer); // Scrive il payload nel buffer
  };


  static bool peek(Header *hdr, Code target, Timer *timeout_us = NULL);
  static bool peek(Header *hdr, Timer *timeout_us = NULL);
  static bool rcv(Message *msg, Timer *timeout_us = NULL);
  static bool snd(Message *msg);

  static bool transmit(Message *sndMsg, Message *rcvMsg, Timer *timeout = NULL);
};


class RobotComm {
public:
  RobotComm(Robot &robot, uint8_t channel); // Costruttore che inizializza il robot e il canale di comunicazione
  ~RobotComm();                             // Distruttore

  void setPinComm(PinControl *pin);         // Imposta il pin di comunicazione per il debug
  void setPinCtrl(PinControl *pin);         // Imposta il pin di controllo per il debug

  void cycle(uint32_t time_us);             // Ciclo di comunicazione con tempo definito (microsecondi)
  void cycle();                             // Ciclo di comunicazione senza tempo definito

private:
  Robot &robot;           // Riferimento al robot controllato dalla comunicazione seriale
  uint8_t channel;        // Canale seriale utilizzato per la comunicazione
  PinControl *pin_comm;   // Pin di debug per il timing della comunicazione
  PinControl *pin_ctrl;   // Pin di debug per il timing del controllo

  Timer timer;            // Timer interno per la comunicazione seriale durante le operazioni di controllo
  Timer timeout;          // Timer interno per il timeout della comunicazione seriale in ricezione
  long *encoders_rcv;     // Valori cumulativi degli encoder ricevuti
  long *encoders_snd;     // Valori cumulativi degli encoder inviati
  uint8_t ticks_allowed;  // Numero di tick consentiti in modalità controllo senza un nuovo controllo
  uint8_t ticks_used;     // Numero di tick utilizzati in modalità controllo senza un nuovo controllo
};

#if defined(DEBUG_COMMUNICATION)
class DebugComm {
  static String indent(uint8_t level, uint8_t size); // Funzione di indentazione per la stampa dei messaggi di debug

public:
  DebugComm() = delete;   // Disabilita il costruttore predefinito
  ~DebugComm() = delete;   // Disabilita il distruttore

  static void print(Communication::Header *header, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa l'intestazione del messaggio
  static void print(Communication::Message *message, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa un messaggio
  static void print(Communication::MsgIDLE *msg_idle, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa il messaggio MsgIDLE
  static void print(Communication::MsgPWM *msg_pwm, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);   // Stampa il messaggio MsgPWM
  static void print(Communication::MsgREF *msg_ref, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);   // Stampa il messaggio MsgREF
  static void print(Communication::MsgROBOT *msg_robot, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa il messaggio MsgROBOT
  static void print(Communication::MsgMOTOR *msg_motor, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa il messaggio MsgMOTOR
  static void print(Communication::MsgPID *msg_pid, String title, uint8_t indent_level = 0, uint8_t indent_size = 2);   // Stampa il messaggio MsgPID
  static void print(Communication::MsgACKC *msg_ackc, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa il messaggio MsgACKC
  static void print(Communication::MsgACKS *msg_acks, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa il messaggio MsgACKS
  static void print(Communication::MsgERROR *msg_error, String title, uint8_t indent_level = 0, uint8_t indent_size = 2); // Stampa il messaggio MsgERROR
};
#endif

#endif  // COMMUNICATION_H