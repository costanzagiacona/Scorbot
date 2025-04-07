#include "components.h"
#include <HardwareTimer.h>
#include "Pid.h"


// ==================================================
// PinControl
// ==================================================
/*Questa classe gestisce un pin di output, con la possibilità di 
controllare lo stato del pin (HIGH o LOW) o
utilizzare il controllo PWM (Pulse Width Modulation)*/

//Costruttore della classe PinControl che imposta il pin come output e inizializza i limiti a 0.0
PinControl::PinControl(uint8_t pin) {
  this->pin = pin;                 // Salva il pin selezionato
  pinMode(pin, OUTPUT);            // Imposta il pin come un'uscita
  setLimits(0.0, 0.0);             // Imposta i limiti a 0.0
}

// Costruttore della classe PinControl con la possibilità di specificare i limiti v1 e v2
PinControl::PinControl(uint8_t pin, float v1, float v2){
  this->pin = pin;                 // Salva il pin selezionato
  pinMode(pin, OUTPUT);            // Imposta il pin come un'uscita
  setLimits(v1, v2);               // Imposta i limiti ai valori passati
}

// Restituisce il numero del pin
uint8_t PinControl::getPin(){
  return this->pin;
}

// Imposta i limiti v1 e v2 per il controllo del pin
void PinControl::setLimits(float v1, float v2){
  this->v1 = v1;                   // Assegna il valore di v1
  this->v2 = v2;                   // Assegna il valore di v2
}

// Imposta lo stato del pin (HIGH o LOW)
void PinControl::set(bool state){
  #if defined(PIN_CONTROL_STORE_VALUES)
  this->set_ = state;              // Memorizza lo stato, se è abilitato il flag PIN_CONTROL_STORE_VALUES
  #endif
  digitalWrite(pin, state ? HIGH : LOW);  // Imposta il pin a HIGH o LOW a seconda dello stato
}


// Imposta un valore PWM (duty cycle) sul pin
void PinControl::pwm(uint8_t pwm){
  #if defined(PIN_CONTROL_STORE_VALUES)
  this->pwm_ = pwm;                // Memorizza il valore PWM, se è abilitato il flag PIN_CONTROL_STORE_VALUES
  #endif
  analogWrite(pin, pwm);           // Imposta il valore PWM sul pin
}

// Controlla il pin in base al valore passato (mappato nell'intervallo 0-255)
void PinControl::control(float value){
  pwm(remap3(value, v1, v2, 0l, 255l, true));  // Mappa il valore e lo invia come PWM
}

#if defined(PIN_CONTROL_EXTRA_FEATURES)
// Funzione di feedback che usa un PID per correggere l'errore
void PinControl::feedback(float error){
  if(pid != NULL) control(pid->evolve(error));  // Se il PID è definito, aggiorna il controllo con l'errore
}

// Funzione di feedback senza errore (usa il PID se definito)
void PinControl::feedback(){
  if(pid != NULL) control(pid->output());  // Se il PID è definito, aggiorna il controllo con l'output del PID
}

// Imposta un PID per il controllo
void PinControl::setPID(PID *pid){
  this->pid = pid;  // Salva il riferimento al PID
}

// Restituisce il PID attuale
PID* PinControl::getPID(){
  return this->pid;  // Restituisce il PID
}
#endif

#if defined(PIN_CONTROL_STORE_VALUES)
// Restituisce l'ultimo stato del pin (memorizzato)
bool PinControl::last_set(){
  return this->set_;  // Restituisce l'ultimo stato memorizzato
}

// Restituisce l'ultimo valore PWM (memorizzato)
uint8_t PinControl::last_pwm(){
  return this->pwm_;  // Restituisce l'ultimo valore PWM memorizzato
}

// Restituisce l'ultimo valore di controllo mappato
float PinControl::last_control(){
  return remap2(this->pwm_, 0l, 255l, v1, v2);  // Mappa l'ultimo valore PWM all'intervallo v1-v2
}
#endif

// ==================================================
// PinMeasure
// ==================================================

// Costruttore della classe PinMeasure che imposta il pin come input e definisce la modalità pull-up opzionale
PinMeasure::PinMeasure(uint8_t pin, bool pullup){
  this->pin = pin;                 // Salva il pin selezionato
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);  // Imposta il pin come input con o senza pull-up
  setLimits(0.0, 0.0);             // Imposta i limiti a 0.0
}

// Costruttore della classe PinMeasure con la possibilità di specificare i limiti v1 e v2
PinMeasure::PinMeasure(uint8_t pin, float v1, float v2, bool pullup){
  this->pin = pin;                 // Salva il pin selezionato
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);  // Imposta il pin come input con o senza pull-up
  setLimits(v1, v2);               // Imposta i limiti ai valori passati
}

// Restituisce il numero del pin
uint8_t PinMeasure::getPin(){
  return this->pin;
}

// Imposta i limiti v1 e v2 per la misurazione
void PinMeasure::setLimits(float v1, float v2){
  this->v1 = v1;                   // Assegna il valore di v1
  this->v2 = v2;                   // Assegna il valore di v2
}

// Restituisce lo stato del pin (HIGH o LOW)
bool PinMeasure::state(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  this->state_ = digitalRead(pin) == HIGH;  // Memorizza lo stato, se è abilitato il flag PIN_MEASURE_STORE_VALUES
  return this->state_;  // Restituisce lo stato memorizzato
  #else
  return digitalRead(pin) == HIGH;  // Restituisce lo stato del pin
  #endif
}

// Restituisce il valore letto dal pin analogico
uint16_t PinMeasure::value(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  this->value_ = analogRead(pin);  // Memorizza il valore analogico, se è abilitato il flag PIN_MEASURE_STORE_VALUES
  return this->value_;  // Restituisce il valore memorizzato
  #else
  return analogRead(pin);  // Restituisce il valore analogico
  #endif
}

// Misura il valore letto, mappandolo nell'intervallo v1-v2
float PinMeasure::measure(){
  return remap2((long) value(), 0l, 1023l, v1, v2, false);  // Mappa il valore letto nell'intervallo v1-v2
}

#if defined(PIN_MEASURE_EXTRA_FEATURES)
// Filtra il valore misurato, se è stato definito un filtro
float PinMeasure::filter(bool readonly){
  return ((fil != NULL) ? (readonly ? fil->output() : fil->evolve(measure())) : measure());  // Applica il filtro, se presente
}

// Filtra il valore misurato (senza argomenti, applica il filtro evolutivo)
float PinMeasure::filter(){
  return ((fil != NULL) ? fil->evolve(measure()) : measure());  // Applica il filtro, se presente
}

// Imposta un filtro per la misurazione
void PinMeasure::setFilter(Filter *filter){
  this->fil = filter;  // Salva il filtro
}

// Restituisce il filtro associato alla misurazione
Filter* PinMeasure::getFilter(){
  return this->fil;  // Restituisce il filtro
}
#endif

#if defined(PIN_MEASURE_STORE_VALUES)
// Restituisce l'ultimo stato memorizzato del pin
bool PinMeasure::last_state(){
  return this->state_;  // Restituisce l'ultimo stato memorizzato
}

// Restituisce l'ultimo valore analogico memorizzato
uint16_t PinMeasure::last_value(){
  return this->value_;  // Restituisce l'ultimo valore analogico memorizzato
}

// Restituisce l'ultima misurazione memorizzata
float PinMeasure::last_measure(){
  return remap2((long) this->value_, 0l, 1023l, v1, v2);  // Mappa l'ultimo valore analogico nell'intervallo v1-v2
}
#endif


// ==================================================
// Motor
// ==================================================

Motor::Motor(PinControl &INA, PinControl &INB, PinControl &PWM, PinMeasure &CHA, PinMeasure &CHB, PinMeasure &END) 
  : pin_INA(INA), pin_INB(INB), pin_PWM(PWM), pin_CHA(CHA), pin_CHB(CHB), pin_END(END){
  readEncoder();
}

Motor::~Motor() {}

void Motor::invertEncoder(bool invert){
  this->encoder_invert = invert;
}

long Motor::getEncoder(){
  return this->encoder;
}

void Motor::setEncoder(long value){
  this->encoder = value;
}

void Motor::readEncoder(){
  enc_A = pin_CHA.state();
  enc_B = pin_CHB.state();
}

void Motor::updateEncoder(){
  bool old_A = enc_A;
  readEncoder();
  if(old_A != enc_A){
    if(enc_B == enc_A) {
      encoder_invert ? encoder-- : encoder++;
    } else {
      encoder_invert ? encoder++ : encoder--;
    }
  }
}

void Motor::invertMotor(bool invert){
  this->motor_invert = invert;
}

void Motor::driveMotor(int16_t spwm){
  OperatingMode mode = OperatingMode::BRAKE_GND;
  spwm = constrain(spwm, -255, 255);

  if(spwm > 0) {
    mode = motor_invert ? OperatingMode::SPIN_CCW : OperatingMode::SPIN_CW;
  } else if (spwm < 0) {
    mode = motor_invert ? OperatingMode::SPIN_CW : OperatingMode::SPIN_CCW;
  } else {
    mode = OperatingMode::BRAKE_GND;
  }
  
  switch(mode){
    case OperatingMode::BRAKE_GND:
      pin_INA.set(false);
      pin_INB.set(false);
      break;
    case OperatingMode::SPIN_CCW:
      pin_INA.set(false);
      pin_INB.set(true);
      break;
    case OperatingMode::SPIN_CW:
      pin_INA.set(true);
      pin_INB.set(false);
      break;
    case OperatingMode::BRAKE_VCC:
      pin_INA.set(true);
      pin_INB.set(true);
      break;
  }

  pin_PWM.pwm((uint8_t) abs(spwm));
}

bool Motor::isInEndStop(){
  return pin_END.state();
}

