#include "components.h"
#include <HardwareTimer.h>
#include"Pid.h"


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


// ==================================================
// Robot
// ==================================================

Robot::Robot(PinControl &enable, uint8_t size, uint32_t ts_us)
  : enable(enable) {
  this->size = size;
  this->ts = ts_us;
  this->motors = (Motor**) malloc(size * sizeof(Motor*));
  this->pids = (PID*) malloc(size * sizeof(PID));
  
  this->status = Status::Idle;

  this->pids_div = (float *) malloc(size * sizeof(float));
  this->pids_kp = (float *) malloc(size * sizeof(float));
  this->pids_ki = (float *) malloc(size * sizeof(float));
  this->pids_kd = (float *) malloc(size * sizeof(float));
  this->pids_sat = (float *) malloc(size * sizeof(float));
  this->pids_pole = (float *) malloc(size * sizeof(float));

  this->mot_encs = (long *) malloc(size * sizeof(long));
  this->mot_pwms = (int16_t *) malloc(size * sizeof(int16_t));
  this->mot_refs = (long *) malloc(size * sizeof(long));
  this->mot_ends = (bool *) malloc(size * sizeof(bool));
  this->mot_acts = (int16_t *) malloc(size * sizeof(int16_t));
  
  for(int i = 0; i < size; i++)
  {
    this->pids_div[i] = 1.0;
    this->pids_kp[i] = 0.0;
    this->pids_ki[i] = 0.0;
    this->pids_kd[i] = 0.0;
    this->pids_sat[i] = 0.0;
    this->pids_pole[i] = 0.0;

    this->mot_encs[i] = 0;
    this->mot_pwms[i] = 0;
    this->mot_refs[i] = 0;
    this->mot_ends[i] = false;
    this->mot_acts[i] = 0;
  }
  
  setStatus(Status::Idle);
}

Robot::~Robot() {
  free(this->motors);
  free(this->pids);
  free(this->pids_div);
  free(this->pids_kp);
  free(this->pids_ki);
  free(this->pids_kd);
  free(this->pids_sat);
  free(this->pids_pole);
  free(this->mot_encs);
  free(this->mot_pwms);
  free(this->mot_refs);
  free(this->mot_ends);
  free(this->mot_acts);
}

int Robot::getSize(){
  return this->size;
}

Robot::Status Robot::getStatus(){
  return this->status;
}

bool Robot::setStatus(Status status, bool reset){
  if(this->status != status || reset){
    resetPIDs();
    resetActions();
    actuate();
    this->status = status;
    return true;
  } else {
    return false;
  }
}

uint32_t Robot::getTimeSampling(){
  return this->ts;
}

void Robot::setTimeSampling(uint32_t ts_us){
  this->ts = ts_us;
  for(uint8_t i = 0; i < size; i++){
    initPID(i, pids_sat[i], pids_pole[i]);
    setupPID(i, pids_div[i], pids_kp[i], pids_ki[i], pids_kd[i]);
  }
}

void Robot::setMotor(uint8_t index, Motor &motor){
  if(index < size) this->motors[index] = &motor;    
}

void Robot::invertMotor(uint8_t index, bool inv){
  if(index < size) this->motors[index]->invertMotor(inv);
}

void Robot::initPID(uint8_t index, float sat, float pole){
  if(index < size){
    this->pids_sat[index] = sat;
    this->pids_pole[index] = pole;
    this->pids[index].init((float) ts / 1000000.0, 0.0, sat, 0.0, 0.0, 0.0, pole, true);
  }
}

void Robot::setupPID(uint8_t index, float div, float kp, float ki, float kd){
  if(index < size){
    this->pids_div[index] = div;
    this->pids_kp[index] = kp;
    this->pids_ki[index] = ki;
    this->pids_kd[index] = kd;
    this->pids[index].setup(kp, ki, kd);
  }
}

void Robot::resetPID(uint8_t index, float xi, float xd){
  if(index < size) this->pids[index].reset(xi, xd);
}

void Robot::resetPID(uint8_t index){
  resetPID(index, 0.0, 0.0);
}

void Robot::resetPIDs(){
  for(uint8_t i = 0; i < size; i++) resetPID(i);
}

void Robot::updateEncoder(uint8_t index){
  if(index < size) {
    motors[index]->updateEncoder();
    mot_encs[index] = motors[index]->getEncoder();
  } 
}

void Robot::updateEncoders(){
  for(int i = 0; i < size; i++) updateEncoder(i);
}

void Robot::invertEncoder(uint8_t index, bool inv){
  if(index < size) this->motors[index]->invertEncoder(inv);
}

long Robot::getEncoder(uint8_t index){
  return (index < size) ? mot_encs[index] : 0;
}

void Robot::setEncoder(uint8_t index, long value){
  if(index < size) {
    motors[index]->setEncoder(value);
    mot_encs[index] = value;
  }
}

void Robot::resetEncoder(uint8_t index){
  if(index < size) setEncoder(index, 0);
}

void Robot::resetEncoders(){
  for(int i = 0; i < size; i++) resetEncoder(i);
}

int16_t Robot::getPwm(uint8_t index){
  return (index < size) ? mot_pwms[index] : 0;
}

void Robot::setPwm(uint8_t index, int16_t pwm){
  if(index < size) mot_pwms[index] = constrain(pwm, -255, 255);
}

void Robot::resetPwm(uint8_t index){
  setPwm(index, 0);
}

void Robot::resetPwms(){
  for(int i = 0; i < size; i++) setPwm(i, 0);
}

long Robot::getTarget(uint8_t index){
  return (index < size) ? mot_refs[index] : 0;
}

void Robot::setTarget(uint8_t index, long value){
  if(index < size) mot_refs[index] = value;
}

void Robot::resetTarget(uint8_t index){
  setTarget(index, 0);
}

void Robot::resetTargets(){
  for(uint8_t i = 0; i < size; i++) resetTarget(i);
}

void Robot::updateEndStop(uint8_t index){
  if(index < size) mot_ends[index] = motors[index]->isInEndStop();
}

void Robot::updateEndStops(){
  for(uint8_t i = 0; i < size; i++) updateEndStop(i);
}

bool Robot::getEndStop(uint8_t index){
  return (index < size) ? mot_ends[index] : false;
}

int16_t Robot::getAction(uint8_t index){
  return (index < size) ? mot_acts[index] : 0;
}

void Robot::setAction(uint8_t index, int16_t act){
  if(index < size)  mot_acts[index] = constrain(act, -255, 255);
}

void Robot::resetAction(uint8_t index){
  setAction(index, 0);
}

void Robot::resetActions(){
  for(uint8_t i = 0; i < size; i++) resetAction(i);
}

void Robot::enableMotors(){
  setStatus(Status::Idle, true);
  enable.set(true);
}

void Robot::disableMotors(){
  setStatus(Status::Idle, true);
  enable.set(false);
}

void Robot::update(){
  updateEncoders();
  updateEndStops();
}

void Robot::compute(){
  switch(status){
    case Status::Idle:
      for(uint8_t i = 0; i < size; i++) mot_acts[i] = 0;
      break;

    case Status::Pwm:
      for(uint8_t i = 0; i < size; i++) mot_acts[i] = mot_pwms[i];
      break;

    case Status::Ref:
      for(uint8_t i = 0; i < size; i++) mot_acts[i] = pids[i].evolve(((float) (mot_refs[i] - mot_encs[i])) / ((pids_div[i] == 0) ? 1.0 : pids_div[i]));
      break;
  }
}

void Robot::actuate(){
  for(int i = 0; i < size; i++) motors[i]->driveMotor(mot_acts[i]);
}

void Robot::reset(){
  resetPIDs();
  resetEncoders();
  resetPwms();
  resetTargets();
  resetActions();
  setStatus(Status::Idle, true);
}