#include "components.h"


// ==================================================
// PWMfreq
// ==================================================

#if defined(UNO) || defined(MEGA)

#if defined(UNO)
void PWMfreq::set(UnoTimer0 freq){
  TCCR0B = (TCCR0B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(UnoTimer1 freq){
  TCCR1B = (TCCR1B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(UnoTimer2 freq){
  TCCR2B = (TCCR2B & 0b11111000) | ((uint8_t) freq);
}
#endif

#if defined(MEGA)
void PWMfreq::set(MegaTimer0 freq){
  TCCR0B = (TCCR0B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer1 freq){
  TCCR1B = (TCCR1B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer2 freq){
  TCCR2B = (TCCR2B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer3 freq){
  TCCR3B = (TCCR3B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer4 freq){
  TCCR4B = (TCCR4B & 0b11111000) | ((uint8_t) freq);
}
void PWMfreq::set(MegaTimer5 freq){
  TCCR5B = (TCCR5B & 0b11111000) | ((uint8_t) freq);
}
#endif

#endif


// ==================================================
// SerialComm
// ==================================================

#if defined(UNO) || defined(MEGA)

HardwareSerial* SerialComm::port(uint8_t channel) {
  switch(channel){
    case 0:
      return &Serial;
    #if defined(MEGA)
    case 1:
      return &Serial1;
    case 2:
      return &Serial2;
    case 3:
      return &Serial3;
    #endif
    default:
      return &Serial;
  }
}

void SerialComm::start(HardwareSerial* hwserial, uint32_t baudrate, uint8_t config) {
  hwserial->begin(baudrate, config);
  hwserial->flush();
}

void SerialComm::start(uint8_t channel, uint32_t baudrate, uint8_t config) {
  start(port(channel), baudrate, config);
}

void SerialComm::close(HardwareSerial* hwserial) {
  hwserial->flush();
  hwserial->end();
}

void SerialComm::close(uint8_t channel) {
  close(port(channel));
}

#endif

// ==================================================
// PinControl
// ==================================================

PinControl::PinControl(uint8_t pin){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(0.0, 0.0);
}

PinControl::PinControl(uint8_t pin, float v1, float v2){
  this->pin = pin;
  pinMode(pin, OUTPUT);
  setLimits(v1, v2);
}

uint8_t PinControl::getPin(){
  return this->pin;
}

void PinControl::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

void PinControl::set(bool state){
  #if defined(PIN_CONTROL_STORE_VALUES)
  this->set_ = state;
  #endif
  digitalWrite(pin, state ? HIGH : LOW);
}

void PinControl::pwm(uint8_t pwm){
  #if defined(PIN_CONTROL_STORE_VALUES)
  this->pwm_ = pwm;
  #endif
  analogWrite(pin, pwm);
}

void PinControl::control(float value){
  pwm(remap(value, v1, v2, 0l, 255l, true));
}

#if defined(PIN_CONTROL_EXTRA_FEATURES)
void PinControl::feedback(float error){
  if(pid != NULL) control(pid->evolve(error));
}

void PinControl::feedback(){
  if(pid != NULL) control(pid->output());
}

void PinControl::setPID(PID *pid){
  this->pid = pid;
}

PID* PinControl::getPID(){
  return this->pid;
}
#endif

#if defined(PIN_CONTROL_STORE_VALUES)
bool PinControl::last_set(){
  return this->set_;
}

uint8_t PinControl::last_pwm(){
  return this->pwm_;
}

float PinControl::last_control(){
  return remap(this->pwm_, 0l, 255l, v1, v2);
}
#endif


// ==================================================
// PinMeasure
// ==================================================

PinMeasure::PinMeasure(uint8_t pin, bool pullup){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(0.0, 0.0);
}

PinMeasure::PinMeasure(uint8_t pin, float v1, float v2, bool pullup){
  this->pin = pin;
  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  setLimits(v1, v2);
}

uint8_t PinMeasure::getPin(){
  return this->pin;
}

void PinMeasure::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

bool PinMeasure::state(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  this->state_ = digitalRead(pin) == HIGH;
  return this->state_;
  #else
  return digitalRead(pin) == HIGH;
  #endif
}

uint16_t PinMeasure::value(){
  #if defined(PIN_MEASURE_STORE_VALUES)
  this->value_ = analogRead(pin);
  return this->value_;
  #else
  return analogRead(pin);
  #endif
}

float PinMeasure::measure(){
  return remap((long) value(), 0l, 1023l, v1, v2);
}

#if defined(PIN_MEASURE_EXTRA_FEATURES)
float PinMeasure::filter(bool readonly){
  return ((fil != NULL) ? (readonly ? fil->output() : fil->evolve(measure())) : measure());
}

float PinMeasure::filter(){
  return ((fil != NULL) ? fil->evolve(measure()) : measure());
}

void PinMeasure::setFilter(Filter *filter){
  this->fil = filter;
}

Filter* PinMeasure::getFilter(){
  return this->fil;
}
#endif

#if defined(PIN_MEASURE_STORE_VALUES)
bool PinMeasure::last_state(){
  return this->state_;
}

uint16_t PinMeasure::last_value(){
  return this->value_;
}

float PinMeasure::last_measure(){
  return remap((long) this->value_, 0l, 1023l, v1, v2);
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