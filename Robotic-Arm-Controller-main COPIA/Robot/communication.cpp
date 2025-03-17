#include "communication.h"


// ==================================================
// Communication
// ==================================================

HardwareSerial* Communication::hwserial = &Serial;

void Communication::channel(uint8_t index){
  hwserial = SerialComm::port(index);
}

void Communication::flush(bool input){
  if(input) {
    while(hwserial->available()) {
      hwserial->read();
    }
  }
  hwserial->flush();
}

bool Communication::peek(Header *hdr, Code target, Timer *timeout_us) {
  int res = -1;
  bool found = false;
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  uint32_t count = 0;
  #endif
  while(true){
    res = hwserial->peek();
    found = (res == -1) ? false : (((uint8_t) res) >> 3) == (uint8_t) target;
    if(found) break;
    if(res != -1) hwserial->read();
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
    count++;
    #endif
    if(timeout_us == NULL || timeout_us->check(micros())) break;
  } 
  if(!found) return false;
  if(hdr != NULL) {
    hdr->setCode(target);
    hdr->setNum((uint8_t) res & 0b00000111);
  }
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  DEBUG_SERIAL.print("peek: ");
  DEBUG_SERIAL.print(count);
  DEBUG_SERIAL.print(" | ");
  char hhex, lhex;
  byteToHex((uint8_t) res, hhex, lhex);
  DEBUG_SERIAL.print(hhex);
  DEBUG_SERIAL.print(lhex);
  DEBUG_SERIAL.print(" (");
  DEBUG_SERIAL.print((uint8_t) target);
  DEBUG_SERIAL.println(")");
  #endif
  return true;
}

bool Communication::peek(Communication::Header *hdr, Timer *timeout_us){
  int res = -1;
  Code code;
  bool found = false;
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  uint32_t count = 0;
  #endif
  while(true){
    res = hwserial->peek();
    found = (res == -1) ? false : Communication::convert(((uint8_t) res) >> 3, code);
    if(found) break;
    if(res != -1) hwserial->read();
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
    count++;
    #endif
    if(timeout_us == NULL || timeout_us->check(micros())) break;
  } 
  if(!found) return false;
  if(hdr != NULL) {
    hdr->setCode(code);
    hdr->setNum((uint8_t) res & 0b00000111);
  }
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  DEBUG_SERIAL.print("peek: ");
  DEBUG_SERIAL.print(count);
  DEBUG_SERIAL.print(" | ");
  char hhex, lhex;
  byteToHex((uint8_t) res, hhex, lhex);
  DEBUG_SERIAL.print(hhex);
  DEBUG_SERIAL.print(lhex);
  DEBUG_SERIAL.print(" (");
  DEBUG_SERIAL.print((uint8_t) code);
  DEBUG_SERIAL.println(")");
  #endif
  return true;
}

bool Communication::rcv(Communication::Message *msg, Timer *timeout_us){
  if(msg == NULL) return false;
  uint8_t buffer[msg->size()];
  while(hwserial->available() < msg->size()) if(timeout_us != NULL && timeout_us->check(micros())) return false;
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  int available = hwserial->available();
  #endif
  if(hwserial->readBytes(buffer, msg->size()) != msg->size()) return false;
  bool valid = msg->from(buffer);
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  DEBUG_SERIAL.print("rcv: ");
  DEBUG_SERIAL.print((uint8_t) msg->getCode());
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(msg->size());
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(available);
  DEBUG_SERIAL.print(" |");
  for(int i = 0; i < msg->size(); i++){
    char hhex, lhex;
    byteToHex(buffer[i], hhex, lhex);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(hhex);
    DEBUG_SERIAL.print(lhex);
  }
  DEBUG_SERIAL.println();
  #endif
  return valid;
}

bool Communication::snd(Communication::Message *msg){
  if(msg == NULL) return false;
  uint8_t buffer[msg->size()];
  msg->fill(buffer);
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  int available = hwserial->availableForWrite();
  #endif
  bool valid = hwserial->write(buffer, msg->size()) == msg->size();
  #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_LOW)
  DEBUG_SERIAL.print("snd: ");
  DEBUG_SERIAL.print((uint8_t) msg->getCode());
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(msg->size());
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(available);
  DEBUG_SERIAL.print(" |");
  for(int i = 0; i < msg->size(); i++){
    char hhex, lhex;
    byteToHex(buffer[i], hhex, lhex);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(hhex);
    DEBUG_SERIAL.print(lhex);
  }
  DEBUG_SERIAL.println();
  #endif
  return valid;
}

bool Communication::transmit(Communication::Message *sndMsg, Communication::Message *rcvMsg, Timer *timeout) {
  Communication::Header hdr;

  if (!Communication::snd(sndMsg)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Send Error");
    #endif
    return false;
  }

  if (!Communication::peek(&hdr, rcvMsg->getCode(), timeout)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Peek Error");
    #endif
    return false;
  }

  if (hdr.getCode() != rcvMsg->getCode()) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.print("Code Error (");
    DEBUG_SERIAL.print((uint8_t) hdr.getCode());
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print((uint8_t) rcvMsg->getCode());
    DEBUG_SERIAL.println(")");
    #endif
    return false;
  }

  if (!Communication::rcv(rcvMsg, timeout)) {
    #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
    DEBUG_SERIAL.println("Receive Error");
    #endif
    return false;
  }

  return true;
}

bool Communication::convert(uint8_t value, Communication::Code &code){
  switch(value){
    case (uint8_t) Code::IDLE:
      code = Code::IDLE;
      return true;
    case (uint8_t) Code::PWM:
      code = Code::PWM;
      return true;
    case (uint8_t) Code::REF:
      code = Code::REF;
      return true;
    case (uint8_t) Code::ROBOT:
      code = Code::ROBOT;
      return true;
    case (uint8_t) Code::MOTOR:
      code = Code::MOTOR;
      return true;
    case (uint8_t) Code::PID:
      code = Code::PID;
      return true;
    case (uint8_t) Code::ACKC:
      code = Code::ACKC;
      return true;
    case (uint8_t) Code::ACKS:
      code = Code::ACKS;
      return true;
    case (uint8_t) Code::ERROR:
      code = Code::ERROR;
      return true;
    default:
      return false;
  }
}

bool Communication::isCtrl(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) == 0) && ((value & 0b01000) == 0) && ((value & 0b00100) == 0);
}

bool Communication::isSetup(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) != 0) && ((value & 0b01000) == 0) && ((value & 0b00100) == 0);
}

bool Communication::isAck(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) != 0) && ((value & 0b01000) != 0) && ((value & 0b00100) == 0);
}

bool Communication::isError(Code code){
  uint8_t value = (uint8_t) code;
  return ((value & 0b10000) != 0) && ((value & 0b01000) != 0) && ((value & 0b00100) != 0);
}

// ===== Header =====

Communication::Header::Header(Communication::Code code, uint8_t num) {
  setCode(code);
  setNum(num);
}

Communication::Code Communication::Header::getCode(){
  return this->code;
}

uint8_t Communication::Header::getNum(){
  return this->num;
}

bool Communication::Header::setCode(Communication::Code code){
  this->code = code;
  return true;
}

bool Communication::Header::setNum(uint8_t num){
  if(num < 8) {
    this->num = num;
    return true;
  } else {
    return false;
  }
}

bool Communication::Header::parse(uint8_t byte){
  Code code;
  if(!convert((byte & 0b11111000) >> 3, code)) return false;
  setCode(code);
  setNum(byte & 0b00000111);
  return true;
}

uint8_t Communication::Header::byte(){
  return (((uint8_t) this->code) << 3) | num;
}

uint8_t Communication::Header::size(){
  return 1;
}

uint8_t Communication::Header::from(uint8_t *buffer){
  return parse(buffer[0]);
}

uint8_t Communication::Header::fill(uint8_t *buffer){
  buffer[0] = byte();
  return 1;
}

// ===== Message =====

Communication::Message::Message(Code code, uint8_t num){
  header.setCode(code);
  header.setNum(num);
}

Communication::Code Communication::Message::getCode(){
  return header.getCode();
}

uint8_t Communication::Message::getNum(){
  return header.getNum();
}

bool Communication::Message::setNum(uint8_t num){
  return header.setNum(num);
}

uint8_t Communication::Message::size(){
  return size_header() + size_payload();
}

uint8_t Communication::Message::from(uint8_t *buffer){
  if(from_header(buffer) != size_header()) return 0;
  if(from_payload(buffer) != size_payload()) return 0;
  return size();
}

uint8_t Communication::Message::fill(uint8_t *buffer){
  if(fill_header(buffer) != size_header()) return 0;
  if(fill_payload(buffer) != size_payload()) return 0;
  return size();
}

uint8_t Communication::Message::size_payload(){
  return 0;
}

uint8_t Communication::Message::from_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

uint8_t Communication::Message::fill_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

uint8_t Communication::Message::size_header(){
  return header.size();
}

uint8_t Communication::Message::from_header(uint8_t *buffer){
  Code code;
  if(!convert((buffer[0] & 0b11111000) >> 3, code)) return 0;
  if(getCode() != code) return 0;
  setNum(buffer[0] & 0b00000111);
  return size_header();
}

uint8_t Communication::Message::fill_header(uint8_t *buffer){
  buffer[0] = (((uint8_t) getCode()) << 3) | getNum();
  return size_header();
}

// ===== MsgIDLE =====

uint8_t Communication::MsgIDLE::getCount(){
  return getNum() + 1;
}

bool Communication::MsgIDLE::setCount(uint8_t count){
  return setNum(count - 1);
}

uint8_t Communication::MsgIDLE::size_payload(){
  return 0;
}

uint8_t Communication::MsgIDLE::from_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

uint8_t Communication::MsgIDLE::fill_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

// ===== MsgPWM =====

uint8_t Communication::MsgPWM::getCount(){
  return getNum() + 1;
}

int16_t Communication::MsgPWM::getPwm(uint8_t index){
  if(index < getCount()){
    return pwms[index];
  } else {
    return 0;
  }
}

bool Communication::MsgPWM::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgPWM::setPwm(uint8_t index, int16_t value){
  if(index < getCount()){
    pwms[index] = min(max(value, -255), 255);
    return true;
  } else {
    return false;
  }
}

uint8_t Communication::MsgPWM::size_payload(){
  return 1 + getCount();
}

uint8_t Communication::MsgPWM::from_payload(uint8_t *buffer){
  for(uint8_t i = 0; i < getCount(); i++){
    setPwm(i, ((buffer[1] & ((0b00000001) << i)) ? -1 : +1) * ((int16_t) buffer[2+i]));
  }
  return size_payload();
}

uint8_t Communication::MsgPWM::fill_payload(uint8_t *buffer){
  buffer[1] = 0b00000000;
  for(uint8_t i = 0; i < getCount(); i++){
    buffer[1] = buffer[1] | ((getPwm(i) < 0) << i);
    buffer[2+i] = abs(getPwm(i));
  }
  return size_payload();
}

// ===== MsgREF =====

uint8_t Communication::MsgREF::getCount(){
  return getNum() + 1;
}

int16_t Communication::MsgREF::getDeltaEnc(uint8_t index){
  if(index < getCount()){
    return deltas[index];
  } else {
    return 0;
  }
}

bool Communication::MsgREF::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgREF::setDeltaEnc(uint8_t index, int16_t value){
  if(index < getCount()){
    deltas[index] = min(max(value, -255), 255);
    return true;
  } else {
    return false;
  }
}

uint8_t Communication::MsgREF::size_payload(){
  return 1 + getCount();
}

uint8_t Communication::MsgREF::from_payload(uint8_t *buffer){
  for(uint8_t i = 0; i < getCount(); i++){
    setDeltaEnc(i, ((buffer[1] & (1 << i)) ? -1 : +1) * ((int16_t) buffer[2+i]));
  }
  return size_payload();
}

uint8_t Communication::MsgREF::fill_payload(uint8_t *buffer){
  buffer[1] = 0b00000000;
  for(uint8_t i = 0; i < getCount(); i++){
    buffer[1] = buffer[1] | ((getDeltaEnc(i) < 0) << i);
    buffer[2+i] = abs(getDeltaEnc(i));
  }
  return size_payload();
}

// ===== MsgROBOT =====

uint8_t Communication::MsgROBOT::getCount(){
  return getNum() + 1;
}

uint32_t Communication::MsgROBOT::getTimeSampling(){
  return timesampling_us;
}

uint8_t Communication::MsgROBOT::getAllowedTicks(){
  return allowed_ticks;
}

bool Communication::MsgROBOT::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgROBOT::setTimeSampling(uint32_t value){
  timesampling_us = value;
  return true;
}

bool Communication::MsgROBOT::setAllowedTicks(uint8_t value){
  allowed_ticks = value;
  return true;
}

uint8_t Communication::MsgROBOT::size_payload(){
  return 5;
}

uint8_t Communication::MsgROBOT::from_payload(uint8_t *buffer){
  uint32_t ts;
  memcpy((void *) &ts, (void *) (buffer+1), 4);
  setTimeSampling(ts);
  setAllowedTicks(buffer[5]);
  return size_payload();
}

uint8_t Communication::MsgROBOT::fill_payload(uint8_t *buffer){
  uint32_t ts;
  ts = getTimeSampling();
  memcpy((void *) (buffer+1), (void *) &ts, 4);
  buffer[5] = getAllowedTicks();
  return size_payload();
}

// ===== MsgMOTOR =====

uint8_t Communication::MsgMOTOR::getIndex(){
  return getNum();
}

bool Communication::MsgMOTOR::getChangeEncoder(){
  return flags & (1 << 0);
}

bool Communication::MsgMOTOR::getInvertSpinDir(){
  return flags & (1 << 1);
}

bool Communication::MsgMOTOR::getChangeSpinDir(){
  return flags & (1 << 2);
}

int8_t Communication::MsgMOTOR::getSpinDirection(){
  return getChangeSpinDir() ? (getInvertSpinDir() ? -1 : 1) : 0;
}

bool Communication::MsgMOTOR::getInvertEncDir(){
  return flags & (1 << 3);
}

bool Communication::MsgMOTOR::getChangeEncDir(){
  return flags & (1 << 4);
}

int8_t Communication::MsgMOTOR::getEncDirection(){
  return getChangeEncDir() ? (getInvertEncDir() ? -1 : 1) : 0;
}

int32_t Communication::MsgMOTOR::getEncoderValue(){
  return encoder;
}

bool Communication::MsgMOTOR::setIndex(uint8_t index){
  return setNum(index);
}

bool Communication::MsgMOTOR::setChangeEncoder(bool value){
  flags = (flags & ~(1 << 0)) | (value << 0);
  return true;
}

bool Communication::MsgMOTOR::setInvertSpinDir(bool value){
  flags = (flags & ~(1 << 1)) | (value << 1);
  return true;
}

bool Communication::MsgMOTOR::setChangeSpinDir(bool value){
  flags = (flags & ~(1 << 2)) | (value << 2);
  return true;
}

bool Communication::MsgMOTOR::setSpinDirection(int8_t dir){
  setInvertSpinDir(dir < 0);
  setChangeSpinDir(dir != 0);
  return true;
}

bool Communication::MsgMOTOR::setInvertEncDir(bool value){
  flags = (flags & ~(1 << 3)) | (value << 3);
  return true;
}

bool Communication::MsgMOTOR::setChangeEncDir(bool value){
  flags = (flags & ~(1 << 4)) | (value << 4);
  return true;
}

bool Communication::MsgMOTOR::setEncDirection(int8_t dir){
  setInvertEncDir(dir < 0);
  setChangeEncDir(dir != 0);
  return true;
}

bool Communication::MsgMOTOR::setEncoderValue(int32_t value){
  encoder = value;
  return true;
}

uint8_t Communication::MsgMOTOR::size_payload(){
  return 5;
}

uint8_t Communication::MsgMOTOR::from_payload(uint8_t *buffer){
  bool ec;
  int8_t sd;
  int8_t ed;
  int32_t ev;
  ec = ((buffer[1] & (1 << 0)) > 0);
  sd = ((buffer[1] & (1 << 2)) > 0) ? (((buffer[1] & (1 << 1)) > 0) ? -1 : +1) : 0;
  ed = ((buffer[1] & (1 << 4)) > 0) ? (((buffer[1] & (1 << 3)) > 0) ? -1 : +1) : 0;
  memcpy((void *) &ev , (void *) (buffer + 2), 4);
  setChangeEncoder(ec);
  setSpinDirection(sd);
  setEncDirection(ed);
  setEncoderValue(ev);
  return size_payload();
}

uint8_t Communication::MsgMOTOR::fill_payload(uint8_t *buffer){
  bool ec;
  int8_t sd;
  int8_t ed;
  int32_t ev;
  ec = getChangeEncoder();
  sd = getSpinDirection();
  ed = getEncDirection();
  ev = getEncoderValue(); 
  buffer[1] = ((ed != 0) << 4) | ((ed < 0) << 3) | ((sd != 0) << 2) | ((sd < 0) << 1) | (ec << 0);
  memcpy((void *) (buffer + 2), (void *) &ev , 4);
  return size_payload();
}

// ===== MsgPID =====

uint8_t Communication::MsgPID::getIndex(){
  return getNum();
}

float Communication::MsgPID::getPidDiv(){
  return div;
}

float Communication::MsgPID::getPidKp(){
  return kp;
}

float Communication::MsgPID::getPidKi(){
  return ki;
}

float Communication::MsgPID::getPidKd(){
  return kd;
}

float Communication::MsgPID::getPidSat(){
  return sat;
}

float Communication::MsgPID::getPidPole(){
  return pole;
}

bool Communication::MsgPID::setIndex(uint8_t index){
  return setNum(index);
}

bool Communication::MsgPID::setPidDiv(float value){
  div = value;
  return true;
}

bool Communication::MsgPID::setPidKp(float value){
  kp = value;
  return true;
}

bool Communication::MsgPID::setPidKi(float value){
  ki = value;
  return true;
}

bool Communication::MsgPID::setPidKd(float value){
  kd = value;
  return true;
}

bool Communication::MsgPID::setPidSat(float value){
  sat = value;
  return true;
}

bool Communication::MsgPID::setPidPole(float value){
  pole = value;
  return true;
}

uint8_t Communication::MsgPID::size_payload(){
  return 24;
}

uint8_t Communication::MsgPID::from_payload(uint8_t *buffer){
  float div;
  float kp;
  float ki;
  float kd;
  float sat;
  float pole; 
  memcpy((void *) &div , (void *) (buffer+ 1), 4);
  memcpy((void *) &kp  , (void *) (buffer+ 5), 4);
  memcpy((void *) &ki  , (void *) (buffer+ 9), 4);
  memcpy((void *) &kd  , (void *) (buffer+13), 4);
  memcpy((void *) &sat , (void *) (buffer+17), 4);
  memcpy((void *) &pole, (void *) (buffer+21), 4);
  setPidDiv(div);
  setPidKp(kp);
  setPidKi(ki);
  setPidKd(kd);
  setPidSat(sat);
  setPidPole(pole);
  return size_payload();
}

uint8_t Communication::MsgPID::fill_payload(uint8_t *buffer){
  float div;
  float kp;
  float ki;
  float kd;
  float sat;
  float pole; 
  div = getPidDiv();
  kp = getPidKp();
  ki = getPidKi();
  kd = getPidKd();
  sat = getPidSat();
  pole = getPidPole();
  memcpy((void *) (buffer+ 1), (void *) &div , 4);
  memcpy((void *) (buffer+ 5), (void *) &kp  , 4);
  memcpy((void *) (buffer+ 9), (void *) &ki  , 4);
  memcpy((void *) (buffer+13), (void *) &kd  , 4);
  memcpy((void *) (buffer+17), (void *) &sat , 4);
  memcpy((void *) (buffer+21), (void *) &pole, 4);
  return size_payload();
}

// ===== MsgACKC =====

uint8_t Communication::MsgACKC::getCount(){
  return getNum() + 1;
}

bool Communication::MsgACKC::getEndStop(uint8_t index){
  if(index < getCount()){
    return endstops & (1 << index);
  } else {
    return false;
  }
}

int16_t Communication::MsgACKC::getDeltaEnc(uint8_t index){
  if(index < getCount()){
    return deltas[index];
  } else {
    return 0;
  }
}

bool Communication::MsgACKC::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgACKC::setEndStop(uint8_t index, bool value){
  if(index < getCount()){
    endstops = endstops | (value << index);
    return true;
  } else {
    return false;
  }
}

bool Communication::MsgACKC::setDeltaEnc(uint8_t index, int16_t value){
  if(index < getCount()){
    deltas[index] = min(max(value, -255), 255);
    return true;
  } else {
    return false;
  }
}

uint8_t Communication::MsgACKC::size_payload(){
  return 2 + getCount();
}

uint8_t Communication::MsgACKC::from_payload(uint8_t *buffer){
  for(uint8_t i = 0; i < getCount(); i++){
    setEndStop(i, buffer[1] & (1 << i));
    setDeltaEnc(i, ((buffer[2] & (1 << i)) ? -1 : +1) * ((int16_t) buffer[3+i]));
  }
  return size_payload();
}

uint8_t Communication::MsgACKC::fill_payload(uint8_t *buffer){
  buffer[1] = 0b00000000;
  buffer[2] = 0b00000000;
  for(uint8_t i = 0; i < getCount(); i++){
    buffer[1] = buffer[1] | (getEndStop(i) << i);
    buffer[2] = buffer[2] | ((getDeltaEnc(i) < 0) << i);
    buffer[3+i] = (uint8_t) abs(getDeltaEnc(i));
  }
  return size_payload();
}

// ===== MsgACKS =====

uint8_t Communication::MsgACKS::getCount(){
  return getNum() + 1;
}

uint8_t Communication::MsgACKS::getIndex(){
  return getNum();
}

bool Communication::MsgACKS::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgACKS::setIndex(uint8_t index){
  return setNum(index);
}

uint8_t Communication::MsgACKS::size_payload(){
  return 0;
}

uint8_t Communication::MsgACKS::from_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

uint8_t Communication::MsgACKS::fill_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

// ===== MsgERROR =====

uint8_t Communication::MsgERROR::getCount(){
  return getNum() + 1;
}

uint8_t Communication::MsgERROR::getIndex(){
  return getNum();
}

bool Communication::MsgERROR::setCount(uint8_t count){
  return setNum(count - 1);
}

bool Communication::MsgERROR::setIndex(uint8_t index){
  return setNum(index);
}

uint8_t Communication::MsgERROR::size_payload(){
  return 0;
}

uint8_t Communication::MsgERROR::from_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}

uint8_t Communication::MsgERROR::fill_payload(uint8_t *buffer){
  UNUSED(buffer);
  return size_payload();
}


// ==================================================
// RobotComm
// ==================================================

RobotComm::RobotComm(Robot &robot, uint8_t channel) :
  robot(robot) 
{
  this->channel = channel;
  this->timer.setup(robot.getTimeSampling());
  this->timeout.setup(robot.getTimeSampling());
  this->encoders_rcv = (long *) malloc(robot.getSize() * sizeof(long));
  this->encoders_snd = (long *) malloc(robot.getSize() * sizeof(long));
  this->ticks_allowed = 0;
  this->ticks_used = 0;

  for(int i = 0; i < robot.getSize(); i++) {
    this->encoders_rcv[i] = 0;
    this->encoders_snd[i] = 0;
  }
}

RobotComm::~RobotComm(){
  free(this->encoders_rcv);
  free(this->encoders_snd);
}

void RobotComm::setPinComm(PinControl *pin) {
  this->pin_comm = pin;
}

void RobotComm::setPinCtrl(PinControl *pin) {
  this->pin_ctrl = pin;
}

void RobotComm::cycle(uint32_t time_us){
  robot.update();

  Communication::channel(channel);
  Communication::Header header;

  bool res = Communication::peek(&header);
  
  if(res){
    if(this->pin_comm != NULL) this->pin_comm->set(true);

    ticks_used = 0;
    timeout.reset(time_us);
    Communication::Code code = header.getCode();
    
    if(Communication::isCtrl(code)) 
    {
      if(robot.getStatus() == Robot::Status::Idle){
        timer.reset(time_us);
      }
      
      if(robot.getStatus() == Robot::Status::Idle || timer.check(time_us)) {
        if(this->pin_ctrl != NULL) this->pin_ctrl->set(true);

        #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
        DEBUG_SERIAL.print("Receiving Control ");
        #endif
        
        switch(code){
          case Communication::Code::IDLE:
            {
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
              DEBUG_SERIAL.println("IDLE");
              #endif
              Communication::MsgIDLE msg_idle;
              msg_idle.setCount(robot.getSize());
              res = Communication::rcv(&msg_idle, &timeout);
              res = res && msg_idle.getCount() == robot.getSize();
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
              DEBUG_SERIAL.print("  operation: ");
              DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
              #endif
              if(!res) break;
              robot.setStatus(Robot::Status::Idle);
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
              DebugComm::print(&msg_idle, "", 1);
              #endif
              break;
            }

          case Communication::Code::PWM:
            {
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
              DEBUG_SERIAL.println("PWM");
              #endif
              Communication::MsgPWM msg_pwm;
              msg_pwm.setCount(robot.getSize());
              res = Communication::rcv(&msg_pwm, &timeout);
              res = res && msg_pwm.getCount() == robot.getSize();
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
              DEBUG_SERIAL.print("  operation: ");
              DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
              #endif
              if(!res) break;
              robot.setStatus(Robot::Status::Pwm);
              for(uint8_t i = 0; i < robot.getSize(); i++) {
                robot.setPwm(i, msg_pwm.getPwm(i));
              }
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
              DebugComm::print(&msg_pwm, "", 1);
              #endif
              break;
            }

          case Communication::Code::REF:
            {
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
              DEBUG_SERIAL.println("REF");
              #endif
              Communication::MsgREF msg_ref;
              msg_ref.setCount(robot.getSize());
              res = Communication::rcv(&msg_ref, &timeout);
              res = res && msg_ref.getCount() == robot.getSize();
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
              DEBUG_SERIAL.print("  operation: ");
              DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
              #endif
              if(!res) break;
              robot.setStatus(Robot::Status::Ref);
              for(uint8_t i = 0; i < robot.getSize(); i++) {
                encoders_rcv[i] += msg_ref.getDeltaEnc(i);
                robot.setTarget(i, encoders_rcv[i]);
              }
              #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
              DebugComm::print(&msg_ref, "", 1);
              #endif
              break;
            }

          default:
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.println("Unexpected");
            #endif
            res = false;
            break;
        }
        
        if(res){
          robot.compute();
          robot.actuate();

          Communication::MsgACKC msg_ackc;
          msg_ackc.setCount(robot.getSize());
          for(uint8_t i = 0; i < robot.getSize(); i++){
            msg_ackc.setEndStop(i, robot.getEndStop(i));
            msg_ackc.setDeltaEnc(i, robot.getEncoder(i) - encoders_snd[i]);
          }

          #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
          DEBUG_SERIAL.println("Sending Control ACKC");
          #endif

          res = Communication::snd(&msg_ackc);

          #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
          DEBUG_SERIAL.print("  operation: ");
          DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
          #if defined(DEBUG_DATA)
          DebugComm::print(&msg_ackc, "", 1);
          #endif
          #endif

          if(res) {
            for(uint8_t i = 0; i < robot.getSize(); i++){
              encoders_snd[i] += msg_ackc.getDeltaEnc(i);
            }
          }

          #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
          DEBUG_SERIAL.print("Encoder rob:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            DEBUG_SERIAL.print(" ");
            DEBUG_SERIAL.print(robot.getEncoder(i));
          }
          DEBUG_SERIAL.println();

          DEBUG_SERIAL.print("Encoder rcv:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            DEBUG_SERIAL.print(" ");
            DEBUG_SERIAL.print(encoders_rcv[i]);
          }
          DEBUG_SERIAL.println();

          DEBUG_SERIAL.print("Encoder snd:");
          for(uint8_t i = 0; i < robot.getSize(); i++){
            DEBUG_SERIAL.print(" ");
            DEBUG_SERIAL.print(encoders_snd[i]);
          }
          DEBUG_SERIAL.println();
          #endif
        }

        if(this->pin_ctrl != NULL) this->pin_ctrl->set(false);
      }
    } 
    
    else if(Communication::isSetup(code)) 
    {
      robot.setStatus(Robot::Status::Idle, true);
      Communication::MsgACKS msg_acks;

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.print("Receiving Setup ");
      #endif

      switch(code){
        case Communication::Code::ROBOT:
          {
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.println("ROBOT");
            #endif
            Communication::MsgROBOT msg_robot;
            res = Communication::rcv(&msg_robot, &timeout);
            res = res && msg_robot.getCount() == robot.getSize();
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.print("  operation: ");
            DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
            #endif
            if(!res) break;
            robot.setTimeSampling(msg_robot.getTimeSampling());
            timer.setup(msg_robot.getTimeSampling());
            timeout.setup(msg_robot.getTimeSampling());
            ticks_allowed = msg_robot.getAllowedTicks();
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
            DebugComm::print(&msg_robot, "", 1);
            #endif
            msg_acks.setCount(robot.getSize());
            break;
          }

        case Communication::Code::MOTOR:
          {
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.println("MOTOR");
            #endif
            Communication::MsgMOTOR msg_motor;
            res = Communication::rcv(&msg_motor, &timeout);
            res = res && msg_motor.getIndex() < robot.getSize();
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.print("  operation: ");
            DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
            #endif
            if(!res) break;
            if(msg_motor.getChangeEncoder()) {
              robot.setEncoder(msg_motor.getIndex(), msg_motor.getEncoderValue());
              encoders_rcv[msg_motor.getIndex()] = msg_motor.getEncoderValue();
              encoders_snd[msg_motor.getIndex()] = msg_motor.getEncoderValue();
            }
            if(msg_motor.getChangeSpinDir()) {
              robot.invertMotor(msg_motor.getIndex(), msg_motor.getInvertSpinDir());
            }
            if(msg_motor.getChangeEncDir()) {
              robot.invertEncoder(msg_motor.getIndex(), msg_motor.getInvertEncDir());
            }
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
            DebugComm::print(&msg_motor, "", 1);
            #endif
            msg_acks.setIndex(msg_motor.getIndex());
            break;
          }

        case Communication::Code::PID:
          {
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.println("PID");
            #endif
            Communication::MsgPID msg_pid;
            res = Communication::rcv(&msg_pid, &timeout);
            res = res && msg_pid.getIndex() < robot.getSize();
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
            DEBUG_SERIAL.print("  operation: ");
            DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
            #endif
            if(!res) break;
            robot.initPID(msg_pid.getIndex(), msg_pid.getPidSat(),  msg_pid.getPidPole());
            robot.setupPID(msg_pid.getIndex(), msg_pid.getPidDiv(), msg_pid.getPidKp(), msg_pid.getPidKi(), msg_pid.getPidKd());
            robot.resetPID(msg_pid.getIndex());
            #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH) && defined(DEBUG_DATA)
            DebugComm::print(&msg_pid, "", 1);
            #endif
            msg_acks.setIndex(msg_pid.getIndex());
            break;
          }

        default:
          #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
          DEBUG_SERIAL.println("Unexpected");
          #endif
          res = false;
          break;
      }
      
      if(res){
        #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
        DEBUG_SERIAL.println("Sending Setup ACKS");
        #endif

        res = Communication::snd(&msg_acks);

        #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
        DEBUG_SERIAL.print("  operation: ");
        DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
        #if defined(DEBUG_DATA)
        DebugComm::print(&msg_acks, "", 1);
        #endif
        #endif
      }
    }

    else 
    {
      res = false;

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("Peeking Unexpected message");
      #endif
    }

    if(!res){
      robot.setStatus(Robot::Status::Idle, true);
      Communication::flush();
      Communication::MsgERROR msg_error;
      msg_error.setCount(robot.getSize());

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("Sending Error ERROR");
      #endif

      res = Communication::snd(&msg_error);

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.print("  operation: ");
      DEBUG_SERIAL.println(res ? "Succeded" : "Failed");
      #if defined(DEBUG_DATA)
      DebugComm::print(&msg_error, "", 1);
      #endif
      #endif
    }

    if(this->pin_comm != NULL) this->pin_comm->set(false);
  }

  else if(robot.getStatus() != Robot::Status::Idle && timer.check(time_us)) {
    if(ticks_used >= ticks_allowed) {
      robot.setStatus(Robot::Status::Idle, true);
      ticks_used = 0;

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.println("Control timeout -> Robot IDLE");
      #endif
    } else {
      robot.compute();
      robot.actuate();
      ticks_used++;

      #if defined(DEBUG_COMMUNICATION) && defined(DEBUG_HIGH)
      DEBUG_SERIAL.print("Missing control: ");
      DEBUG_SERIAL.println(ticks_used);
      #endif
    }
  }
}

void RobotComm::cycle(){
  cycle(micros());
}


// ==================================================
// DebugComm
// ==================================================

#if defined(DEBUG_COMMUNICATION)

String DebugComm::indent(uint8_t level, uint8_t size) {
  String str = "";
  for(int i = 0; i < (int) (level * size); i++) str += " ";
  return str;
}


void DebugComm::print(Communication::Header      *header, String title, uint8_t indent_level, uint8_t indent_size){
  if(header == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Header:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("code: ");
  DEBUG_SERIAL.println((uint8_t) header->getCode());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("num: ");
  DEBUG_SERIAL.println(header->getNum());
}

void DebugComm::print(Communication::Message    *message, String title, uint8_t indent_level, uint8_t indent_size){
  if(message == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("code: ");
  DEBUG_SERIAL.println((uint8_t) message->getCode());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("num: ");
  DEBUG_SERIAL.println(message->getNum());
}

void DebugComm::print(Communication::MsgIDLE   *msg_idle, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_idle == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message IDLE:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count: ");
  DEBUG_SERIAL.println(msg_idle->getCount());
}

void DebugComm::print(Communication::MsgPWM     *msg_pwm, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_pwm == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message PWM:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count: ");
  DEBUG_SERIAL.println(msg_pwm->getCount());

  for(uint8_t i = 0; i < msg_pwm->getCount(); i++){
    DEBUG_SERIAL.print(indent_1);
    DEBUG_SERIAL.print("pwm ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(msg_pwm->getPwm(i));
  }
}

void DebugComm::print(Communication::MsgREF     *msg_ref, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_ref == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message REF:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count: ");
  DEBUG_SERIAL.println(msg_ref->getCount());

  for(uint8_t i = 0; i < msg_ref->getCount(); i++){
    DEBUG_SERIAL.print(indent_1);
    DEBUG_SERIAL.print("denc ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(msg_ref->getDeltaEnc(i));
  }
}

void DebugComm::print(Communication::MsgROBOT *msg_robot, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_robot == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message ROBOT:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count: ");
  DEBUG_SERIAL.println(msg_robot->getCount());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("ts: ");
  DEBUG_SERIAL.println(msg_robot->getTimeSampling());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("ticks: ");
  DEBUG_SERIAL.println(msg_robot->getAllowedTicks());
}

void DebugComm::print(Communication::MsgMOTOR *msg_motor, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_motor == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message MOTOR:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("index: ");
  DEBUG_SERIAL.println(msg_motor->getIndex());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("encoder: ");
  DEBUG_SERIAL.print(msg_motor->getChangeEncoder() ? "1 " : "0 ");
  DEBUG_SERIAL.println(msg_motor->getEncoderValue());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("spin dir: ");
  DEBUG_SERIAL.println(msg_motor->getSpinDirection());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("enc dir: ");
  DEBUG_SERIAL.println(msg_motor->getEncDirection());
}

void DebugComm::print(Communication::MsgPID     *msg_pid, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_pid == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message PID:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("index: ");
  DEBUG_SERIAL.println(msg_pid->getIndex());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("div: ");
  DEBUG_SERIAL.println(msg_pid->getPidDiv(), 3);

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("kp: ");
  DEBUG_SERIAL.println(msg_pid->getPidKp(), 3);

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("ki: ");
  DEBUG_SERIAL.println(msg_pid->getPidKi(), 3);

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("kd: ");
  DEBUG_SERIAL.println(msg_pid->getPidKd(), 3);

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("sat: ");
  DEBUG_SERIAL.println(msg_pid->getPidSat(), 3);

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("pole: ");
  DEBUG_SERIAL.println(msg_pid->getPidPole(), 3);
}

void DebugComm::print(Communication::MsgACKC   *msg_ackc, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_ackc == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message ACKC:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count: ");
  DEBUG_SERIAL.println(msg_ackc->getCount());

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("endstops:");
  for(uint8_t i = 0; i < msg_ackc->getCount(); i++){
    DEBUG_SERIAL.print(msg_ackc->getEndStop(i) ? " 1" : " 0");
  }
  DEBUG_SERIAL.println();

  for(uint8_t i = 0; i < msg_ackc->getCount(); i++){
    DEBUG_SERIAL.print(indent_1);
    DEBUG_SERIAL.print("denc ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(msg_ackc->getDeltaEnc(i));
  }
}

void DebugComm::print(Communication::MsgACKS   *msg_acks, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_acks == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message ACKS:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count/index: ");
  DEBUG_SERIAL.print(msg_acks->getCount());
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.println(msg_acks->getIndex());
}

void DebugComm::print(Communication::MsgERROR *msg_error, String title, uint8_t indent_level, uint8_t indent_size){
  if(msg_error == NULL) return;
  String indent_0 = indent(indent_level + 0, indent_size);
  String indent_1 = indent(indent_level + 1, indent_size);

  DEBUG_SERIAL.print(indent_0);
  if(title.length() == 0) {
    DEBUG_SERIAL.println("Message ERROR:");
  } else {
    DEBUG_SERIAL.println(title);
  }

  DEBUG_SERIAL.print(indent_1);
  DEBUG_SERIAL.print("count/index: ");
  DEBUG_SERIAL.print(msg_error->getCount());
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.println(msg_error->getIndex());
}

#endif