// Joystick board code

#if SELECT_SKETCH == 4


// ============================================================
// Includes
// ============================================================

#include "util.h"
#include "control.h"
#include "components.h"
#include "communication.h"
#include "variant.h"

// ============================================================
// Pins
// ============================================================

#define PIN_ANALOG_1  PC0   //A0
#define PIN_ANALOG_2  PC1   //A1
#define PIN_ANALOG_3  PC2   //A2
#define PIN_ANALOG_4  PC3   //A3
#define PIN_ANALOG_5  PA2   //A4
#define PIN_ANALOG_6  PA3   //A5


// ============================================================
// Parameters
// ============================================================

// Joystick
#define DEADZONE_INNER  64
#define DEADZONE_OUTER  64

// Serial Communication
#define COUNT         6       // Motor count
#define CHANNEL       1       // Serial channel
#define BAUDRATE      115200  // Serial baudrate
#define SAMPLING_US   100000  // Communication timeout
#define ALLOW_TICKS   1       // Allowed missing control ticks
#define COMM_WAIT_MS  1000    // Communication error wait


// ============================================================
// Components & Variables
// ============================================================

PinMeasure analogs[COUNT] = {
  PinMeasure(PIN_ANALOG_1),
  PinMeasure(PIN_ANALOG_2),
  PinMeasure(PIN_ANALOG_3),
  PinMeasure(PIN_ANALOG_4),
  PinMeasure(PIN_ANALOG_5),
  PinMeasure(PIN_ANALOG_6)
};

uint16_t reads[COUNT]; // Array per i valori letti dai pin analogici
int16_t pwms[COUNT]; // Array per i segnali di controllo PWM

uint16_t analog_neg_lb =  511 - DEADZONE_INNER;
uint16_t analog_neg_ub =    0 + DEADZONE_OUTER;
uint16_t analog_pos_lb =  512 + DEADZONE_INNER;
uint16_t analog_pos_ub = 1023 - DEADZONE_OUTER;

bool comm_ok = false; // Stato della comunicazione
Timer timeout;

Communication::MsgROBOT msg_robot;
Communication::MsgACKS msg_acks;

Communication::MsgPWM msg_pwm;
Communication::MsgACKC msg_ackc;


// ============================================================
// Setup
// ============================================================

void setup() {
  SerialComm::start((uint8_t) CHANNEL, BAUDRATE);

  msg_robot.setCount(COUNT);
  msg_robot.setTimeSampling(SAMPLING_US);
  msg_robot.setAllowedTicks(ALLOW_TICKS);

  msg_pwm.setCount(COUNT);

  timeout.setup((uint32_t) 2 * SAMPLING_US);

  delay(COMM_WAIT_MS);
}


// ============================================================
// Loop
// ============================================================

void loop() {
  uint32_t time_us = micros();

  if(!comm_ok) {
    timeout.reset(time_us);
    comm_ok = Communication::transmit(&msg_robot, &msg_acks, &timeout);
    if(!comm_ok) {
      delay(COMM_WAIT_MS);
      return;
    }
  }

  //Legge i valori dai pin analogici e li memorizza nell'array reads[]
  for(uint8_t i = 0; i < COUNT; i++) {
    reads[i] = analogs[i].value();
  }

  //Controlloo valori ricevuti da joystick
  for(uint8_t i = 0; i < COUNT; i++) {
    pwms[i] = 0;
    if(reads[i] <= 511) {
      if(reads[i] > analog_neg_lb) {
        pwms[i] = -0;
      } else if (reads[i] < analog_neg_ub) {
        pwms[i] = -255;
      } else {
        pwms[i] = (int16_t) remap1((float) analog_neg_lb, (float) analog_neg_ub, -0.0f, -255.0f, true);
      }
    } else {
      if(reads[i] < analog_pos_lb) {
        pwms[i] = +0;
      } else if(reads[i] > analog_pos_ub) {
        pwms[i] = +255;
      } else {
        pwms[i] = (int16_t) remap1((float) analog_pos_lb, (float) analog_pos_ub, +0.0f, +255.0f, true);
      }
    }

    //Memorizza il valore PWM calcolato nel messaggio di controllo.
    msg_pwm.setPwm(i, pwms[i]);
  }

  //Invio dei dati
  timeout.reset(time_us);
  comm_ok = Communication::transmit(&msg_pwm, &msg_ackc, &timeout);
}


// ============================================================

#endif