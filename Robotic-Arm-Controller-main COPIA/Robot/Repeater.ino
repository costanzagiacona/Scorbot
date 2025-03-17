// Comunication Echo

#if SELECT_SKETCH == 3


// ============================================================
// Includes
// ============================================================

#include "utils.h"
#include "control.h"
#include "components.h"
#include "communication.h"


// ============================================================
// Pins
// ============================================================

#define PIN_TOGGLE 13       // Toggle pin


// ============================================================
// Parameters
// ============================================================

#define CHANNEL_IN         1
#define BAUDRATE_IN   115200
#define CHANNEL_OUT        0
#define BAUDRATE_OUT  115200


// ============================================================
// Components & Variables
// ============================================================

PinControl toggle = PinControl(PIN_TOGGLE);
HardwareSerial *serial_in;
HardwareSerial *serial_out;


// ============================================================
// Setup
// ============================================================

void setup() {
  toggle.set(true);

  serial_in = SerialComm::port(CHANNEL_IN);
  serial_out = SerialComm::port(CHANNEL_OUT);

  SerialComm::start(serial_in, BAUDRATE_IN);
  SerialComm::start(serial_out, BAUDRATE_OUT);

  toggle.set(false);
}


// ============================================================
// Loop
// ============================================================

void loop() {
  if(serial_in->available() > 0 || serial_out->available() > 0) {
    toggle.set(true);
    while(serial_in->available() > 0) {
      serial_out->write((uint8_t) serial_in->read());
    }
    while(serial_out->available() > 0) {
      serial_out->read();
    }
  }
  toggle.set(false);
}


// ============================================================

#endif