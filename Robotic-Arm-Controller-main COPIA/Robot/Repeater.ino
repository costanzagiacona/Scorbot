// Comunication Echo
/*
Questo codice implementa una comunicazione seriale di tipo echo tra due canali seriali.
 In pratica, qualsiasi dato ricevuto su un canale viene immediatamente inoltrato all'altro.
*/

#if SELECT_SKETCH == 3


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

#define PIN_TOGGLE PE3    //13       // Toggle pin
// viene usato per indicare quando i dati vengono ricevuti e trasmessi


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
  toggle.set(true); // Attiva il pin di toggle all'inizio

  serial_in = SerialComm::port(CHANNEL_IN); // Ottiene la porta seriale in ingresso
  serial_out = SerialComm::port(CHANNEL_OUT); // Ottiene la porta seriale in uscita

  SerialComm::start(serial_in, BAUDRATE_IN); // Avvia la comunicazione seriale in ingresso
  SerialComm::start(serial_out, BAUDRATE_OUT); // Avvia la comunicazione seriale in uscita

  toggle.set(false); // Spegne il pin di toggle
}


// ============================================================
// Loop
// ============================================================

void loop() {
  if(serial_in->available() > 0 || serial_out->available() > 0) {
    toggle.set(true);  // Accende il toggle quando ci sono dati da leggere
    while(serial_in->available() > 0) {
      serial_out->write((uint8_t) serial_in->read()); // Trasmette i dati ricevuti da serial_in a serial_out
    }
    while(serial_out->available() > 0) {
      serial_out->read(); // Svuota il buffer serial_out (probabilmente per evitare overflow)
    }
  }
  toggle.set(false); // Spegne il toggle quando non ci sono dati
}


// ============================================================

#endif