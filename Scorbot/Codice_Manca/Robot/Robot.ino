/*
Questo codice è un sistema di protezione che garantisce che solo schede compatibili 
possano eseguire determinate modalità (SELECT_SKETCH). Se l’utente imposta un 
valore non supportato, il programma non verrà eseguito.

Arduino Uno	1, 2, 3	❌ Bloccato
Arduino Uno	4	✅ Funziona
Arduino Mega	1, 2, 3, 4	✅ Funziona
Arduino Leonardo	1, 2, 3	❌ Bloccato
Arduino Leonardo	4	✅ Funziona
*/

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif

//STM32
#if defined(ARDUINO_ARCH_STM32) 
#define STM32
#endif

#define SELECT_SKETCH 1
// 1: Controller
// 2: Debugger
// 3: Repeater
// 4: Joystick

#if !defined(SELECT_SKETCH) || \
    (SELECT_SKETCH < 1 || SELECT_SKETCH > 4) || \
    (!defined(MEGA) && !defined(STM32) && SELECT_SKETCH < 4)

#ifdef SELECT_SKETCH
#undef SELECT_SKETCH
#endif

void setup() {
  Serial.begin(115200);
    while (!Serial); 
    Serial.println("Verifica Seriali disponibili:");

#ifdef SERIAL_PORT_HARDWARE1
    Serial.println("Serial1 è disponibile");
#endif

#ifdef SERIAL_PORT_HARDWARE2
    Serial.println("Serial2 è disponibile");
#endif

#ifdef SERIAL_PORT_HARDWARE3
    Serial.println("Serial3 è disponibile");
#endif
}
void loop() {}

#endif