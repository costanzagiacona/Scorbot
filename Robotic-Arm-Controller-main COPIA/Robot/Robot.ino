#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif

#define SELECT_SKETCH 1
// 1: Controller
// 2: Debugger
// 3: Repeater
// 4: Joystick

#if !defined(SELECT_SKETCH) || \
    (SELECT_SKETCH < 1 || SELECT_SKETCH > 4) || \
    (!defined(MEGA) && SELECT_SKETCH < 4)

#ifdef SELECT_SKETCH
#undef SELECT_SKETCH
#endif

void setup() {}
void loop() {}

#endif