// Robot control board code

#if SELECT_SKETCH == 1


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

// DC Motors PINs
#define MOTORS_EN     PG10    //38    // Motors enabler -  pin per abilitare i motori, controllare la direzione di rotazione, la PWM (velocità), gli encoder e gli endstop switch (fine corsa).

#define MOTOR_1_INA   PG13    //22   // Motor 1 spin direction
#define MOTOR_1_INB   PG14   //23    // Motor 1 spin direction
#define MOTOR_1_PWM   PE7    //5     // Motor 1 spin pwm  
#define MOTOR_1_CHA   PF6   //A8    // Motor 1 encoder A channel
#define MOTOR_1_CHB   PF3  //A14   // Motor 1 encoder B channel                         
#define MOTOR_1_END   PG9   //37    // Motor 1 endstop switch

#define MOTOR_2_INA   PA13  //24    // Motor 2 spin direction                           
#define MOTOR_2_INB   PF13  //25    // Motor 2 spin direction
#define MOTOR_2_PWM   PE5   //2     // Motor 2 spin pwm 
#define MOTOR_2_CHA   PF7   //A9    // Motor 2 encoder A channel
#define MOTOR_2_CHB   PF5  //A15   // Motor 2 encoder B channel                          
#define MOTOR_2_END   PG8   //36    // Motor 2 endstop switch

#define MOTOR_3_INA   PF14  //26    // Motor 3 spin direction
#define MOTOR_3_INB   PF15  //27    // Motor 3 spin direction
#define MOTOR_3_PWM   PE6   //3     // Motor 3 spin pwm
#define MOTOR_3_CHA   PF8   //A10   // Motor 3 encoder A channel
#define MOTOR_3_CHB   PA15  //10    // Motor 3 encoder B channel        PWM
#define MOTOR_3_END   PG7   //35    // Motor 3 endstop switch

#define MOTOR_4_INA   PG0   //28    // Motor 4 spin direction
#define MOTOR_4_INB   PG1   //29    // Motor 4 spin direction
#define MOTOR_4_PWM   PA1   //6     // Motor 4 spin pwm                                  
#define MOTOR_4_CHA   PF9   //A11   // Motor 4 encoder A channel
#define MOTOR_4_CHB   PB5   //11    // Motor 4 encoder B channel
#define MOTOR_4_END   PG6   //34    // Motor 4 endstop switch

#define MOTOR_5_INA   PB3   //49    // Motor 5 spin direction            PWM
#define MOTOR_5_INB   PB4   //48    // Motor 5 spin direction
#define MOTOR_5_PWM   PE1   //7     // Motor 5 spin pwm
#define MOTOR_5_CHA   PF10  //A12   // Motor 5 encoder A channel
#define MOTOR_5_CHB   PE2   //12    // Motor 5 encoder B channel                           
#define MOTOR_5_END   PG5   //33    // Motor 5 endstop switch

#define MOTOR_6_INA   PE4   //47    // Motor 6 spin direction                              
#define MOTOR_6_INB   PB6   //46    // Motor 6 spin direction             PWM
#define MOTOR_6_PWM   PE0   //8     // Motor 6 spin pwm
#define MOTOR_6_CHA   PF4   //A13   // Motor 6 encoder A channel                           
#define MOTOR_6_CHB   PE3   //13    // Motor 6 encoder B channel                            
#define MOTOR_6_END   PG4   //32    // Motor 6 endstop switch

// Other pins
#define TOGGLE_COMM   PF1   //52    // Toggle pin used to communication timing           
#define TOGGLE_CTRL   PA15  //53    // Toggle pin used to control timesampling    


// ============================================================
// Parameters 
// ============================================================

// Control
#define TS_US       100000  // Control time sampling (microseconds)

#define PID_1_DIV   1.0     // Motor 1 PID encoder error divider
#define PID_1_KP    0.0     // Motor 1 PID proportional coefficient
#define PID_1_KI    0.0     // Motor 1 PID integral coefficient
#define PID_1_KD    0.0     // Motor 1 PID derivative coefficient
#define PID_1_SAT   0.0     // Motor 1 PID integral saturation
#define PID_1_POLE  0.0     // Motor 1 PID dirty derivative pole

#define PID_2_DIV   1.0     // Motor 2 PID encoder error divider
#define PID_2_KP    0.0     // Motor 2 PID proportional coefficient
#define PID_2_KI    0.0     // Motor 2 PID integral coefficient
#define PID_2_KD    0.0     // Motor 2 PID derivative coefficient
#define PID_2_SAT   0.0     // Motor 2 PID integral saturation
#define PID_2_POLE  0.0     // Motor 2 PID dirty derivative pole

#define PID_3_DIV   1.0     // Motor 3 PID encoder error divider
#define PID_3_KP    0.0     // Motor 3 PID proportional coefficient
#define PID_3_KI    0.0     // Motor 3 PID integral coefficient
#define PID_3_KD    0.0     // Motor 3 PID derivative coefficient
#define PID_3_SAT   0.0     // Motor 3 PID integral saturation
#define PID_3_POLE  0.0     // Motor 3 PID dirty derivative pole

#define PID_4_DIV   1.0     // Motor 4 PID encoder error divider
#define PID_4_KP    0.0     // Motor 4 PID proportional coefficient
#define PID_4_KI    0.0     // Motor 4 PID integral coefficient
#define PID_4_KD    0.0     // Motor 4 PID derivative coefficient
#define PID_4_SAT   0.0     // Motor 4 PID integral saturation
#define PID_4_POLE  0.0     // Motor 4 PID dirty derivative pole

#define PID_5_DIV   1.0     // Motor 5 PID encoder error divider
#define PID_5_KP    0.0     // Motor 5 PID proportional coefficient
#define PID_5_KI    0.0     // Motor 5 PID integral coefficient
#define PID_5_KD    0.0     // Motor 5 PID derivative coefficient
#define PID_5_SAT   0.0     // Motor 5 PID integral saturation
#define PID_5_POLE  0.0     // Motor 5 PID dirty derivative pole

#define PID_6_DIV   1.0     // Motor 6 PID encoder error divider
#define PID_6_KP    0.0     // Motor 6 PID proportional coefficient
#define PID_6_KI    0.0     // Motor 6 PID integral coefficient
#define PID_6_KD    0.0     // Motor 6 PID derivative coefficient
#define PID_6_SAT   0.0     // Motor 6 PID integral saturation
#define PID_6_POLE  0.0     // Motor 6 PID dirty derivative pole

// Serial Communication
#define CHANNEL     0       // Serial channel
#define BAUDRATE    115200  // Serial baudrate

// Debug
#if defined(DEBUG_COMMUNICATION)
#define DEBUG_BAUDRATE 115200
#endif


// ============================================================
// Components & Variables
// ============================================================

PinControl mot1_ina = PinControl(MOTOR_1_INA);
PinControl mot1_inb = PinControl(MOTOR_1_INB);
PinControl mot1_pwm = PinControl(MOTOR_1_PWM);
PinMeasure mot1_cha = PinMeasure(MOTOR_1_CHA, true);
PinMeasure mot1_chb = PinMeasure(MOTOR_1_CHB, true);
PinMeasure mot1_end = PinMeasure(MOTOR_1_END);

PinControl mot2_ina = PinControl(MOTOR_2_INA);
PinControl mot2_inb = PinControl(MOTOR_2_INB);
PinControl mot2_pwm = PinControl(MOTOR_2_PWM);
PinMeasure mot2_cha = PinMeasure(MOTOR_2_CHA, true);
PinMeasure mot2_chb = PinMeasure(MOTOR_2_CHB, true);
PinMeasure mot2_end = PinMeasure(MOTOR_2_END);

PinControl mot3_ina = PinControl(MOTOR_3_INA);
PinControl mot3_inb = PinControl(MOTOR_3_INB);
PinControl mot3_pwm = PinControl(MOTOR_3_PWM);
PinMeasure mot3_cha = PinMeasure(MOTOR_3_CHA, true);
PinMeasure mot3_chb = PinMeasure(MOTOR_3_CHB, true);
PinMeasure mot3_end = PinMeasure(MOTOR_3_END);

PinControl mot4_ina = PinControl(MOTOR_4_INA);
PinControl mot4_inb = PinControl(MOTOR_4_INB);
PinControl mot4_pwm = PinControl(MOTOR_4_PWM);
PinMeasure mot4_cha = PinMeasure(MOTOR_4_CHA, true);
PinMeasure mot4_chb = PinMeasure(MOTOR_4_CHB, true);
PinMeasure mot4_end = PinMeasure(MOTOR_4_END);

PinControl mot5_ina = PinControl(MOTOR_5_INA);
PinControl mot5_inb = PinControl(MOTOR_5_INB);
PinControl mot5_pwm = PinControl(MOTOR_5_PWM);
PinMeasure mot5_cha = PinMeasure(MOTOR_5_CHA, true);
PinMeasure mot5_chb = PinMeasure(MOTOR_5_CHB, true);
PinMeasure mot5_end = PinMeasure(MOTOR_5_END);

PinControl mot6_ina = PinControl(MOTOR_6_INA);
PinControl mot6_inb = PinControl(MOTOR_6_INB);
PinControl mot6_pwm = PinControl(MOTOR_6_PWM);
PinMeasure mot6_cha = PinMeasure(MOTOR_6_CHA, true);
PinMeasure mot6_chb = PinMeasure(MOTOR_6_CHB, true);
PinMeasure mot6_end = PinMeasure(MOTOR_6_END);

Motor motor1 = Motor(mot1_ina, mot1_inb, mot1_pwm, mot1_cha, mot1_chb, mot1_end);
Motor motor2 = Motor(mot2_ina, mot2_inb, mot2_pwm, mot2_cha, mot2_chb, mot2_end);
Motor motor3 = Motor(mot3_ina, mot3_inb, mot3_pwm, mot3_cha, mot3_chb, mot3_end);
Motor motor4 = Motor(mot4_ina, mot4_inb, mot4_pwm, mot4_cha, mot4_chb, mot4_end);
Motor motor5 = Motor(mot5_ina, mot5_inb, mot5_pwm, mot5_cha, mot5_chb, mot5_end);
Motor motor6 = Motor(mot6_ina, mot6_inb, mot6_pwm, mot6_cha, mot6_chb, mot6_end);


// oggetti che rappresentano i motori, permettendo di attivarli, controllarne la velocità e leggere gli encoder.
PinControl enable = PinControl(MOTORS_EN);
PinControl toggle_comm = PinControl(TOGGLE_COMM);
PinControl toggle_ctrl = PinControl(TOGGLE_CTRL);

Robot robot = Robot(enable, 6, TS_US); //gestisce i 6 motori e il controllo PID
RobotComm robotcomm = RobotComm(robot, CHANNEL); //gestisce la comunicazione con il computer o un microcontrollore


// ============================================================
// Setup
// ============================================================

void setup()
{
  //Imposta la frequenza della PWM per il controllo della velocità
  PWMfreq::set(PWMfreq::MegaTimer3::FREQ_3921_16);
  PWMfreq::set(PWMfreq::MegaTimer4::FREQ_3921_16);
  //Avvia la comunicazione seriale a 115200 baud.
  SerialComm::start((uint8_t) CHANNEL, BAUDRATE);

  #if defined(DEBUG_COMMUNICATION)
  SerialComm::start((uint8_t) DEBUG_CHANNEL, DEBUG_BAUDRATE);
  #endif

  //collega ogni motore al sistema di controllo
  robot.setMotor(0, motor1);
  robot.setMotor(1, motor2);
  robot.setMotor(2, motor3);
  robot.setMotor(3, motor4);
  robot.setMotor(4, motor5);
  robot.setMotor(5, motor6);

  //inizializzano il controllo PID
  robot.initPID(0, PID_1_SAT, PID_1_POLE);
  robot.initPID(1, PID_2_SAT, PID_2_POLE);
  robot.initPID(2, PID_3_SAT, PID_3_POLE);
  robot.initPID(3, PID_4_SAT, PID_4_POLE);
  robot.initPID(4, PID_5_SAT, PID_5_POLE);
  robot.initPID(5, PID_6_SAT, PID_6_POLE);

  robot.setupPID(0, PID_1_DIV, PID_1_KP, PID_1_KI, PID_1_KD);
  robot.setupPID(1, PID_2_DIV, PID_2_KP, PID_2_KI, PID_2_KD);
  robot.setupPID(2, PID_3_DIV, PID_3_KP, PID_3_KI, PID_3_KD);
  robot.setupPID(3, PID_4_DIV, PID_4_KP, PID_4_KI, PID_4_KD);
  robot.setupPID(4, PID_5_DIV, PID_5_KP, PID_5_KI, PID_5_KD);
  robot.setupPID(5, PID_6_DIV, PID_6_KP, PID_6_KI, PID_6_KD);

  //abilita i motori 
  robot.enableMotors();
  //configura la comunicazione
  robotcomm.setPinComm(&toggle_comm);
  robotcomm.setPinCtrl(&toggle_ctrl);

  Communication::channel(CHANNEL);
  Communication::flush();
}


// ============================================================
// Loop
// ============================================================

void loop()
{
  uint32_t time_us = micros(); //prende il tempo corrente in microsecondi

  robotcomm.cycle(time_us); //gestisce la comunicazione e aggiorna i controlli
}


// ============================================================


#endif