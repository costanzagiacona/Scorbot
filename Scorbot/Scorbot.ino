#include "components.h"
#include "Pid.h"
#include "util.h"
#include "task.h"

//DEFINE PIN
// DC Motors PINs
#define MOTORS_EN PG10  //38    // Motors enabler -  pin per abilitare i motori, controllare la direzione di rotazione, la PWM (velocità), gli encoder e gli endstop switch (fine corsa).

#define MOTOR_1_INA PG13  //22   // Motor 1 spin direction -> giro orario INA LOW e INB HIGH
#define MOTOR_1_INB PG14  //23    // Motor 1 spin direction
#define MOTOR_1_PWM PE7   //5     // Motor 1 spin pwm
#define MOTOR_1_CHA PF6   //A8    // Motor 1 encoder A channel
#define MOTOR_1_CHB PF3   //A14   // Motor 1 encoder B channel
#define MOTOR_1_END PG9   //37    // Motor 1 endstop switch

#define MOTOR_2_INA PA13  //24    // Motor 2 spin direction
#define MOTOR_2_INB PF13  //25    // Motor 2 spin direction
#define MOTOR_2_PWM PE5   //2     // Motor 2 spin pwm
#define MOTOR_2_CHA PF7   //A9    // Motor 2 encoder A channel
#define MOTOR_2_CHB PF5   //A15   // Motor 2 encoder B channel
#define MOTOR_2_END PG8   //36    // Motor 2 endstop switch

#define MOTOR_3_INA PF14  //26    // Motor 3 spin direction
#define MOTOR_3_INB PF15  //27    // Motor 3 spin direction
#define MOTOR_3_PWM PE6   //3     // Motor 3 spin pwm
#define MOTOR_3_CHA PF8   //A10   // Motor 3 encoder A channel
#define MOTOR_3_CHB PA15  //10    // Motor 3 encoder B channel        PWM
#define MOTOR_3_END PG7   //35    // Motor 3 endstop switch

#define MOTOR_4_INA PG0  //28    // Motor 4 spin direction
#define MOTOR_4_INB PG1  //29    // Motor 4 spin direction
#define MOTOR_4_PWM PA1  //6     // Motor 4 spin pwm
#define MOTOR_4_CHA PF9  //A11   // Motor 4 encoder A channel
#define MOTOR_4_CHB PB5  //11    // Motor 4 encoder B channel
#define MOTOR_4_END PG6  //34    // Motor 4 endstop switch

#define MOTOR_5_INA PB3   //49    // Motor 5 spin direction            PWM
#define MOTOR_5_INB PB4   //48    // Motor 5 spin direction
#define MOTOR_5_PWM PE1   //7     // Motor 5 spin pwm
#define MOTOR_5_CHA PF10  //A12   // Motor 5 encoder A channel
#define MOTOR_5_CHB PE2   //12    // Motor 5 encoder B channel
#define MOTOR_5_END PG5   //33    // Motor 5 endstop switch

#define MOTOR_6_INA PE4  //47    // Motor 6 spin direction
#define MOTOR_6_INB PB6  //46    // Motor 6 spin direction             PWM
#define MOTOR_6_PWM PE0  //8     // Motor 6 spin pwm
#define MOTOR_6_CHA PF4  //A13   // Motor 6 encoder A channel
#define MOTOR_6_CHB PE3  //13    // Motor 6 encoder B channel
#define MOTOR_6_END PG4  //32    // Motor 6 endstop switch

// ============================================================
// Parameters
// ============================================================

// Control
#define TS_US 100000  // Control time sampling (microseconds)

#define PID_1_DIV 1.0   // Motor 1 PID encoder error divider
#define PID_1_KP 0.0    // Motor 1 PID proportional coefficient
#define PID_1_KI 0.0    // Motor 1 PID integral coefficient
#define PID_1_KD 0.0    // Motor 1 PID derivative coefficient
#define PID_1_SAT 0.0   // Motor 1 PID integral saturation
#define PID_1_POLE 0.0  // Motor 1 PID dirty derivative pole

#define PID_2_DIV 1.0   // Motor 2 PID encoder error divider
#define PID_2_KP 0.0    // Motor 2 PID proportional coefficient
#define PID_2_KI 0.0    // Motor 2 PID integral coefficient
#define PID_2_KD 0.0    // Motor 2 PID derivative coefficient
#define PID_2_SAT 0.0   // Motor 2 PID integral saturation
#define PID_2_POLE 0.0  // Motor 2 PID dirty derivative pole

#define PID_3_DIV 1.0   // Motor 3 PID encoder error divider
#define PID_3_KP 0.0    // Motor 3 PID proportional coefficient
#define PID_3_KI 0.0    // Motor 3 PID integral coefficient
#define PID_3_KD 0.0    // Motor 3 PID derivative coefficient
#define PID_3_SAT 0.0   // Motor 3 PID integral saturation
#define PID_3_POLE 0.0  // Motor 3 PID dirty derivative pole

#define PID_4_DIV 1.0   // Motor 4 PID encoder error divider
#define PID_4_KP 0.0    // Motor 4 PID proportional coefficient
#define PID_4_KI 0.0    // Motor 4 PID integral coefficient
#define PID_4_KD 0.0    // Motor 4 PID derivative coefficient
#define PID_4_SAT 0.0   // Motor 4 PID integral saturation
#define PID_4_POLE 0.0  // Motor 4 PID dirty derivative pole

#define PID_5_DIV 1.0   // Motor 5 PID encoder error divider
#define PID_5_KP 0.0    // Motor 5 PID proportional coefficient
#define PID_5_KI 0.0    // Motor 5 PID integral coefficient
#define PID_5_KD 0.0    // Motor 5 PID derivative coefficient
#define PID_5_SAT 0.0   // Motor 5 PID integral saturation
#define PID_5_POLE 0.0  // Motor 5 PID dirty derivative pole

#define PID_6_DIV 1.0   // Motor 6 PID encoder error divider
#define PID_6_KP 0.0    // Motor 6 PID proportional coefficient
#define PID_6_KI 0.0    // Motor 6 PID integral coefficient
#define PID_6_KD 0.0    // Motor 6 PID derivative coefficient
#define PID_6_SAT 0.0   // Motor 6 PID integral saturation
#define PID_6_POLE 0.0  // Motor 6 PID dirty derivative pole

//Init pin e motori
// oggetto che rappresentano i motori, permettendo di attivarli, controllarne la velocità e leggere gli encoder.
PinControl enable = PinControl(MOTORS_EN);

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

void led_toggle(void *arg) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial.println("Task LED eseguito!");
}


// Crea la struttura per passare i parametri (PWM e Motor)
motor_task_args args = { 200, motor1 };  // Imposta PWM a 100 per il motore
motor_task_args args1 = { -200, motor1 };  // Imposta PWM a 100 per il motore

unsigned long previousMillis = 0;  // Variabile per memorizzare l'ultimo tempo in cui un task è stato eseguito
const long interval = 100;         // Intervallo in millisecondi per eseguire il task (1 secondo)
 

void setup() {
  //DA MODIFICARE

  // Inizializzazione delle risorse
  Serial.begin(115200);  // Inizializza la comunicazione seriale (opzionale, utile per debug)
  Serial.println("setup");
  // Inizializza il taskset
  init_taskset();


  //create_task(led_toggle, NULL, 10, 0, 100, "LED Task");

  // Crea il task per muovere il motore
  create_task(moveMotor, &args, 10, 0, 12, "MotorMoveTaskCounterClockwise");  //antiorario
  create_task(moveMotor, &args1, 10, 0, 10, "MotorMoveTaskClockwise"); //orario
}

void loop() {

  //Serial.println("loop");
  // Esegui i task periodici
  check_periodic_tasks();
  run_periodic_tasks();
  //moveMotor(&args);
  //delay(5000);
 
  // Ottieni il tempo attuale
  unsigned long currentMillis = millis();
 
  // Controlla se è passato l'intervallo per eseguire un task periodico
  if (currentMillis - previousMillis >= interval) {
    // Salva l'ultimo tempo
    previousMillis = currentMillis;
 
    // Esegui il task del motore
    run_periodic_tasks();
  }
}
