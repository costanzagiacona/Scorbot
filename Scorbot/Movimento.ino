#define MOTOR_1_INA   PG13   // Direzione motore
#define MOTOR_1_INB   PG14   // Direzione motore
#define MOTOR_1_PWM   PE7    // PWM per il motore

#include "components.h"
#include"Pid.h"
/*
void setup() {
    pinMode(MOTOR_1_INA, OUTPUT);
    pinMode(MOTOR_1_INB, OUTPUT);
    pinMode(MOTOR_1_PWM, OUTPUT);

    analogWriteFrequency(1000);  // Imposta la frequenza PWM a 1 kHz
}

void loop() {
  //GIRO ORARIO  
  /*
  // Imposta la direzione del motore (invertita)
    digitalWrite(MOTOR_1_INA, LOW);
    digitalWrite(MOTOR_1_INB, HIGH);

    // Aumenta gradualmente la velocità del motore
    for (int pwm = 0; pwm <= 255; pwm += 25) {
        analogWrite(MOTOR_1_PWM, pwm);  
        delay(500);
    }

    // Ferma il motore per un attimo
    analogWrite(MOTOR_1_PWM, 0);
    delay(1000);
   


   //GIRA ANTIORARIO

 // Imposta la direzione del motore
    digitalWrite(MOTOR_1_INA, HIGH);
    digitalWrite(MOTOR_1_INB, LOW);

    // Aumenta gradualmente la velocità del motore
    for (int pwm = 0; pwm <= 255; pwm += 25) {
        analogWrite(MOTOR_1_PWM, pwm);  
        delay(500);
    }

    // Ferma il motore per un attimo
    analogWrite(MOTOR_1_PWM, 0);
    delay(1000);



}

*/



