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

  // // Aggiorna gli encoder per entrambi i motori
  //motor1.updateEncoder();
  // //motor2.updateEncoder();

  // // ina false, inb true -> antiorario
  // mot1_ina.set(false); // digitalWrite 
  // mot1_ina.set(true);
  // // digitalWrite(MOTOR_1_INA, HIGH);
  // // digitalWrite(MOTOR_1_INB, LOW);




  // // // Imposta la direzione del motore
  // // digitalWrite(MOTOR_1_INA, LOW);
  // // digitalWrite(MOTOR_1_INB, HIGH);
  // mot1_pwm.pwm(150);  // analogWrite
  // delay(500);
  // mot1_pwm.pwm(0);
  // delay(1000);


  // // Aumenta gradualmente la velocità del motore
  // for (int pwm = 0; pwm <= 255; pwm += 25) {
  //     analogWrite(MOTOR_1_PWM, pwm);
  //     delay(500);
  // }

  // // Ferma il motore per un attimo
  // analogWrite(MOTOR_1_PWM, 0);
  // delay(1000);

  // Controllo motori: imposta la velocità (PWM)
  // int pwm_motor1 = 200;   // Velocità del primo motore (positivo per in avanti)
  // int pwm_motor2 = -100;  // Velocità del secondo motore (negativo per all'indietro)

  // // Imposta il PWM per entrambi i motori
  // motor1.driveMotor(-pwm_motor1);  // Imposta la velocità del primo motore
  // //motor2.driveMotor(pwm_motor2);  // Imposta la velocità del secondo motore

  // // Verifica se i motori raggiungono l'endstop
  // if (motor1.isInEndStop()) {
  //   // Fermiamo entrambi i motori se un endstop è attivato
  //   motor1.driveMotor(0);
  //   //motor2.driveMotor(0);
  //   Serial.println("Endstop raggiunto! I motori sono fermi.");
  // }

  // // Pausa per evitare che il loop sia troppo veloce (100ms)
  // delay(100);

  Serial.println("Motore avanti");
  motor1.driveMotor(150);  // Valore positivo => avanti
  delay(2000);

  Serial.println("Stop");
  motor1.driveMotor(0);  // Ferma il motore
  delay(5000);

  Serial.println("Motore indietro");
  motor1.driveMotor(-150);  // Valore negativo => indietro
  delay(2000);

  Serial.println("Stop");
  motor1.driveMotor(0);
  delay(5000);


}

*/



