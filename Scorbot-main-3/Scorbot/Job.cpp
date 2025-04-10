#include <STM32FreeRTOS.h>
#include "Job.h"
#include "components.h"
#include "control.h"
#include <Arduino.h>
//MACCHINA A STATI

volatile RobotState currentState = IDLE;

void robotStateManager(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  // Estrai i membri della struttura in variabili locali
  int pwm = args->pwm;
  Motor &motor = args->motor;

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    switch (currentState) {
      case IDLE:
        // Ferma il motore
        pwm = 0;
        // (opzionale) Reset encoder, se desiderato
        // motor->resetEncoder();
        Serial.println("Stato: IDLE");
        break;

      case READING_ENCODERS:
        // Leggi solo gli encoder
        motor.updateEncoder();
        Serial.print("Encoder: ");
        Serial.println(motor.getEncoder());  // Se esiste un metodo così
        break;

      case MOVING:
        // Continua a guidare il motore con il PWM corrente
        pwm = motor.pin_PWM.pwm_;
        // Legge anche encoder mentre si muove (opzionale ma utile)
        motor.updateEncoder();
        Serial.print("Stato: MOVING, Encoder: ");
        Serial.println(motor.getEncoder());
        break;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


//MOTORI
void moveMotor(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;
  // Estrai i membri della struttura in variabili locali
  int pwm = args->pwm;
  Motor &motor = args->motor;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;  // Esegui ogni 10 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();  // Salva il tempo iniziale

  for (;;) {
    // Verifica se il sensore di fine corsa è attivo
    if (motor.isInEndStop()) {
      pwm = 0;  // Ferma il motore
      Serial.println("Fine corsa raggiunto, motore fermato");
    } else {
      // Altrimenti, muovi il motore come previsto
      if (pwm > 0) {
        motor.driveMotor(args->pwm);  // Muove in senso orario
      } else if (args->pwm < 0) {
        motor.driveMotor(args->pwm);  // Muove in senso antiorario
      } else {
        motor.driveMotor(0);  // Ferma il motore
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



//ENCODER
void read_motor_encoders(void *arg) {
  Motor *motor = (Motor *)arg;

  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;  // Leggi ogni 5 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();        // Tempo iniziale

  for (;;) {
    motor->updateEncoder();  // Aggiorna il conteggio dell'encoder
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Aspetta il prossimo ciclo
  }
}
