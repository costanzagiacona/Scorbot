#include <STM32FreeRTOS.h>
#include "Job.h"
#include "components.h"
#include "control.h"
#include <Arduino.h>


// ==================================================
// Macchina a Stati
// ==================================================

volatile RobotState currentState = IDLE; // Variabile condivisa tra il task di controllo e quello di attuazione
volatile int pwm_command = 0; // Contiene il comando PWM calcolato dal PID e usato da moveMotor


void robotStateManager(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  // Estrai i membri della struttura in variabili locali
  int pwm = args->pwm;
  Motor &motor = args->motor;

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task Macchina a Stati__________________________________________ ");

  for (;;) {
    switch (currentState) {
      case IDLE:
        //enable.set(false); // imposta il pin a false
        // Ferma il motore
        pwm = 0;
        // (opzionale) Reset encoder, se desiderato
        // motor->resetEncoder();
        Serial.println("Stato: IDLE __________________________________________");
        currentState = READING_ENCODERS;
        break;

      case READING_ENCODERS:
        // Leggi solo gli encoder
        motor.updateEncoder();
        Serial.println("Encoder __________________________________________");
        Serial.print("Encoder: ");
        Serial.println(motor.getEncoder());  // Se esiste un metodo così
        currentState = PID_STATE;
        break;

      case PID_STATE:
        Serial.println("PID manager __________________________________________");
        currentState = MOVING;
        break;

      case MOVING:
        // Continua a guidare il motore con il PWM corrente
        // Legge anche encoder mentre si muove (opzionale ma utile)
        motor.updateEncoder();
        Serial.print("Stato: MOVING__________________________________________\n");
        Serial.print("PWM:");
        Serial.println(pwm);
        currentState = READING_ENCODERS;
        break;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==================================================
// PiD
// ==================================================
void pidTask(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;
  Motor &motor = args->motor;
  PID *pid = args->pid;
  float ref = args->reference;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task PWM: ");

  for (;;) {
    // Calcola errore: riferimento - attuale
    float error = ref - motor.getEncoder();

    // Calcola output del PID (cioè il comando di controllo)
    int pwm_cmd = (int)pid->evolve(error);
    Serial.println("\nerrore pid");
    Serial.println(pwm_cmd);

    // Aggiorna la variabile globale pwm_command con il valore calcolato
    pwm_command = pwm_cmd;

    // Aggiorna il valore PWM nella struttura (verrà usato dal task di attuazione)
    args->pwm = pwm_cmd;

    Serial.print("Errore: ");
    Serial.print(error);
    Serial.print(" → PWM: ");
    Serial.println(pwm_cmd);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// ==================================================
// Motori
// ==================================================
// Applica il comando PWM al motore per farlo muovere
void moveMotor(void *arg) {
  Motor *motor = (Motor *)arg;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;  // Ciclo ogni 10 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task Motori: ");

  for (;;) {
    // 1. Se il motore ha raggiunto il fine corsa, fermalo
    if (motor->isInEndStop()) {
      motor->driveMotor(0);  // Sicurezza: fermiamo il motore
    }
    // 2. Altrimenti, applica il comando calcolato dal PID
    else {
      motor->driveMotor(pwm_command);
    }

    // 3. Aspetta fino al prossimo ciclo
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==================================================
// Encoder
// ==================================================
void read_motor_encoders(void *arg) {

  Motor *motor = (Motor *)arg;

  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;  // Leggi ogni 5 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();        // Tempo iniziale

  Serial.print("Task Encoder: ");

  for (;;) {
    motor->updateEncoder();                       // Aggiorna il conteggio dell'encoder
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Aspetta il prossimo ciclo
  }
}
