#include <STM32FreeRTOS.h>
#include "Job.h"
#include "components.h"
#include "control.h"
#include <Arduino.h>


#define N_Motors 1  //numero di motori \
                    // Variabile per controllare se il robot deve tornare indietro
bool returning = false;
//VAriabile per controllare se il robot deve rimanere fermo, dopo returning
bool idle = false;
//contatori
int motorsAtTargetCount = 0;  // Variabile per tenere traccia di quanti motori hanno raggiunto il target
int motorsAtHomeCount = 0;    // Contatore dei motori che sono tornati alla posizione di partenza

// ==================================================
// Macchina a Stati
// ==================================================

volatile RobotState currentState = IDLE;  // Variabile condivisa tra il task di controllo e quello di attuazione
volatile int pwm_command = 0;             // Contiene il comando PWM calcolato dal PID e usato da moveMotor


void robotStateManager(void *arg) {

  //motor_task_args *args = (motor_task_args *)arg;
  motor_task_args *args = (motor_task_args *)arg;  // Cast dell'argomento passato a un array di motor_task_args

  Serial.print("Task Macchina a Stati__________________________________________ ");

  // Estrai i membri della struttura in variabili locali
  // int pwm = args->pwm;
  // Motor &motor = args->motor;

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();





  for (;;) {
    // motorsAtTargetCount = 0;                  // Reset del contatore ogni ciclo
    // motorsAtHomeCount = 0;                    // Reset del contatore di ritorno

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;
      int &pwm = motor_args.pwm;
      float target_position = motor_args.reference;  // Ottieni il target dal riferimento della struttura


      switch (currentState) {
        case IDLE:
          // Stato di riposo, i motori sono fermi
          pwm = 0;
          // (opzionale) Reset encoder, se desiderato
          // motor->resetEncoder();
          Serial.println("Stato: IDLE __________________________________________");
          if (returning) {
            // Se il robot sta tornando indietro, passa allo stato di movimento inverso
            currentState = RETURNING;
            returning = false;  // Resetta il flag returning
          } else {
            // Altrimenti, passa alla lettura degli encoder
            currentState = READING_ENCODERS;
          }

          if (idle) {
            currentState = IDLE;
          } else {
            currentState = READING_ENCODERS;
            idle = false;  // << AGGIUNGI QUESTO
          }


          break;

        case READING_ENCODERS:
          // Leggi solo gli encoder
          motor.updateEncoder();
          Serial.println("Encoder __________________________________________");
          Serial.print("Encoder motore ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.println(motor.getEncoder());
          currentState = PID_STATE;

          // Controlla se il motore ha raggiunto il target
          if (motor.getEncoder() >= target_position) {
            motorsAtTargetCount++;  // Incrementa il contatore
            Serial.print("Motore ");
            Serial.print(i + 1);
            Serial.println(" ha raggiunto il target!");

            // Se tutti i motori hanno raggiunto il target, passiamo allo stato di ritorno
            if (motorsAtTargetCount == N_Motors) {
              Serial.println("Tutti i motori hanno raggiunto il target. Inizio il ritorno!");
              currentState = RETURNING;
              motorsAtTargetCount = 0;  // Reset del contatore
            }
          }


          break;

        case PID_STATE:
          Serial.println("PID manager __________________________________________");
          currentState = MOVING;
          break;

        case MOVING:
          // Continua a guidare il motore con il PWM corrente
          // Legge anche encoder mentre si muove (opzionale ma utile)
          motor.updateEncoder();
          Serial.println("Stato: MOVING__________________________________________");
          Serial.print("MOVING motore ");
          Serial.print(i + 1);
          Serial.print(" → PWM: ");
          Serial.println(pwm);
          currentState = READING_ENCODERS;

          break;

        case RETURNING:
          // Esegui il ritorno (movimento inverso)
          motor.updateEncoder();
          Serial.println("Stato: RETURNING__________________________________________");
          Serial.print("RETURNING motore ");
          Serial.print(i + 1);
          Serial.print(" → PWM: ");
          Serial.println(-pwm);  // Movimento inverso

          // Se un motore ha raggiunto la posizione di partenza (encoder <= 0)
          if (motor.getEncoder() <= 0) {
            motorsAtHomeCount++;  // Incrementa il contatore
            Serial.print("Motore ");
            Serial.print(i + 1);
            Serial.println(" ha raggiunto la posizione di partenza!");

            // Quando tutti i motori sono tornati alla posizione di partenza, vai a IDLE
            if (motorsAtHomeCount == N_Motors) {
              Serial.println("Tutti i motori sono tornati alla posizione di partenza.");
              motorsAtHomeCount = 0;  // Reset del contatore
              returning = false;      // Ferma il ritorno
              currentState = IDLE;
              idle = true;  // Robot rimane fermo
            }
          }
          break;
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

// ==================================================
// PiD
// ==================================================
void pidTask(void *arg) {

  //motor_task_args *args = (motor_task_args *)arg;
  motor_task_args *args = (motor_task_args *)arg;

  // Motor &motor = args->motor;
  // PID *pid = args->pid;
  // float ref = args->reference;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task PID: ");

  for (;;) {
    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;
      PID *pid = motor_args.pid;
      float ref = motor_args.reference;

      // Calcola errore: riferimento - attuale
      float error = ref - motor.getEncoder();

      // Calcola output del PID (cioè il comando di controllo)
      int pwm_cmd = (int)pid->evolve(error);
      // Serial.println("\nerrore pid");
      // Serial.println(pwm_cmd);

      // Aggiorna la variabile globale pwm_command con il valore calcolato
      pwm_command = pwm_cmd;

      // Aggiorna il valore PWM nella struttura (verrà usato dal task di attuazione)
      //args->pwm = pwm_cmd;
      motor_args.pwm = pwm_cmd;

      Serial.print("Errore motore ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(error);
      Serial.print(" → PWM: ");
      Serial.println(pwm_cmd);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}


// ==================================================
// Motori
// ==================================================
// Applica il comando PWM al motore per farlo muovere
void moveMotor(void *arg) {
  //Motor *motor = (Motor *)arg;
  motor_task_args *args = (motor_task_args *)arg;


  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;  // Ciclo ogni 10 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task Motori: ");

  for (;;) {
    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;
      int pwm_cmd = motor_args.pwm;

      Serial.print("Movimento Motore ");
      Serial.print(i + 1);
      Serial.print(" → PWM : ");
      Serial.println(pwm_cmd);

      pwm_command = pwm_cmd;


      // 1. Se il motore ha raggiunto il fine corsa, fermalo
      if (motor.isInEndStop()) {
        motor.driveMotor(0);  // Sicurezza: fermiamo il motore
      }
      // 2. Altrimenti, applica il comando calcolato dal PID
      else {
        motor.driveMotor(pwm_command);
      }

      Serial.print("PWM finale applicato: ");
      Serial.println(pwm_cmd);

      Serial.print("Finecorsa? ");
      Serial.println(motor.isInEndStop());

      // 3. Aspetta fino al prossimo ciclo
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

// ==================================================
// Encoder
// ==================================================
void read_motor_encoders(void *arg) {

  //Motor *motor = (Motor *)arg;
  motor_task_args *args = (motor_task_args *)arg;


  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;  // Leggi ogni 5 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();        // Tempo iniziale

  Serial.print("Task Encoder: ");

  // for (;;) {
  //   motor->updateEncoder();                       // Aggiorna il conteggio dell'encoder
  //   vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Aspetta il prossimo ciclo
  // }

  for (;;) {
    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;

      motor.updateEncoder();  // Aggiorna il conteggio dell'encoder
      Serial.print("Encoder motore ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(motor.getEncoder());
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
