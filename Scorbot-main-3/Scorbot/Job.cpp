#include <STM32FreeRTOS.h>
#include "Job.h"
#include "components.h"
#include "control.h"
#include <Arduino.h>


#define N_Motors 1  //numero di motori \
// Variabile per controllare se il robot deve tornare indietro
bool returning = false;
//Variabile per controllare se il robot deve rimanere fermo, dopo returning
bool idle = false;
//contatori
int motorsAtTargetCount = 0;  // Variabile per tenere traccia di quanti motori hanno raggiunto il target
int motorsAtHomeCount = 0;    // Contatore dei motori che sono tornati alla posizione di partenza

//calcolo WCET
uint32_t wcet_manager = 0;
uint32_t wcet_pid = 0;
uint32_t wcet_motor = 0;
uint32_t wcet_encoder = 0;


// ==================================================
// Macchina a Stati
// ==================================================

volatile RobotState currentState = IDLE;  // variabile per determinare in che stato si trova il robot
// Variabile condivisa tra il task di controllo e quello di attuazione
volatile int pwm_command = 0;  // Contiene il comando PWM calcolato dal PID e usato da moveMotor


void robotStateManager(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;  // Cast dell'argomento passato a un array di motor_task_args

  //Serial.print("Task Macchina a Stati__________________________________________ ");

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // motorsAtTargetCount = 0;                  // Reset del contatore ogni ciclo
    // motorsAtHomeCount = 0;                    // Reset del contatore di ritorno

    uint32_t start = micros();  // inizio conteggio tempo esecuzione

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;
      int &pwm = motor_args.pwm;
      float target_position = motor_args.reference;  // Ottieni il target dal riferimento della struttura


      switch (currentState) {
        case IDLE:
          // Stato di riposo, i motori sono fermi
          pwm = 0;
          motor.updateEncoder();
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
          // Serial.print("Encoder motore ");
          // Serial.print(i + 1);
          // Serial.print(": ");
          //Serial.println(motor.getEncoder());

          // Controlla se il motore ha raggiunto il target
          if (abs(motor.getEncoder()) >= abs(target_position) || motor.isInEndStop()) {
            motorsAtTargetCount++;  // Incrementa il contatore
            // Serial.print("Motore ");
            // Serial.print(i + 1);
            Serial.println(" ha raggiunto il target!");

            // Se tutti i motori hanno raggiunto il target, passiamo allo stato di ritorno
            if (motorsAtTargetCount == N_Motors) {
              Serial.println("Tutti i motori hanno raggiunto il target. Inizio il ritorno!");
              currentState = RETURNING;
              motorsAtTargetCount = 0;  // Reset del contatore
            }
            break;
          }
          currentState = PID_STATE;

          break;

        case PID_STATE:
          Serial.println("PID manager __________________________________________");
          currentState = MOVING;
          break;

        case MOVING:
          // Continua a guidare il motore con il PWM corrente
          // Legge anche encoder mentre si muove
          motor.updateEncoder();
          Serial.println("Stato: MOVING__________________________________________");
          // Serial.print("MOVING motore ");
          // Serial.print(i + 1);
          //Serial.print(" → PWM: ");
          //Serial.print(pwm);
          currentState = READING_ENCODERS;

          break;

        case RETURNING:
          // Esegui il ritorno (movimento inverso)
          motor.updateEncoder();
          Serial.println("Stato: RETURNING__________________________________________");
          // Serial.print("RETURNING motore ");
          // Serial.print(i + 1);
          //Serial.print(" → PWM: ");
          //Serial.print(-pwm);  
          // Movimento inverso
          for (int i = 0; i < N_Motors; i++) {
            args[i].reference = -args[i].reference;
          }

          // Se un motore ha raggiunto la posizione di partenza (encoder <= 0)
          if (motor.getEncoder() <= 0) {      //---------------------------------------------------bisogna verificare questa condizione
            motorsAtHomeCount++;  // Incrementa il contatore
                                  // Serial.print("Motore ");
            //Serial.print(i + 1);
            // Serial.println(" ha raggiunto la posizione di partenza!");

            // Quando tutti i motori sono tornati alla posizione di partenza, vai a IDLE
            if (motorsAtHomeCount == N_Motors) {
              Serial.println("Tutti i motori sono tornati alla posizione di partenza. ");
              Serial.print(motorsAtHomeCount);
              motorsAtHomeCount = 0;  // Reset del contatore
              returning = false;      // Ferma il ritorno
              currentState = IDLE;
              idle = true;  // Robot rimane fermo
            }
          }
          break;
      }
      uint32_t end = micros();
      uint32_t elapsed = end - start;

      // Registro WCET
      if (elapsed > wcet_manager) {
        wcet_manager = elapsed;
      }

      //Serial.println("WCET macchina a stati attuale: ");
      //Serial.println(wcet_manager);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}


// ==================================================
// PID
// ==================================================
void pidTask(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task PID: ");

  for (;;) {
    uint32_t start = micros();  // inizio conteggio tempo esecuzione

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

      // Serial.print("Errore motore ");
      // Serial.print(i + 1);
      // Serial.print(": ");
      // Serial.print(error);
      // Serial.print(" → PWM: ");
      // Serial.println(pwm_cmd);

      uint32_t end = micros();
      uint32_t elapsed = end - start;

      // Registro WCET
      if (elapsed > wcet_pid) {
        wcet_pid = elapsed;
      }

      //Serial.println("WCET PID attuale: ");
      //Serial.println(wcet_pid);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}


// ==================================================
// Motori
// ==================================================
// Applica il comando PWM al motore per farlo muovere
void moveMotor(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;  // Ciclo ogni 10 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.print("Task Motori: ");

  for (;;) {
    uint32_t start = micros();

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;
      int pwm_cmd = motor_args.pwm;

      // Serial.print("Movimento Motore ");
      // Serial.print(i + 1);
      // Serial.print(" → PWM : ");
      // Serial.println(pwm_cmd);

      pwm_command = pwm_cmd;


      // Se il motore ha raggiunto il fine corsa, fermalo
      if (motor.isInEndStop()) {
        motor.driveMotor(0);  // Sicurezza: fermiamo il motore
      }
      // Altrimenti, applica il comando calcolato dal PID
      else {
        motor.driveMotor(pwm_command);
      }

      //Serial.print("PWM finale applicato: ");
      //Serial.println(pwm_cmd);

      //Serial.print("Finecorsa? ");
      //Serial.println(motor.isInEndStop());

      uint32_t end = micros();
      uint32_t elapsed = end - start;

      // Registro WCET
      if (elapsed > wcet_motor) {
        wcet_motor = elapsed;
      }

      //Serial.println("WCET motor attuale: ");
      //Serial.println(wcet_motor);

      // Aspetta fino al prossimo ciclo
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

// ==================================================
// Encoder
// ==================================================
void read_motor_encoders(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;  // Leggi ogni 5 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();        // Tempo iniziale

  Serial.print("Task Encoder: ");

  // for (;;) {
  //   motor->updateEncoder();                       // Aggiorna il conteggio dell'encoder
  //   vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Aspetta il prossimo ciclo
  // }

  for (;;) {

    uint32_t start = micros();

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor &motor = motor_args.motor;

      motor.updateEncoder();  // Aggiorna il conteggio dell'encoder
      // Serial.print("Encoder motore ");
      // Serial.print(i + 1);
      // Serial.print(": ");
      // Serial.println(motor.getEncoder());
    }

    uint32_t end = micros();
    uint32_t elapsed = end - start;

    // Registro WCET
    if (elapsed > wcet_encoder) {
      wcet_encoder = elapsed;
    }

    //Serial.println("WCET encoder attuale: ");
    //Serial.println(wcet_encoder);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// ==================================================
// TASK LOGGER -> calcola il WCET del ciclo
// ==================================================
void loggerTask(void *pvParameters) {
  const TickType_t freq = pdMS_TO_TICKS(1000);  // ogni 1 secondo
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Serial.println("=== WCET (in µs) ===");
    // Serial.print("PID: ");
    // Serial.println(wcet_pid);
    // Serial.print("Encoders: ");
    // Serial.println(wcet_encoder);
    // Serial.print("MoveMotor: ");
    // Serial.println(wcet_motor);
    // Serial.print("StateManager: ");
    // Serial.println(wcet_manager);
    // Serial.println("====================");

    vTaskDelayUntil(&xLastWakeTime, freq);
  }
}
