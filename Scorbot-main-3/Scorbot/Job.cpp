#include <STM32FreeRTOS.h>
#include "Job.h"
#include "components.h"
#include "control.h"
#include <Arduino.h>


#define N_Motors 6  //numero di motori

//FLAG
bool volatile returning = false;  // Variabile per controllare se il robot deve tornare indietro
bool volatile idle = false;       // Variabile per controllare se il robot deve rimanere fermo, dopo returning
bool volatile target = false;     //Variabile arrivo target

//CONTATORI
int volatile motorsAtTargetCount[6] = { 0, 0, 0, 0, 0, 0 };
bool volatile bool_motorsAtTargetCount = false;  // Contatore dei motori che sono tornati alla posizione di partenza
int volatile motorsAtHomeCount[6] = { 0, 0, 0, 0, 0, 0 };
bool volatile bool_motorsAtHomeCount = false;

// variabile epsilon per la differenza tra riferimento e posizione attuale
//                   1 , 2 , 3 , 4 , 5 , 6
int epsilon_a[6] = { 22, 14, 25, 26, 26, 15 };
int epsilon_r[6] = { 20, 10, 25, 28, 30, 15 };

int variabile = 3;

//calcolo WCET
uint32_t wcet_manager = 0;
uint32_t wcet_pid = 0;
uint32_t wcet_motor = 0;
uint32_t wcet_encoder = 0;

volatile RobotState currentState = IDLE;  // variabile per determinare in che stato si trova il robot
// Variabile condivisa tra il task di controllo e quello di attuazione
volatile int pwm_command = 0;  // Contiene il comando PWM calcolato dal PID e usato da moveMotor


// ==================================================
// Macchina a Stati
// ==================================================
void robotStateManager(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;  // Cast dell'argomento passato a un array di motor_task_args

  //Serial.print("Task Macchina a Stati__________________________________________ ");

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    uint32_t start = micros();  // inizia conteggio tempo esecuzione

    for (int i = 0; i < N_Motors; i++) {  // Itera su tutti i motori

      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor *motor = motor_args.motor;
      int &pwm = motor_args.pwm;
      float target_position = motor_args.reference;  // Ottieni il target dal riferimento della struttura

      switch (currentState) {
        case IDLE:
          // Stato di riposo, i motori sono fermi
          pwm = 0;
          motor_args.pwm = 0;
          motor->updateEncoder();
          //Serial.println("Stato: IDLE __________________________________________");

          if (returning) {
            // Se il robot sta tornando indietro, passa allo stato di movimento inverso
            currentState = RETURNING;
          } else if (idle) {
            currentState = IDLE;
            //Serial.print("PWM IDLE: ");
            //Serial.println(motor_args.pwm);
          } else {
            // Altrimenti, passa alla lettura degli encoder
            currentState = READING_ENCODERS;
            idle = false;
          }

          break;

        case READING_ENCODERS:
          // Leggi solo gli encoder
          motor->updateEncoder();
          // Serial.println("Encoder __________________________________________");
          // Serial.print("Encoder motore ");
          // Serial.print(i + 1);
          // Serial.print(": ");
          // Serial.println(motor.getEncoder());

          // Controlla se il motore ha raggiunto il target
          // if (i == variabile) {
          //   Serial.print("Encoder ");
          //   Serial.print(i + 1);
          //   Serial.print(": ");
          //   Serial.println(motor->getEncoder());
          //   Serial.print("Target:");
          //   Serial.println(target_position);
          //   Serial.print("Distanza da target: ");
          //   Serial.println(abs(abs(motor->getEncoder()) - abs(target_position)));
          //   Serial.println("");
          // }

          if (abs(abs(motor->getEncoder()) - abs(target_position)) <= abs(epsilon_a[i])) {
            // Se siamo vicini alla posizione target, fermati
            motorsAtTargetCount[i] = 1;  // Incrementa il contatore
            // Serial.print("motorsAtTargetCount ++ motore ->");
            // Serial.println(i+1);

            // Se tutti i motori hanno raggiunto il target, passiamo allo stato di ritorno
            bool_motorsAtTargetCount = true;
            for (int j = 0; j < N_Motors; j++) {
              if (motorsAtTargetCount[j] == 0) {
                bool_motorsAtTargetCount = false;
                break;
              }
            }

            if (bool_motorsAtTargetCount) {
              //Serial.println("Tutti i motori hanno raggiunto il target. Inizio il ritorno!");
              currentState = RETURNING;
              target = true;
            }
            break;
          }

          currentState = PID_STATE;

          break;

        case PID_STATE:
          //Serial.println("PID manager __________________________________________");
          currentState = MOVING;
          break;

        case MOVING:
          // Continua a guidare il motore con il PWM corrente
          motor->updateEncoder();   // Legge anche encoder mentre si muove
          // Serial.println("Stato: MOVING__________________________________________");
          // Serial.print("MOVING motore ");
          // Serial.print(i + 1);
          // Serial.print(" → PWM: ");
          // Serial.print(pwm);
          currentState = READING_ENCODERS;

          break;

        case RETURNING:
          // Esegui il ritorno (movimento inverso)
          motor->updateEncoder();
          // Serial.println("Stato: RETURNING__________________________________________");
          // Serial.print("RETURNING motore ");
          // Serial.print(i + 1);
          // Serial.print(" → PWM: ");
          // Serial.print(-pwm);
          // Movimento inverso

          if (target) {
            for (int i = 0; i < N_Motors; i++) {
              args[i].reference = -args[i].reference;
              if (i == 0) args[i].reference = 0;
            }
            target = false;
          }

          // Controlla se il motore ha raggiunto casa
          // if (i == variabile) {
          //   Serial.print("Encoder ");
          //   Serial.print(i + 1);
          //   Serial.print(": ");
          //   Serial.println(motor->getEncoder());
          //   Serial.print("Target:");
          //   Serial.println(target_position);
          //   Serial.print("Distanza da home: ");
          //   Serial.println(abs(abs(motor->getEncoder()) - abs(target_position)));
          //   Serial.println("");
          // }
          // Se un motore ha raggiunto la posizione di partenza (encoder <= 0)
          if (abs(abs(motor->getEncoder()) - abs(target_position)) <= epsilon_r[i]) {
            motorsAtHomeCount[i] = 1;  // Incrementa il contatore

            bool_motorsAtHomeCount = true;
            for (int j = 0; j < N_Motors; j++) {
              if (motorsAtHomeCount[j] == 0) {
                bool_motorsAtHomeCount = false;
                break;
              }
            }

            // Quando tutti i motori sono tornati alla posizione di partenza, vai a IDLE
            if (bool_motorsAtHomeCount) {
              //Serial.println("Tutti i motori sono tornati alla posizione di partenza!");
              returning = false;  // Ferma il ritorno
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
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// ==================================================
// PID
// ==================================================
void pidTask(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  //Serial.print("Task PID: ");

  for (;;) {
    uint32_t start = micros();  // inizia conteggio tempo esecuzione

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor *motor = motor_args.motor;
      PID *pid = motor_args.pid;
      float ref = motor_args.reference;

      motor->updateEncoder();

      // Calcola errore: riferimento - attuale
      float error = ref - motor->getEncoder();

      // Calcola output del PID (comando di controllo)
      int pwm_cmd = (int)pid->evolve(error);
      // Serial.println("\nerrore pid");
      // Serial.println(pwm_cmd);

      // Aggiorna la variabile globale pwm_command con il valore calcolato
      pwm_command = pwm_cmd;

      if (abs(motor->getEncoder() - ref) <= abs(epsilon_a[i])) pwm_command = 0;  //se arrivato a target

      // Se un motore ha raggiunto la posizione di partenza (encoder <= 0)
      if (abs(motor->getEncoder()) <= epsilon_r[i] && returning == true) pwm_command = 0;

      if (idle) pwm_cmd = 0;  //robot fermo

      // Aggiorna il valore PWM nella struttura (usato dal task di attuazione -> moveMotors)
      motor_args.pwm = pwm_cmd;

      uint32_t end = micros();
      uint32_t elapsed = end - start;

      // Registro WCET
      if (elapsed > wcet_pid) {
        wcet_pid = elapsed;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// ==================================================
// Motori
// ==================================================
void moveMotor(void *arg) {  // Applica il comando PWM al motore per farlo muovere

  motor_task_args *args = (motor_task_args *)arg;

  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;  // Rilascio ogni 10 ms 
  TickType_t xLastWakeTime = xTaskGetTickCount();

  //Serial.print("Task Motori: ");

  for (;;) {
    uint32_t start = micros();

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor *motor = motor_args.motor;
      int pwm_cmd = motor_args.pwm;

      motor->updateEncoder();

      pwm_command = pwm_cmd;  // variabile globale di movimento pari alla variabile locale di movimento

      //Serial.println(idle);
      if (idle) pwm_command = 0;  //robot fermo

      // Altrimenti, applica il comando calcolato dal PID
      motor->driveMotor(pwm_command);

      if (i == variabile) {
        // Serial.print("Movimento Motore ");
        // Serial.print(i + 1);
        // Serial.print(" → PWM : ");
        // Serial.println(pwm_command);
      }

      uint32_t end = micros();
      uint32_t elapsed = end - start;

      // Registro WCET
      if (elapsed > wcet_motor) {
        wcet_motor = elapsed;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==================================================
// Encoder
// ==================================================
void read_motor_encoders(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;

  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;  // Leggi ogni 5 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();        // Tempo iniziale

  for (;;) {

    uint32_t start = micros();

    for (int i = 0; i < N_Motors; i++) {      // Itera su tutti i motori
      motor_task_args &motor_args = args[i];  // Ottieni il riferimento per il motore corrente
      Motor *motor = motor_args.motor;

      motor->updateEncoder();  // Aggiorna il conteggio dell'encoder
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
    TickType_t total_wcet = wcet_pid + wcet_encoder + wcet_motor + wcet_manager;

    Serial.println("=== WCET (in microsecondi) ===");
    Serial.print("StateManager: ");
    Serial.println(wcet_manager);
    Serial.print("PID: ");
    Serial.println(wcet_pid);
    Serial.print("Encoders: ");
    Serial.println(wcet_encoder);
    Serial.print("MoveMotor: ");
    Serial.println(wcet_motor);
    Serial.print("TOT: ");
    Serial.print(total_wcet);
    Serial.println(" microsecondi");

    if (total_wcet > 300) {
      Serial.println("ATTENZIONE: Superato limite di 300 microsecondi!");
    }
    //Serial.println("Il ciclo di esecuzione non supera i 300 microsecondi.");

    Serial.println("==============================");

    // resetta i WCET ogni secondo
    wcet_pid = wcet_encoder = wcet_motor = wcet_manager = 0;

    vTaskDelayUntil(&xLastWakeTime, freq);
  }
}
