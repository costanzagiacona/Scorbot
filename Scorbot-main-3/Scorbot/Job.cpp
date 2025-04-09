#include <STM32FreeRTOS.h>
#include "Job.h"
#include "components.h"
#include "control.h"
#include <Arduino.h>
//MACCHINA A STATI

volatile RobotState currentState = IDLE;
 
void robotStateManager(void *arg) {

    Motor *motor = (Motor *)arg;  // Cast dell'argomento passato

    const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;

    TickType_t xLastWakeTime = xTaskGetTickCount();
 
    for (;;) {

        switch (currentState) {

            case IDLE:

                // Ferma il motore

                motor->driveMotor(0);

                // (opzionale) Reset encoder, se desiderato

                // motor->resetEncoder();

                Serial.println("Stato: IDLE");

                break;
 
            case READING_ENCODERS:

                // Leggi solo gli encoder

                motor->updateEncoder();

                Serial.print("Encoder: ");

                Serial.println(motor->getEncoder()); // Se esiste un metodo così

                break;
 
            case MOVING:

                // Continua a guidare il motore con il PWM corrente

                motor->driveMotor( motor->pin_PWM.pwm_ ); // Assumendo getPWM() restituisce l’ultimo valore impostato

                // Legge anche encoder mentre si muove (opzionale ma utile)

                motor->updateEncoder();

                Serial.print("Stato: MOVING, Encoder: ");

                Serial.println(motor->getEncoder());

                break;

        }
 
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }

}
 




//MOTORI

void moveMotor(void *arg) {

  motor_task_args *args = (motor_task_args *)arg;
 
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // Esegui ogni 10 ms

  TickType_t xLastWakeTime = xTaskGetTickCount();        // Salva il tempo iniziale
 
  for (;;) {

    if (args->pwm > 0) {

      args->motor.driveMotor(args->pwm);  // Muove in senso orario

    } else if (args->pwm < 0) {

      args->motor.driveMotor(args->pwm);  // Muove in senso antiorario

    } else {

      args->motor.driveMotor(0);          // Ferma il motore

    }
 
    // Aspetta fino al prossimo "tic" previsto

    vTaskDelayUntil(&xLastWakeTime, xFrequency);

  }

}


//ENCODER

void read_motor_encoders(void *arg) {
    Motor *motor = (Motor *)arg;
 
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS; // Leggi ogni 5 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();       // Tempo iniziale
 
    for (;;) {
        motor->updateEncoder(); // Aggiorna il conteggio dell'encoder
 
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Aspetta il prossimo ciclo
    }
}
 