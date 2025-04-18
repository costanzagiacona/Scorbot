#ifndef JOB_H
#define JOB_H

#include"components.h"

//MACCHINA A STATI
enum RobotState {
    IDLE,
    READING_ENCODERS,
    MOVING,
    PID_STATE,
    RETURNING
};
 
extern volatile RobotState currentState;
void robotStateManager(void *arg); // Task FreeRTOS


//MOTORI
struct motor_task_args {
  Motor* motor;    // Oggetto del motore
  int pwm;        // Valore del PWM
  PID *pid;
  float reference;


  // // Costruttore per inizializzare i membri della struttura
  // motor_task_args(Motor m, int p, PID *pid_ref, float ref)
  //     : motor(m), pwm(p), pid(pid_ref), reference(ref) {}

      // Costruttore di default per motor_task_args
  motor_task_args(Motor* motor_ref, int pwm_val, PID *pid_ref, float ref)
    : motor(motor_ref), pwm(pwm_val), pid(pid_ref), reference(ref) {}
};



extern volatile int pwm_command;
void moveMotor(void *arg);

// PID
void pidTask(void *arg);

//ENCODER
void read_motor_encoders(void *arg);

//WCET TOTALE
void loggerTask(void *pvParameters);


#endif