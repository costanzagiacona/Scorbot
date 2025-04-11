#ifndef JOB_H
#define JOB_H

#include"components.h"




//MOTORI
struct motor_task_args {
  Motor motor;    // Oggetto del motore
  int pwm;        // Valore del PWM
  PID *pid;
  float reference;
};


//MACCHINA A STATI
enum RobotState {
    IDLE,
    READING_ENCODERS,
    MOVING
};
 
extern volatile RobotState currentState;
void robotStateManager(void *arg); // Task FreeRTOS

 
extern volatile int pwm_command;
void moveMotor(void *arg);
void pidTask(void *arg);

//ENCODER

void read_motor_encoders(void *arg);




#endif