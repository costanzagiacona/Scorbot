#ifndef JOB_H
#define JOB_H

#include"components.h"




//MOTORI
struct motor_task_args {
  int pwm;        // Valore del PWM
  Motor motor;    // Oggetto del motore
};


//MACCHINA A STATI
enum RobotState {
    IDLE,
    READING_ENCODERS,
    MOVING
};
 
extern volatile RobotState currentState;
 
void robotStateManager(void *arg); // Task FreeRTOS

 

void moveMotor(void *arg);


//ENCODER

void read_motor_encoders(void *arg);




#endif