#ifndef TASKS_H
#define TASKS_H

#include "components.h"
#include "Pid.h"
#include "task.h"
#include "util.h"

// Definizione del numero massimo di task
#define MAX_NUM_TASKS 30

// Task
struct task {
  int valid;             // evita che si scheduli un task non inizializzato
  void (*job)(void *);   // Puntatore a funzione che accetta un argomento generico (void*)
  void *arg;             // argomento da passare al job
  uint32_t releaseTime;  // Tempo di arrivo (quando il task è stato creato)
  uint32_t released;     // num di job rilasciati ma non ancora eseguiti per il task
  uint32_t period;       // periodo di rilascio
  uint32_t priority;     //priorità del task
  const char *name;      // nome del task, utile per debug
};

// Variabili globali
extern int num_tasks;
extern struct task taskset[MAX_NUM_TASKS];
extern volatile uint32_t globalreleases;

// Funzioni per la gestione dei task
void init_taskset(void);                        // Inizializza il set di task
int create_task(void (*job)(void *), void *arg, int period, int delay, int priority, const char *name); // Crea un nuovo task
void check_periodic_tasks(void);                // Controlla i task periodici
int time_after_eq(unsigned long time1, unsigned long time2); // Funzione di confronto tra due tempi
struct task *select_best_task(void);            // Seleziona il task con la priorità più alta
void run_periodic_tasks(void);

// Funzioni per i job dei task
// Struttura per passare i parametri al task
struct motor_task_args {
  int pwm;        // Valore del PWM
  Motor motor;    // Oggetto del motore
};
void moveMotor(void *arg);          // Muove il motore
void read_motor_encoders(void *arg);          // Legge gli encoder dei motori

#endif // TASKS_H
