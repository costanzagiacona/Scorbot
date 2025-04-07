#ifndef TASK_H
#define TASK_H

#include <stdint.h>
#include "util.h"
#include "components.h"

// Definizione delle costanti
#define MAX_NUM_TASKS 30
extern int num_tasks;
extern struct task taskset[MAX_NUM_TASKS];
extern volatile uint32_t globalreleases;

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


// Inizializza la task set
void init_taskset(void);

// Crea un nuovo task
int create_task(void (*job)(void *), void *arg, int period, int delay, int priority, const char *name);

// Controlla i task periodici
void check_periodic_tasks(void);

// Funzione di utilità per il confronto di tempi
int time_after_eq(unsigned long time1, unsigned long time2);

// Seleziona il miglior task da eseguire
static inline struct task *select_best_task(void);

// MOVIMENTO
// Funzione che gestisce il movimento del motore
void moveMotor(int pwm, Motor &motor);

// ENCODER
// Funzione per leggere gli encoder
void read_encoders(void *arg)

#endif // TASK_MANAGER_H
