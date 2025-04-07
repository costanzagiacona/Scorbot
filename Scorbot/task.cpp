//TASK
#define MAX_NUM_TASKS 30

/*
1. Macchina a stati
   Gestisce gli stati della macchina : Idle, Movimento, I/O sensori
2. Movimento
   Per ogni motore abbiamo un job che si occupa del movimento
3. Sensori
   Lettura finecorsa
*/

#include "components.h"
#include "Pid.h"
#include "task.h"
#include "util.h"

int num_tasks;
struct task taskset[MAX_NUM_TASKS];
volatile uint32_t globalreleases = 0;


// funzione per inizializzare i task
void init_taskset(void) {
  num_tasks = 0;
  for (int i = 0; i < MAX_NUM_TASKS; ++i) {
    taskset[i].valid = 0;
    taskset[i].job = NULL;  // Inizializza il job a NULL
    taskset[i].releaseTime = 0;
    taskset[i].released = 0;
    taskset[i].period = 0;
    taskset[i].priority = 0;
    taskset[i].name = "";  // Inizializza il nome come stringa vuota
  }
}

int create_task(void (*job)(void *), void *arg, int period, int delay, int priority, const char *name) {
  int i;
  task *t;

  // Trova una slot libera per il task
  for (i = 0; i < MAX_NUM_TASKS; ++i) {
    if (!taskset[i].valid) {
      break;
    }
  }

  // Se non ci sono slot liberi, restituisci -1
  if (i == MAX_NUM_TASKS) {
    return -1;
  }

  // Assegna il puntatore al task
  t = &taskset[i];

  // Inizializza i parametri del task
  t->job = job;                       // Assegna la funzione job
  t->arg = arg;                       // Assegna l'argomento per il job
  t->name = name;                     // Assegna il nome
  t->period = period;                 // Periodo del task
  t->priority = priority;             // Priorità del task
  t->releaseTime = millis() + delay;  // Calcola il tempo di rilascio
  t->released = 0;                    // Numero di job rilasciati ma non ancora eseguiti
  t->valid = 1;                       // Imposta il task come valido

  noInterrupts();  // Disabilita le interruzioni mentre aggiorni il taskset
  ++num_tasks;     // Incrementa il numero di task
  interrupts();    // Riabilita le interruzioni

  char buffer[100];  // Assicurati che il buffer sia abbastanza grande per contenere la stringa
  sprintf(buffer, "Task %s created, TID=%d", name, i);
  Serial.println(buffer);

  return i;  // Restituisce l'indice del task creato
}

void check_periodic_tasks(void) {
    uint32_t now = millis(); // Ottieni il valore attuale del timer globale (o ticks)
    struct task *f;            // Puntatore alla struttura del task
    int i;                     // Variabile per l'indice nel ciclo

    for (i = 0, f = taskset; i < num_tasks; ++i, ++f) { // Ciclo su tutti i task
        if (!f->valid)         // Se il task non è valido, salta al prossimo
            continue;

        // Controlla se è passato abbastanza tempo per eseguire il job del task
        if (time_after_eq(now, f->releaseTime)) {
            ++f->released;     // Aumenta il numero di job rilasciati per questo task
            f->releaseTime += f->period; // Aggiorna il tempo di rilascio del task, aggiungendo il suo periodo
            ++globalreleases;   // Incrementa il contatore globale di rilascio dei task
        }
    }
}

int time_after_eq(unsigned long time1, unsigned long time2) {
    return (time1 >= time2);
}


struct task *select_best_task(void) {
    uint32_t maxprio = UINT32_MAX;  // Definisci la massima priorità possibile
    struct task *best = NULL;
    struct task *f;
    int i;

    for (i = 0, f = taskset; i < num_tasks; ++f) {
        if (f - taskset >= MAX_NUM_TASKS) {
            panic0(); /* Should never happen */
        }
        if (!f->valid) {
            continue;  // Task non valido, lo saltiamo
        }
        ++i;
        if (f->released == 0) {
            continue;  // Task non rilasciato, lo saltiamo
        }
        if (f->priority < maxprio) {
            maxprio = f->priority;  // Troviamo il task con la priorità più alta
            best = f;  // Salviamo il task con la priorità migliore
        }
    }

    return best;  // Ritorna il task migliore da eseguire
}


void run_periodic_tasks(void) {
    struct task *best;
    unsigned long state = globalreleases; // Memorizza lo stato attuale dei task rilasciati

    // Seleziona il task con la priorità più alta da eseguire
    best = select_best_task();

    // Se un task valido è stato trovato e il numero di rilasci non è cambiato
    if (best != NULL && state == globalreleases) {
        // Esegui il job associato al task
        best->job(best->arg);
        
        // Decrementa il contatore dei task rilasciati (un task è stato eseguito)
        best->released--;
    }
      delay(4000); // questo delay serve per far eseguire il movimento del motore, altrimenti non visibile 
}


// JOB per il movimento dei motori
// Funzione per muovere il motore, adattata per lavorare con void *
void moveMotor(void *arg) {
  motor_task_args *args = (motor_task_args *)arg;

  if (args->pwm > 0) {
    args->motor.driveMotor(args->pwm);  // Muove il motore in senso orario
  } else if (args->pwm < 0) {
    args->motor.driveMotor(args->pwm);  // Muove il motore in senso antiorario
  } else {
    args->motor.driveMotor(0);  // Ferma il motore
  }
  
}

// Funzione per leggere gli encoder
void read_motor_encoders(void *arg) {
    // Cast dell'argomento in un puntatore al tipo corretto (Motor)
    Motor *motor = (Motor *)arg;
    
    // utilizzare i metodi di Motor come updateEncoder
    motor->updateEncoder();
}

// MACCHINA A STATI
RobotState currentState = IDLE;
unsigned long stateEntryTime = 0;

// Struttura per passare dati ai job
extern motor_task_args motorArgs;  // Suppongo tu lo abbia definito altrove

void state_machine(void *arg) {
  switch (currentState) {
    case IDLE:
      Serial.println("Stato: IDLE");
      // Logica di uscita dall'IDLE (es. comandi, trigger, timeout, ecc.)
      if (ENABLE == LOW) { /* condizione per iniziare il movimento */
        // Puoi settare argomenti per il task del motore qui
        motorArgs.pwm = 100;  // esempio
        create_task(moveMotor, &motorArgs, 100, 0, 1, "Motore");
        currentState = MOVING;
        stateEntryTime = millis();
      }
      break;

    case MOVING:
      Serial.println("Stato: MOVING");
      if  (ENABLE == HIGH)  {
        currentState = IO;
        stateEntryTime = millis();
      }
      break;

    case IO:
      Serial.println("Stato: IO");
      // Esegui task di I/O, come leggere encoder o finecorsa
      create_task(read_motor_encoders, &motorArgs.motor, 100, 0, 2, "Encoder");

      // Dopo aver letto i sensori, torna in IDLE o passa ad altro
      currentState = IDLE;
      stateEntryTime = millis();
      break;
  }
}

