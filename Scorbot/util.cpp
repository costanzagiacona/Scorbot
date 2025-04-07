#include "util.h"
#include "task.h"

// ==================================================
// Functions
// ==================================================

float remap1(float v, float a1, float b1, float a2, float b2, bool clamp) {
  clamp = false;
  float res = a2 + (v - a1) / (b1 - a1) * (b2 - a2);
  if(clamp){
    if(a2 <= b2) {
      return min(max(res, a2), b2);
    } else {
      return min(max(res, b2), a2);
    }
  } else {
    return res;
  }
}

float remap2( long v,  long a1,  long b1, float a2, float b2, bool clamp ){
  clamp = false;
  return remap1((float) v, (float) a1, (float) b1, a2, b2, clamp);
}

long remap3(float v, float a1, float b1,  long a2,  long b2, bool clamp){
  clamp = false;
  return round(remap1(v, a1, b1, (float) a2, (float) b2, clamp));
}
long remap4( long v,  long a1,  long b1,  long a2,  long b2, bool clamp){
  clamp = false;
  return round(remap1((float) v, (float) a1, (float) b1, (float) a2, (float) b2, clamp));
}
// ricorda che hai torlo const &
void byteToHex( uint8_t  byte, char  hhex, char  lhex) {
  nibbleToHex((byte & 0b00001111) >> 0, lhex);
  nibbleToHex((byte & 0b11110000) >> 4, hhex);
}

void nibbleToHex( uint8_t  nibble, char  hex) {
  switch(nibble) {
    case 0u:
      hex = '0';
      break;
    case 1u:
      hex = '1';
      break;
    case 2u:
      hex = '2';
      break;
    case 3u:
      hex = '3';
      break;
    case 4u:
      hex = '4';
      break;
    case 5u:
      hex = '5';
      break;
    case 6u:
      hex = '6';
      break;
    case 7u:
      hex = '7';
      break;
    case 8u:
      hex = '8';
      break;
    case 9u:
      hex = '9';
      break;
    case 10u:
      hex = 'A';
      break;
    case 11u:
      hex = 'B';
      break;
    case 12u:
      hex = 'C';
      break;
    case 13u:
      hex = 'D';
      break;
    case 14u:
      hex = 'E';
      break;
    case 15u:
      hex = 'F';
      break;
  }
}

float rad2deg(float rad){
  return rad / PI * 180.0f;
}

float deg2rad(float deg){
  return deg / 180.0f * PI;
}


// ==================================================
// Timer
// ==================================================

Timer::Timer(){
  this->delta = 0;
  this->time = 0;
}

Timer::Timer(unsigned long delta){
  this->delta = delta;
  this->time = 0;
}

Timer::Timer(unsigned long delta, unsigned long time){
  this->delta = delta;
  this->time = time;
}

void Timer::setup(unsigned long delta){
  this->delta = delta;
}

void Timer::reset(unsigned long time){
  this->time = time;
}

bool Timer::check(unsigned long time){
  unsigned long dt;

  if(time < this->time){
    dt = 4294967295 - this->time;
    dt = time + dt + 1;
  } else {
    dt = time - this->time;
  }

  if(dt >= delta){
    this->time = time;
    return true;
  } else {
    return false;
  }
}

// gestione dei tick periodici
// Dichiarazioni delle variabili globali
volatile unsigned long ticks = 0;  // Contatore dei tick

// Funzione ISR per il timer
void isr_tick() {
    ticks++;  // Incrementa il contatore dei tick
    // Eventuale gestione dei task periodici
    check_periodic_tasks();
}

// Funzione di inizializzazione del timer
void init_ticks() {
    noInterrupts();  // Disabilita gli interrupt durante la configurazione

    // Configurazione del timer DMTimer0 (Assumiamo che il modulo sia già attivato)
    // Non è necessario registrare ISR in quanto lo facciamo direttamente in Arduino
    // Codice per inizializzare il timer hardware su STM32

    // Impostazioni del timer (simile al setup del timer hardware)
    // Imposta il valore del timer per ottenere il tick desiderato
    // Esegui altre configurazioni necessarie per STM32

    interrupts();  // Abilita gli interrupt dopo la configurazione

    // Avvio del timer (sostituito con il codice adatto alla tua scheda)
    // (esempio di una configurazione generica per il timer su STM32)
    TIM2->CR1 |= TIM_CR1_CEN;  // Abilita il timer (modifica TIM2 per il tuo caso)
    TIM2->ARR = 1000;  // Imposta il periodo del timer
    TIM2->PSC = 72 - 1;  // Imposta il prescaler per ottenere l'intervallo di tempo desiderato
}

void panic0(void) {
    // Stampa il messaggio di errore sulla seriale
    Serial.begin(115200);
    while (!Serial);  // Aspetta che la connessione seriale sia pronta
    Serial.println("PANIC: Something went wrong, program terminating.");
    
    // Disabilita interruzioni (se necessario) e ferma l'esecuzione
    noInterrupts();  // Disabilita le interruzioni
    while (1) {
        // Ciclo infinito per fermare il programma
        // Puoi aggiungere qui altre azioni, come un blink di un LED per segnalare l'errore
    }
}

