


// // TASK invio messaggi
// void sendPWMMessage(uint8_t motorIndex, int16_t pwmValue) {
//   // Crea un messaggio PWM
//   Communication::MsgPWM msg_pwm;

//   // Imposta il valore PWM per il motore specificato
//   msg_pwm.setPwm(motorIndex, pwmValue);

//   // Invia il messaggio tramite il canale di comunicazione (seriale)
//   if (Communication::snd(&msg_pwm)) {
//     Serial.println("Messaggio PWM inviato con successo!");
//   } else {
//     Serial.println("Errore nell'invio del messaggio PWM.");
//   }
// }

// void sendMotorControlMessage(uint8_t motorIndex, int32_t encoderValue, int8_t direction) {
//   // Crea un messaggio di controllo motore
//   Communication::MsgMOTOR msg_motor;

//   // Imposta i parametri del motore
//   msg_motor.setIndex(motorIndex);
//   msg_motor.setEncoderValue(encoderValue);
//   msg_motor.setSpinDirection(direction);  // 1 per avanti, -1 per indietro
//   // Invia il messaggio tramite il canale di comunicazione
//   if (Communication::snd(&msg_motor)) {
//     Serial.println("Messaggio di controllo motore inviato con successo!");
//   } else {
//     Serial.println("Errore nell'invio del messaggio di controllo motore.");
//   }
// }


// void cycle() {
//   // Assicurati che il ciclo esegua l'azione dopo l'invio del messaggio
//   // (ad esempio, elaborando eventuali messaggi ricevuti)
//   robotComm.cycle();
// }

// void controlMotors(uint8_t motorIndex, int16_t pwmValue, int32_t encoderValue, int8_t direction) {
//   // Esegui l'azione per modificare il PWM del motore 0
//   sendPWMMessage(motorIndex, pwmValue);  // Imposta il PWM del motore 0 a 1500
//   // Esegui l'azione per modificare la direzione del motore 1
//   sendMotorControlMessage(motorIndex, encoderValue, direction);  // Imposta encoder e direzione per il motore 1
//   // Chiamata a cycle per eseguire l'aggiornamento
//   cycle();
// }