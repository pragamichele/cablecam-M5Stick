/*
  This Arduino code is provided "as is", without any warranty of any kind.
  It may be used, modified, and distributed exclusively for non-commercial purposes.
  Use for commercial purposes is expressly prohibited without prior written consent from the author.
  
  Author: Praga Michele

  License: Non-commercial use only
*/

#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>

// Configura il nome e la password dell'Access Point
const char* ap_ssid = "CableCam_AP";     // Nome della rete WiFi che l'ESP8266 creerà
const char* ap_password = "12345678";    // Password per connettersi alla rete WiFi

// Configura il WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// Pin dei motori
const int in1Pin = D8;  // Pin IN1
const int in2Pin = D7;  // Pin IN2
const int in3Pin = D6;  // Pin IN3
const int in4Pin = D5;  // Pin IN4

const int redLedPin = D1;   // LED rosso per connessione non riuscita
const int greenLedPin = D2; // LED verde per connessione riuscita

int motorSpeed = 0;   // Velocità attuale dei motori
int targetSpeed = 0;  // Velocità desiderata

unsigned long lastReceivedTime = 0;  // Timer per monitorare il tempo trascorso dall'ultimo segnale
unsigned long lastLedToggleTime = 0; // Timer per gestire il lampeggio del LED verde

const int step = 10;  // Incremento per la rampa di accelerazione e frenata
const int rampDelay = 5;  // Ritardo tra ogni incremento della rampa (in millisecondi)
const unsigned long signalTimeout = 500;  // Timeout di 2 secondi per il segnale
const int ledOnDuration = 300;  // Durata accensione del LED verde (in millisecondi)

void setup() {
    Serial.begin(115200);

    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);

    pinMode(redLedPin, OUTPUT);
    pinMode(greenLedPin, OUTPUT);

    digitalWrite(redLedPin, HIGH);  // LED rosso acceso all'inizio (non connesso)
    digitalWrite(greenLedPin, LOW); // LED verde spento all'inizio

    // Configura l'ESP8266 in modalità Access Point
    WiFi.softAP(ap_ssid, ap_password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point started. IP Address: ");
    Serial.println(IP);

    // Avvia il WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started.");
      // Imposta la potenza del WiFi al massimo (20.5 dBm)
    WiFi.setOutputPower(20.5);

}

void loop() {
    webSocket.loop();  // Mantieni il WebSocket attivo

    // Controllo del timeout per fermare i motori se non si riceve un segnale per più di 2 secondi
    if (millis() - lastReceivedTime > signalTimeout) {
        targetSpeed = 0;  // Ferma i motori se il segnale è perso
        digitalWrite(greenLedPin, LOW);  // Spegni il LED verde
        digitalWrite(redLedPin, HIGH);   // Accendi il LED rosso per segnalare la perdita di connessione
    }

    // Se ci sono dispositivi connessi, gestisci il LED verde in base alla velocità del motore
    if (millis() - lastReceivedTime <= signalTimeout) {
        digitalWrite(redLedPin, LOW);    // Spegni il LED rosso quando la connessione è attiva

        // Se il motore non è in movimento, il LED verde rimane acceso fisso
        if (targetSpeed == 0) {
            digitalWrite(greenLedPin, HIGH);  // LED verde acceso fisso
        } 
        // Se il motore è in movimento, fai lampeggiare il LED verde
        else {
            int blinkInterval = map(abs(targetSpeed), 0, 100, 1000, 100);  // Mappa la velocità al tempo di lampeggio

            // Accendi il LED verde per `ledOnDuration` ms, poi spegnilo per il resto dell'intervallo
            if (millis() - lastLedToggleTime >= blinkInterval) {
                bool ledState = digitalRead(greenLedPin);  // Leggi lo stato attuale del LED verde
                digitalWrite(greenLedPin, !ledState);      // Cambia lo stato del LED
                lastLedToggleTime = millis();
            }
        }
    }

    updateMotorSpeed();
    delay(10);
}

// Funzione chiamata ogni volta che c'è un evento WebSocket
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        String message = String((char*)payload);
        Serial.println("Received: " + message);
        message.trim();

        if (message.startsWith("SPEED_")) {
            int receivedSpeed = message.substring(6).toInt();
            if (receivedSpeed > 50) {
                targetSpeed = map(receivedSpeed, 51, 100, 0, 100);  // Mappa velocità avanti
            } else if (receivedSpeed < 50) {
                targetSpeed = map(receivedSpeed, 49, 0, 0, -100);  // Mappa velocità indietro
            } else {
                targetSpeed = 0;  // Se la velocità ricevuta è 50 (orizzontale), ferma il motore
            }
            lastReceivedTime = millis();  // Aggiorna il tempo dell'ultimo segnale ricevuto
        } else if (message.startsWith("STOP")) {
            targetSpeed = 0;
            lastReceivedTime = millis();  // Aggiorna il tempo dell'ultimo segnale ricevuto
        }
    }
}

void updateMotorSpeed() {
    static int currentSpeed = 0;

    if (currentSpeed != targetSpeed) {
        if (targetSpeed > currentSpeed) {
            currentSpeed += step;
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        } else if (targetSpeed < currentSpeed) {
            currentSpeed -= step;
            if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }

        if (currentSpeed > 0) {
            analogWrite(in1Pin, map(currentSpeed, 0, 100, 0, 255));
            analogWrite(in3Pin, map(currentSpeed, 0, 100, 0, 255));
            digitalWrite(in2Pin, LOW);
            digitalWrite(in4Pin, LOW);
        } else if (currentSpeed < 0) {
            analogWrite(in2Pin, map(-currentSpeed, 0, 100, 0, 255));
            analogWrite(in4Pin, map(-currentSpeed, 0, 100, 0, 255));
            digitalWrite(in1Pin, LOW);
            digitalWrite(in3Pin, LOW);
        } else {
            analogWrite(in1Pin, 0);
            analogWrite(in2Pin, 0);
            analogWrite(in3Pin, 0);
            analogWrite(in4Pin, 0);
        }

        Serial.println("Updated motor speed to: " + String(currentSpeed));
        delay(rampDelay);  // Aggiunge un ritardo tra gli incrementi per una rampa più lenta
    }
}
