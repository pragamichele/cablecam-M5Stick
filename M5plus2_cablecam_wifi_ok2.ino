/*
  This Arduino code is provided "as is", without any warranty of any kind.
  It may be used, modified, and distributed exclusively for non-commercial purposes.
  Use for commercial purposes is expressly prohibited without prior written consent from the author.
  
  Author: Praga Michele

  License: Non-commercial use only
*/

#include <M5Unified.h>  // Libreria M5Unified
#include <WiFi.h>
#include <WebSocketsClient.h>

// Configura il nome e la password del WiFi a cui il M5StickC Plus si connetterà (rete creata dall'ESP8266)
const char* ssid = "CableCam_AP";  // Nome della rete WiFi creata dall'ESP8266
const char* password = "12345678";  // Password della rete WiFi

// Configura il WebSocket client
WebSocketsClient webSocket;

// Variabili per la gestione dello stato del motore e del display
int motorSpeed = 0;  // Variabile per memorizzare la velocità corrente
int displaySpeed = 0;  // Variabile per memorizzare la velocità visualizzata
unsigned long lastActivityTime = 0;  // Timer per monitorare l'inattività
unsigned long lastDisplayUpdate = 0;  // Timer per controllare la frequenza di aggiornamento dello schermo
bool displayOn = true;  // Stato del display
unsigned long lastWifiUpdate = 0;  // Timer per aggiornare il segnale WiFi
const unsigned long wifiUpdateInterval = 5000;  // Aggiorna ogni 5 secondi

void setup() {
    // Inizializzazione del M5StickC Plus
    auto cfg = M5.config();  // Ottieni la configurazione di default
    M5.begin(cfg);
    
    if (M5.Imu.isEnabled()) {
        M5.Imu.begin();  // Inizializza l'accelerometro, se disponibile
    }

    M5.Lcd.setRotation(1);  // Ruota il display di 90 gradi
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Connecting to WiFi..                      Mini Cablecam                           By Praga Michele");

    // Connetti il M5StickC Plus alla rete WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi!");

    // Aggiorna lo schermo per mostrare la connessione WiFi
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Connected to WiFi");

    // Configura il WebSocket client
    webSocket.begin("192.168.4.1", 81, "/");  // Indirizzo IP e porta del server ESP8266
    webSocket.onEvent(webSocketEvent);

    updateDisplay();  // Inizializza la visualizzazione
}

void loop() {
    M5.update();
    webSocket.loop();  // Mantieni la connessione WebSocket attiva

    // Aggiorna la visualizzazione del segnale WiFi ogni 5 secondi
    if (millis() - lastWifiUpdate > wifiUpdateInterval) {
        updateWifiSignal();
        lastWifiUpdate = millis();
    }

    // Riaccendi lo schermo alla pressione del pulsante
    if (M5.BtnA.wasPressed()) {
        if (!displayOn) {
            M5.Lcd.wakeup();  // Riaccendi il display
            displayOn = true;
            updateDisplay();  // Aggiorna la visualizzazione
        }
        lastActivityTime = millis();  // Resetta il timer di inattività
    }

    // Spegni lo schermo dopo 22 secondi di inattività (20 secondi + 2 secondi extra)
    if (displayOn && millis() - lastActivityTime > 22000) {
        M5.Lcd.sleep();  // Spegni il display
        displayOn = false;
    }

    // Invia la velocità tramite WebSocket quando il pulsante A è premuto
    if (M5.BtnA.isPressed()) {
        // Leggi l'inclinazione dall'accelerometro
        float accX, accY, accZ;
        M5.Imu.getAccel(&accX, &accY, &accZ);
        
        // Mappa l'inclinazione lungo l'asse Y (da -1 a 1) a una velocità percentuale (da 0% a 100%)
        motorSpeed = map(accY * 100, -100, 100, 0, 100);
        motorSpeed = constrain(motorSpeed, 0, 100);  // Limita la velocità tra 0% e 100%

        // Invia la velocità tramite WebSocket
        String message = "SPEED_" + String(motorSpeed);
        webSocket.sendTXT(message);
    }

    // Invia il comando STOP quando il pulsante viene rilasciato
    if (M5.BtnA.wasReleased()) {
        String message = "STOP";
        webSocket.sendTXT(message);
        motorSpeed = 0;
    }

    // Aggiorna il display meno frequentemente
    if (millis() - lastDisplayUpdate > 100) {  // Aggiorna ogni 100ms
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    delay(10);
}

// Funzione di callback per eventi WebSocket (puoi estenderla se necessario)
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("Disconnected from WebSocket server");
            break;
        case WStype_CONNECTED:
            Serial.println("Connected to WebSocket server");
            break;
        case WStype_TEXT:
            Serial.printf("Received text: %s\n", payload);
            break;
    }
}

// Funzione per aggiornare il display
void updateDisplay() {
    // Cancella lo schermo
    M5.Lcd.fillScreen(TFT_BLACK);

    // Mostra le iniziali "MP" a sinistra dello schermo
    M5.Lcd.setTextSize(4);  // Imposta una dimensione grande per le iniziali
    M5.Lcd.setTextColor(TFT_BLUE);
    M5.Lcd.setCursor(10, 20);  // Posiziona le iniziali
    M5.Lcd.println("MP");

    // Mappa la velocità per la visualizzazione: 100% -> 100, 50% -> 0, 0% -> -100
    if (motorSpeed == 0) {
        displaySpeed = 0;  // Visualizza 0 quando il motore è fermo
    } else {
        displaySpeed = map(motorSpeed, 0, 100, -100, 100);
    }

    // Disegna una banda colorata in alto e in basso
    M5.Lcd.fillRect(0, 0, M5.Lcd.width(), 5, TFT_GREEN);
    M5.Lcd.fillRect(0, M5.Lcd.height() - 5, M5.Lcd.width(), 5, TFT_GREEN);

    // Imposta il testo della velocità al centro dello schermo con un carattere grande
    M5.Lcd.setTextSize(5); // Imposta una dimensione del testo più grande
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor((M5.Lcd.width() - M5.Lcd.textWidth(String(displaySpeed))) / 2, (M5.Lcd.height() - 38) / 2); // Centra il testo
    M5.Lcd.printf("%d", displaySpeed);

    // Mostra la potenza del segnale WiFi (Spostata sopra la batteria)
    updateWifiSignal();

    // Mostra la tensione della batteria in basso a destra
    M5.Lcd.setTextSize(3);  // Aumenta la dimensione del testo della batteria
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(M5.Lcd.width() - 150, M5.Lcd.height() - 38);  // Posiziona la scritta in basso a destra
    int batteryLevel = M5.Power.getBatteryLevel();
    M5.Lcd.printf("BAT.%d%%", batteryLevel);  // Visualizza il livello della batteria con "BAT" davanti

    // Se la batteria è sotto il 20%, emetti un suono di avviso
    if (batteryLevel < 10) {
        M5.Speaker.setVolume(100);  // Imposta il volume
        M5.Speaker.tone(10000, 100);  // Emetti un tono di avviso
        delay(100);
        M5.Speaker.tone(4000, 20);  // Emetti un secondo tono
    }
}

// Funzione per aggiornare il segnale WiFi
void updateWifiSignal() {
    // Verifica se la connessione WiFi è ancora attiva
    if (WiFi.status() == WL_CONNECTED) {
        // Leggi l'RSSI (potenza del segnale WiFi)
        int rssi = WiFi.RSSI();

        // Calcola la qualità del segnale in percentuale
        int wifiQuality = map(rssi, -90, -30, 0, 100);  // Mappa da -90dBm a -30dBm (potenza minima e massima)
        wifiQuality = constrain(wifiQuality, 0, 100);  // Limita la qualità tra 0 e 100%

        // Mostra la percentuale del segnale WiFi sopra la batteria
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(TFT_ORANGE);
        M5.Lcd.setCursor(M5.Lcd.width() - 120, M5.Lcd.height() - 118);  // Posiziona la scritta sopra la batteria
        M5.Lcd.printf("WiFi:%d%%", wifiQuality);  // Mostra la percentuale della potenza del segnale
    } else {
        // Se la connessione WiFi è persa, imposta il segnale WiFi a 0%
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.setCursor(M5.Lcd.width() - 120, M5.Lcd.height() - 118);  // Posiziona la scritta sopra la batteria
        M5.Lcd.println("WiFi:0%");  // Mostra che il segnale WiFi è 0%
    }
}
