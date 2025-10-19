/******************************************************************************************
 *  LoRaWANTransmitter.cpp – Gestione uplink LoRaWAN (API Move-X v1.0.3)
 ******************************************************************************************/

#include "LoRaWANTransmitter.h"
#include "LoRaWANManager.h"
#include <LibLoRaWAN.h>

bool net_joined = false;
unsigned long lastTxMillis = 0;
#define TX_INTERVAL_MS  30000UL
#define LORA_PORT       1

// Funzione locale
static void userloop();

/**
 * Inizializza lo stack LoRaWAN e avvia il join OTAA
 */
bool LoRaWANTransmitter::init() {
    Serial.println("[LoRaWAN] Inizializzazione stack...");

    // Collega il loop di gestione interno (non-blocking)
    LoRaWAN.attachLoop(userloop);

    // Avvia la libreria (false = no verbose debug)
    LoRaWAN.begin(false);

    // Configura regione e chiavi (caricate da SD)
    if (LRW_OK != LoRaWAN.setRegion(LRW_REGION_EU868)) Serial.println("setRegion() error!");
    if (LRW_OK != LoRaWAN.setDevEUI(LoRaWANManager::devEUI)) Serial.println("setDevEUI() error!");
    if (LRW_OK != LoRaWAN.setJoinEUI(LoRaWANManager::appEUI)) Serial.println("setJoinEUI() error!");
    if (LRW_OK != LoRaWAN.setAppKey(LoRaWANManager::appKEY)) Serial.println("setAppKey() error!");
    if (LRW_OK != LoRaWAN.setNwkKey(LoRaWANManager::nwkKEY)) Serial.println("setNwkKey() error!");

    // Avvia il join OTAA
    if (LRW_OK != LoRaWAN.Join(LRW_JOIN_OTAA)) {
        Serial.println("[LoRaWAN] Join OTAA FALLITO.");
        return false;
    }

    Serial.println("[LoRaWAN] Join OTAA in corso...");
    lastTxMillis = millis();
    return true;
}

/**
 * Verifica se il dispositivo è unito alla rete
 */
bool LoRaWANTransmitter::isJoined() {
    return net_joined;
}

/**
 * Invia un pacchetto JSON via LoRaWAN (una volta unito)
 */
bool LoRaWANTransmitter::send(const char* payload) {
    if (!net_joined) {
        Serial.println("[LoRaWAN] Nodo non ancora unito – impossibile inviare.");
        return false;
    }

    uint8_t TxSize = strlen(payload);
    uint8_t TxData[TxSize];
    memcpy(TxData, payload, TxSize);

    Serial.print("[LoRaWAN] TX (Port ");
    Serial.print(LORA_PORT);
    Serial.print("): '");
    Serial.print(payload);

    if (LRW_OK != LoRaWAN.Send(LORA_PORT, LRW_UNCONFIRMED_MSG, TxData, TxSize)) {
        Serial.println("' -> ERRORE invio!");
        return false;
    }

    Serial.println("' -> OK");
    return true;
}

/**
 * Loop utente gestito dalla libreria
 */
static void userloop() {
    LoRaWAN.process();

    if (!net_joined) {
        if (LRW_OK == LoRaWAN.NetJoined()) {
            net_joined = true;
            Serial.println("[LoRaWAN] Join completato con successo!");
            lastTxMillis = millis();
        }
    } else {
        unsigned long currentMillis = millis();
        if (currentMillis - lastTxMillis >= TX_INTERVAL_MS) {
            lastTxMillis = currentMillis;
            const char* testMsg = "{\"ping\":true}";
            uint8_t TxSize = strlen(testMsg);
            uint8_t TxData[TxSize];
            memcpy(TxData, testMsg, TxSize);

            if (LRW_OK == LoRaWAN.Send(LORA_PORT, LRW_UNCONFIRMED_MSG, TxData, TxSize)) {
                Serial.println("[LoRaWAN] Ping periodico inviato.");
            } else {
                Serial.println("[LoRaWAN] Errore invio ping.");
            }
        }
    }
}

/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
