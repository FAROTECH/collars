/******************************************************************************************
 *  LoRaWANTransmitter.ino – Gestione uplink LoRaWAN (Move-X Cicerone)
 *  Basato su API reali di LibLoRaWAN v1.0.3
 *  ---------------------------------------------------------------
 *  Espone:
 *      bool LoRaWANTransmitter_init();
 *      bool LoRaWANTransmitter_isJoined();
 *      bool LoRaWANTransmitter_send(const char* payload);
 ******************************************************************************************/

// ======================================================================================
// === VARIABILI GLOBALI ===
// ======================================================================================

static bool net_joined = false;
static unsigned long lastTxMillis = 0;

#define TX_INTERVAL_MS  30000UL     // Intervallo minimo tra due uplink
#define LORA_PORT       1           // FPort
#define JOIN_TIMEOUT_MS 60000UL

// ======================================================================================
// === LOOP DI GESTIONE INTERNO ===
// ======================================================================================
static void userloop()
{
    // 1. Se non ancora joinato → verifica stato rete
    if (!net_joined)
    {
        if (LRW_OK == LoRaWAN.NetJoined()) {
            net_joined = true;
            Serial.println("[LoRaWAN] NetJoined OK – collegato alla rete.");
            lastTxMillis = millis();
        }
    }
    else
    {
        // 2. Invio periodico automatico (se serve)
        unsigned long now = millis();
        if (now - lastTxMillis >= TX_INTERVAL_MS) {
            lastTxMillis = now;
            Serial.println("[LoRaWAN] Timer TX scaduto – pronto a inviare payload.");
        }
    }
}

// ======================================================================================
// === FUNZIONI PUBBLICHE ===
// ======================================================================================

/**
 * Inizializza lo stack LoRaWAN e avvia il join OTAA
 */
bool LoRaWANTransmitter_init()
{
    Serial.println("[LoRaWAN] Inizializzazione stack Move-X...");

    // Collega il loop interno (non blocking)
    LoRaWAN.attachLoop(userloop);
    LoRaWAN.setRegion(LRW_REGION_EU868);

    // Imposta chiavi OTAA (da LoRaWANManager)
    if (LRW_OK != LoRaWAN.setDevEUI(devEUI))   Serial.println("[LoRaWAN] setDevEUI() errore!");
    if (LRW_OK != LoRaWAN.setJoinEUI(appEUI))  Serial.println("[LoRaWAN] setJoinEUI() errore!");
    if (LRW_OK != LoRaWAN.setAppKey(appKEY))    Serial.println("[LoRaWAN] setAppKey() errore!");
    if (LRW_OK != LoRaWAN.setNwkKey(nwkKEY))    Serial.println("[LoRaWAN] setNwkKey() errore!");

    // Avvia la procedura di join OTAA
    Serial.println("[LoRaWAN] Avvio OTAA Join...");
    if (LRW_OK != LoRaWAN.Join(LRW_JOIN_OTAA)) {
        Serial.println("[LoRaWAN] Join() fallito – verif. chiavi o copertura!");
        net_joined = false;
        return false;
    }

    Serial.println("[LoRaWAN] Join avviato, attendere conferma da rete...");
    unsigned long start = millis();
    while (!LoRaWAN.NetJoined() && millis() - start < JOIN_TIMEOUT_MS) {
        LoRaWAN.process();
        delay(10);
    }

    net_joined = LoRaWAN.NetJoined();
    if (net_joined)
        Serial.println("[LoRaWAN] Join completato con successo!");
    else
        Serial.println("[LoRaWAN] Join non completato (timeout).");

    return net_joined;
}

/**
 * Restituisce lo stato di connessione alla rete LoRaWAN
 */
bool LoRaWANTransmitter_isJoined()
{
    return net_joined && (LoRaWAN.NetJoined() == LRW_OK);
}

/**
 * Invia un payload JSON tramite uplink LoRaWAN
 */
bool LoRaWANTransmitter_send(const char* payload)
{
    if (!net_joined || LoRaWAN.NetJoined() != LRW_OK) {
        Serial.println("[LoRaWAN] Non joinato – impossibile inviare.");
        return false;
    }

    if (millis() - lastTxMillis < TX_INTERVAL_MS) {
        Serial.println("[LoRaWAN] Troppo presto per un nuovo invio.");
        return false;
    }

    uint8_t TxSize = strlen(payload);
    uint8_t TxData[TxSize];
    memcpy(TxData, payload, TxSize);

    Serial.printf("[LoRaWAN] TX (%d byte): %s\n", TxSize, payload);

    LRW_Error_t err = LoRaWAN.Send(LORA_PORT, LRW_UNCONFIRMED_MSG, TxData, TxSize);
    if (err != LRW_OK) {
        Serial.printf("[LoRaWAN] Errore Send(): %d\n", err);
        return false;
    }

    lastTxMillis = millis();
    Serial.println("[LoRaWAN] Uplink completato.");
    return true;
}

/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
