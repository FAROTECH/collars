/******************************************************************************************
 *  SDManager.ino – Gestione Storage su SD card (SPI1)
 *  Versione adattata per architettura multi-INO (Cicerone Collar)
 *  ---------------------------------------------------------------
 *  Espone funzioni:
 *      bool SDManager_init();
 *      bool SDManager_isReady();
 *      bool SDManager_appendRecord(const char* jsonLine);
 ******************************************************************************************/

#include <Arduino.h>
#define SDFAT_FILE_TYPE 1
#include <SdFatConfig.h>
#include <SdFat.h>
#include "config_pins.h"

// ======================================================================================
// === ISTANZE GLOBALI E VARIABILI LOCALI ===
// ======================================================================================
SdFat sd;
static bool sd_ok = false;
static const char* data_filename = "data_log.json";

// ======================================================================================
// === FUNZIONI PUBBLICHE ===
// ======================================================================================

/**
 * Inizializza la SD card su SPI1
 */
bool SDManager_init() 
{
    // Configurazione SPI1 manuale (Cicerone)
    SPI.end();
    SPI.setMOSI(PIN_SD_MOSI);
    SPI.setMISO(PIN_SD_MISO);
    SPI.setSCLK(PIN_SD_SCK);
    SPI.begin();
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);

    delay(50); // stabilizzazione

    Serial.print("[SD] Inizializzazione... ");
    if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(4))) {   // 4 MHz = stabile su Move-X
        Serial.println("FALLITA");
        sd_ok = false;
        return false;
    }

    File test = sd.open(data_filename, FILE_WRITE);
    if (!test) {
        Serial.println("Errore apertura file!");
        sd_ok = false;
        return false;
    }
    test.close();

    Serial.println("OK");
    sd_ok = true;
    return true;
}

/**
 * Restituisce lo stato di disponibilità della SD
 */
bool SDManager_isReady() {
    return sd_ok;
}

/**
 * Scrive una riga JSON sul file data_log.json
 */
bool SDManager_appendRecord(const char* jsonLine) {
    if (!sd_ok) return false;
    File f = sd.open(data_filename, FILE_WRITE);
    if (!f) {
        Serial.println("[SD] Errore apertura file per scrittura!");
        return false;
    }
    f.println(jsonLine);
    f.close();
    Serial.println("[SD] Record aggiunto a data_log.json");
    return true;
}

/******************************************************************************************
 *  LoRaWANManager Section – Gestione chiavi LoRaWAN da SD Card
 *  Compatibile con Move-X LibLoRaWAN v1.0.3
 *  ---------------------------------------------------------------
 *  Espone funzioni:
 *      bool LoRaWANManager_loadKeysFromSD();
 *      void LoRaWANManager_printKeys();
 *
 *  Il file "lorawan.cfg" sulla SD deve contenere righe nel formato:
 *      devEUI=5DF5F921B0760A43
 *      appEUI=0000000000000000
 *      appKEY=A67AF941696B48269B4A535CDC6E2516
 *      nwkKEY=A67AF941696B48269B4A535CDC6E2516
 ******************************************************************************************/
 
static const char* LORAWAN_CONFIG_FILE = "lorawan.cfg";

// ======================================================================================
// === FUNZIONI INTERNE ===
// ======================================================================================
static void LoRaWANManager_parseHex(const char* str, uint8_t* dest, int len) {
    int j = 0;
    for (int i = 0; str[i] != '\0' && j < len; i++) {
        if (isxdigit(str[i]) && isxdigit(str[i + 1])) {
            char byteStr[3] = { str[i], str[i + 1], 0 };
            dest[j++] = (uint8_t)strtoul(byteStr, NULL, 16);
            i++;
        }
    }
    while (j < len) dest[j++] = 0x00;
}

// ======================================================================================
// === FUNZIONI PUBBLICHE ===
// ======================================================================================

/**
 * Legge le chiavi LoRaWAN dal file lorawan.cfg su SD
 */
bool LoRaWANManager_loadKeysFromSD() {
    Serial.println("[LoRaWAN] Lettura configurazione da SD...");

    File cfg = sd.open(LORAWAN_CONFIG_FILE, FILE_READ);
    if (!cfg) {
        Serial.println("[LoRaWAN] Errore: file lorawan.cfg non trovato!");
        return false;
    }

    char line[96];
    while (cfg.available()) {
        int len = cfg.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';

        // Ignora linee vuote o commentate
        if (line[0] == '#' || line[0] == '\0' || line[0] == '\r' || line[0] == '\n')
            continue;

        // Rimuovi eventuali \r (file DOS)
        char* cr = strchr(line, '\r');
        if (cr) *cr = '\0';

        if (strncmp(line, "devEUI=", 7) == 0)
            LoRaWANManager_parseHex(line + 7, devEUI, 8);
        else if (strncmp(line, "appEUI=", 7) == 0)
            LoRaWANManager_parseHex(line + 7, appEUI, 8);
        else if (strncmp(line, "appKEY=", 7) == 0)
            LoRaWANManager_parseHex(line + 7, appKEY, 16);
        else if (strncmp(line, "nwkKEY=", 7) == 0)
            LoRaWANManager_parseHex(line + 7, nwkKEY, 16);
    }
    cfg.close();

    Serial.println("[LoRaWAN] Configurazione caricata correttamente.");
    LoRaWANManager_printKeys();
    return true;
}

/**
 * Stampa le chiavi su seriale
 */
void LoRaWANManager_printKeys() {
    auto printArray = [](const char* name, const uint8_t* arr, int len) {
        Serial.printf("  %s = { ", name);
        for (int i = 0; i < len; i++) {
            Serial.printf("0x%02X", arr[i]);
            if (i < len - 1) Serial.print(", ");
        }
        Serial.println(" };");
    };

    Serial.println("[LoRaWAN] Chiavi attive:");
    printArray("devEUI", devEUI, 8);
    printArray("appEUI", appEUI, 8);
    printArray("appKEY", appKEY, 16);
    printArray("nwkKEY", nwkKEY, 16);
}

/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
