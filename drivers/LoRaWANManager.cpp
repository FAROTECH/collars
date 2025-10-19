/******************************************************************************************
 *  LoRaWANManager.cpp – Lettura chiavi da SD (compatibile con SdFat)
 ******************************************************************************************/

#include "LoRaWANManager.h"
#define SDFAT_FILE_TYPE 1
#include <SdFatConfig.h>
#include <SdFat.h>

// Usa l'istanza globale di SdFat definita in SDManager.cpp
extern SdFat sd;

uint8_t LoRaWANManager::devEUI[8] = {0};
uint8_t LoRaWANManager::appEUI[8] = {0};
uint8_t LoRaWANManager::appKEY[16] = {0};
uint8_t LoRaWANManager::nwkKEY[16] = {0};

bool LoRaWANManager::loadKeysFromSD() {
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

        // Rimuovi eventuali \r finali (file DOS)
        char* cr = strchr(line, '\r');
        if (cr) *cr = '\0';

        if (strncmp(line, "devEUI=", 7) == 0)
            parseHex(line + 7, devEUI, 8);
        else if (strncmp(line, "appEUI=", 7) == 0)
            parseHex(line + 7, appEUI, 8);
        else if (strncmp(line, "appKEY=", 7) == 0)
            parseHex(line + 7, appKEY, 16);
        else if (strncmp(line, "nwkKEY=", 7) == 0)
            parseHex(line + 7, nwkKEY, 16);
    }
    cfg.close();
    Serial.println("[LoRaWAN] Configurazione caricata correttamente.");
    printKeys();
    return true;
}

void LoRaWANManager::parseHex(const char* str, uint8_t* dest, int len) {
    for (int i = 0; i < len; i++) {
        sscanf(str + 2 * i, "%2hhx", &dest[i]);
    }
}

void LoRaWANManager::printKeys() {
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
