/******************************************************************************************
 *  SDManager.cpp – Gestione Storage su SD card (SPI1)
 ******************************************************************************************/

#include "SDManager.h"

#define SDFAT_FILE_TYPE 1
#include <SdFatConfig.h>
#include <SdFat.h>

// Istanza globale di SdFat condivisa da tutto il firmware
SdFat sd;

static File dataFile;
static bool sd_ok = false;
constexpr const char* filename = "data_log.json";

bool SDManager::init() {
    SPI.setMOSI(PIN_SD_MOSI);
    SPI.setMISO(PIN_SD_MISO);
    SPI.setSCLK(PIN_SD_SCK);
    pinMode(PIN_SD_CS, OUTPUT);

    Serial.print("[SD] Inizializzazione... ");
    if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(18))) {
        Serial.println("FALLITA");
        sd_ok = false;
        return false;
    }

    // Test apertura file
    dataFile = sd.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.println("Errore apertura file!");
        sd_ok = false;
        return false;
    }
    dataFile.close();

    Serial.println("OK");
    sd_ok = true;
    return true;
}

bool SDManager::isReady() {
    return sd_ok;
}

/**
 * Scrive una riga JSON sul file
 */
bool SDManager::appendRecord(const char* jsonLine) {
    if (!sd_ok) {
        Serial.println("[SD] Non inizializzata!");
        return false;
    }

    dataFile = sd.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.println("[SD] Errore apertura file!");
        return false;
    }

    dataFile.println(jsonLine);
    dataFile.close();

    Serial.println("[SD] Record salvato su SD.");
    return true;
}
