/******************************************************************************************
 *  SDManager.h – Gestione Storage su SD card (SPI1)
 *  ------------------------------------------------
 *  Livello 5: STORAGE & RECOVERY LAYER
 ******************************************************************************************/

#ifndef SDMANAGER_H
#define SDMANAGER_H

#include <Arduino.h>
#include "../config_pins.h"

#define SDFAT_FILE_TYPE 1
#include <SdFatConfig.h>
#include <SdFat.h>

namespace SDManager {

    /** Inizializza la SD card */
    bool init();

    /** Scrive un record JSON (una riga) nel file data_log.json */
    bool appendRecord(const char* jsonLine);

    /** Controlla lo stato di inizializzazione */
    bool isReady();
}

#endif
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
