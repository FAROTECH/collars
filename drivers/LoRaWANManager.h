/******************************************************************************************
 *  LoRaWANManager.h – Gestione chiavi LoRaWAN da SD Card
 *  -----------------------------------------------------
 *  Livello 4 – TRANSMISSION LAYER
 *  Carica le chiavi (devEUI, appEUI, appKEY, nwkKEY) dal file di configurazione
 *  "lorawan.cfg" presente nella root della SD Card.
 ******************************************************************************************/

#ifndef LORAWANMANAGER_H
#define LORAWANMANAGER_H

#include <Arduino.h>

#define SDFAT_FILE_TYPE 1
#include <SdFatConfig.h>
#include <SdFat.h>

#define LORAWAN_CONFIG_FILE "lorawan.cfg"

namespace LoRaWANManager {

    extern uint8_t devEUI[8];
    extern uint8_t appEUI[8];
    extern uint8_t appKEY[16];
    extern uint8_t nwkKEY[16];

    /** Carica le chiavi dal file su SD */
    bool loadKeysFromSD();

    /** Stampa le chiavi correnti su seriale (debug) */
    void printKeys();

    /** Helper: converte stringa esadecimale in array uint8_t */
    void parseHex(const char* str, uint8_t* dest, int len);
}

#endif
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
