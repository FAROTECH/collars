/******************************************************************************************
 *  LoRaWANTransmitter.h – Gestione uplink LoRaWAN (API Move-X v1.0.3)
 *  ---------------------------------------------------------------
 *  Wrapper ad alto livello per l'invio di pacchetti JSON via LoRaWAN.
 *  Il modulo usa la libreria LibLoRaWAN del core Move-X STM32 (v1.0.3).
 *
 *  Funzioni principali:
 *    - init()      → inizializza stack e avvia join OTAA
 *    - send()      → invia un payload JSON come uplink
 *    - isJoined()  → verifica se il nodo è connesso alla rete
 ******************************************************************************************/

#ifndef LORAWANTRANSMITTER_H
#define LORAWANTRANSMITTER_H

#include <Arduino.h>
#include <LibLoRaWAN.h>

namespace LoRaWANTransmitter {

    /**
     * Inizializza il modulo LoRaWAN e avvia la procedura di join OTAA.
     * Richiede che le chiavi siano già state caricate in LoRaWANManager.
     * @return true se il join è avviato correttamente, false in caso di errore.
     */
    bool init();

    /**
     * Invia un payload JSON via uplink LoRaWAN (porta 1, messaggio non confermato).
     * @param payload  stringa JSON da trasmettere.
     * @return true se l'invio è riuscito, false in caso di errore.
     */
    bool send(const char* payload);

    /**
     * Ritorna true se il nodo risulta unito alla rete (OTAA completato).
     */
    bool isJoined();
}

#endif
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
