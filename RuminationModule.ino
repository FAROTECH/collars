/******************************************************************************************
 *  RuminationModule.ino – Rilevamento attività di ruminazione (audio MAX9814)
 *  Versione adattata per architettura multi-INO (Cicerone Collar)
 *  ---------------------------------------------------------------
 *  Espone funzioni:
 *      void  RuminationModule_init();
 *      uint8_t RuminationModule_run(const int16_t* audioBuffer, int nSamples);
 *
 *  Nota:
 *      Attualmente implementa una versione "stub" con soglia RMS fissa.
 *      Il codice potrà essere esteso con analisi FFT o ML a bordo.
 ******************************************************************************************/

#include <Arduino.h>

// ======================================================================================
// === VARIABILI LOCALI ===
// ======================================================================================
static bool rumination_ready = false;
static const float RUMINATION_THRESHOLD = 80.0f;   // RMS empirico (valore da calibrare)

// ======================================================================================
// === FUNZIONI PUBBLICHE ===
// ======================================================================================

/**
 * Inizializza il modulo di analisi audio per la ruminazione
 */
void RuminationModule_init() {
    Serial.println("[Rumination] Modulo ruminazione inizializzato.");
    rumination_ready = true;
}

/**
 * Analizza il buffer audio e restituisce 1 se attività di ruminazione rilevata
 */
uint8_t RuminationModule_run(const int16_t* audioBuffer, int nSamples) {
    if (!rumination_ready) {
        Serial.println("[Rumination] Modulo non inizializzato.");
        return 0;
    }

    if (audioBuffer == nullptr || nSamples <= 0) {
        Serial.println("[Rumination] Buffer audio non valido.");
        return 0;
    }

    // Calcolo RMS del segnale (energia sonora)
    double sumSq = 0.0;
    for (int i = 0; i < nSamples; i++) {
        sumSq += (double)audioBuffer[i] * audioBuffer[i];
    }
    double rms = sqrt(sumSq / nSamples);

    // Soglia fissa (stub): sopra → ruminazione attiva
    uint8_t ruminating = (rms > RUMINATION_THRESHOLD) ? 1 : 0;

    Serial.print("[Rumination] RMS=");
    Serial.print(rms, 2);
    Serial.print("  -> Ruminating=");
    Serial.println(ruminating);
    
    return ruminating;
}

/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
