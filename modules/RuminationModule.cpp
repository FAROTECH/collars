/******************************************************************************************
 *  RuminationModule.cpp – Stub per modulo MAX9814 + ML (Edge Impulse)
 *  -------------------------------------------------------------
 *  In versione completa chiamerà la libreria generata da Edge Impulse.
 ******************************************************************************************/

#include "RuminationModule.h"

// In futuro: #include "RicardoCllT-project-1_inferencing.h"

void RuminationModule::init() {
    Serial.println("[RuminationModule] Init completed.");
}

/**
 * Stub del classificatore Edge Impulse.
 * Per ora restituisce un valore casuale per test del flusso.
 */
uint8_t RuminationModule::run(const float* audio_buffer, size_t len) {

    if (len < RUMINATION_WINDOW_SAMPLES) {
        Serial.println("[RuminationModule] Not enough audio samples.");
        return 0;
    }

    // TODO: sostituire con chiamata Edge Impulse:
    // signal_t signal; signal.get_data = raw_feature_get_data;
    // run_classifier(&signal, &result, false);

    uint8_t state = random(0, 2);
    Serial.printf("[RuminationModule] Mock result -> ruminate=%d\n", state);

    return state;
}
