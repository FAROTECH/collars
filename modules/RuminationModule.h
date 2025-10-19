/******************************************************************************************
 *  RuminationModule.h – Stub per modulo di riconoscimento ruminazione
 *  -------------------------------------------------------------
 *  Usa i campioni audio del microfono MAX9814 e un modello Edge Impulse.
 ******************************************************************************************/

#ifndef RUMINATION_MODULE_H
#define RUMINATION_MODULE_H

#include <Arduino.h>

// Numero campioni per finestra Edge Impulse (~3s a 10kHz)
#define RUMINATION_WINDOW_SAMPLES 32000

namespace RuminationModule {

    /** Inizializza le risorse del modulo */
    void init();

    /**
     * Esegue il riconoscimento di ruminazione sul buffer audio.
     * @param audio_buffer Array di campioni audio float
     * @param len          Numero di campioni (32000 tipici)
     * @return stato ruminazione (1 = ruminante, 0 = no)
     */
    uint8_t run(const float* audio_buffer, size_t len);
}

#endif
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
