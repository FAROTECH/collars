/******************************************************************************************
 *  MotionModule.h – Stub per modulo di riconoscimento movimento animale
 *  -------------------------------------------------------------
 *  Usa i dati dell'accelerometro LIS3DH e i modelli emlearn
 *  per determinare gli stati: grazing, sitting, standing, walking.
 ******************************************************************************************/

#ifndef MOTION_MODULE_H
#define MOTION_MODULE_H

#include <Arduino.h>

// Numero di campioni per finestra
#define MOTION_WINDOW_SAMPLES 300

namespace MotionModule {

    /** Inizializza le risorse del modulo (eventuali modelli o buffer) */
    void init();

    /**
     * Esegue l'inferenza ML sul movimento.
     * @param x, y, z     Array di campioni accelerometrici
     * @param len         Numero di campioni per asse
     * @param grazing     [out] stato grazing (1/0)
     * @param sitting     [out] stato sitting (1/0)
     * @param standing    [out] stato standing (1/0)
     * @param walking     [out] stato walking (1/0)
     * @return true se l'inferenza è andata a buon fine
     */
    bool run(const float* x, const float* y, const float* z, size_t len,
             uint8_t &grazing, uint8_t &sitting, uint8_t &standing, uint8_t &walking);
}

#endif
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/


