/******************************************************************************************
 *  MotionModule.cpp – Stub per modulo LIS3DH + ML (emlearn)
 *  --------------------------------------------------------
 *  Integra la logica di inferenza sui dati accelerometrici.
 ******************************************************************************************/

#include "MotionModule.h"

// In futuro: #include "features.c" e modelli emlearn (.h)
// #include "grazing.h"
// #include "sitting.h"
// #include "standing.h"
// #include "walking.h"

void MotionModule::init() {
    Serial.println("[MotionModule] Init completed.");
}

/**
 * Stub per inferenza ML su LIS3DH.
 * In versione completa estrarrà le features e chiamerà i modelli emlearn.
 */
bool MotionModule::run(const float* x, const float* y, const float* z, size_t len,
                       uint8_t &grazing, uint8_t &sitting, uint8_t &standing, uint8_t &walking) {

    if (len < MOTION_WINDOW_SAMPLES) {
        Serial.println("[MotionModule] Not enough samples.");
        return false;
    }

    // TODO: calcolo features → estrazione + classificazione
    // Per ora: valori casuali simulati per test del flusso
    grazing  = random(0, 2);
    sitting  = random(0, 2);
    standing = random(0, 2);
    walking  = random(0, 2);

    Serial.printf("[MotionModule] Mock results -> G:%d S:%d ST:%d W:%d\n",
                  grazing, sitting, standing, walking);

    return true;
}
