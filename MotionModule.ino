/******************************************************************************************
 *  MotionModule.ino – Analisi del movimento (Accelerometro LIS3DH)
 *  Versione adattata per architettura multi-INO (Cicerone Collar)
 *  ---------------------------------------------------------------
 *  Espone funzioni:
 *      void MotionModule_init();
 *      void MotionModule_run(float* x, float* y, float* z, int n,
 *                             uint8_t& grazing, uint8_t& sitting,
 *                             uint8_t& standing, uint8_t& walking);
 ******************************************************************************************/

#include <Arduino.h>
#include <Adafruit_LIS3DH.h>

// ======================================================================================
// === VARIABILI LOCALI ===
// ======================================================================================
static bool motion_ready = false;

// ======================================================================================
// === FUNZIONI PUBBLICHE ===
// ======================================================================================

/**
 * Inizializzazione del modulo di analisi movimento
 */
void MotionModule_init() {
    Serial.println("[Motion] Modulo movimento inizializzato.");
    motion_ready = true;
}

/**
 * Esegue analisi di base del movimento in base ai dati accelerometrici
 */
void MotionModule_run(float* x, float* y, float* z, int n,
                      uint8_t& grazing, uint8_t& sitting,
                      uint8_t& standing, uint8_t& walking)
{
    if (!motion_ready) {
        Serial.println("[Motion] Modulo non inizializzato.");
        return;
    }

    float sum_energy = 0.0f;
    for (int i = 0; i < n; i++) {
        float mag = sqrtf(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
        sum_energy += mag;
    }
    float avg_energy = sum_energy / n;

    grazing  = (avg_energy > 0.15f && avg_energy <= 0.40f);
    walking  = (avg_energy > 0.40f);
    standing = (avg_energy <= 0.15f && avg_energy > 0.05f);
    sitting  = (avg_energy <= 0.05f);

    Serial.print("[Motion] E=");
    Serial.print(avg_energy, 3);
    Serial.print(" -> Grazing:");
    Serial.print(grazing);
    Serial.print("  Walking:");
    Serial.print(walking);
    Serial.print("  Standing:");
    Serial.print(standing);
    Serial.print("  Sitting:");
    Serial.println(sitting);
}
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
