/******************************************************************************************
 *  data_structs.h – Compact telemetry record for LoRaWAN/TTN transmission
 *  ----------------------------------------------------------------------
 *  Struttura dati ottimizzata per invio JSON compatto (<242 B)
 *  Compatibile con serializeTelemetryJSON() e Task_ECG() aggiornati.
 ******************************************************************************************/

#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <stdint.h>

/**
 * Struttura principale di telemetria del collare Cicerone.
 * Tutti i valori sono aggregati e pronti per serializzazione JSON.
 */
typedef struct {
    uint32_t timestamp;      // Epoch UTC (seconds)
    double   latitude;       // Posizione GPS
    double   longitude;      // Posizione GPS
    float    temp_body;      // MLX90614 – temperatura corpo (°C)
    float    temp_ambient;   // BME280 – temperatura ambiente (°C)
    float    humidity;  // Umidità relativa (%)
    float    heart_bpm;      // BPM medio da AD8232 (pecora 60–120)
    uint8_t  ruminate;       // ML audio MAX9814 → 1/0
    uint8_t  grazing;        // ML movimento LIS3DH → 1/0
    uint8_t  standing;       // ML movimento LIS3DH → 1/0
    uint8_t  walking;        // ML movimento LIS3DH → 1/0
    uint8_t  sitting;        // ML movimento LIS3DH → 1/0
    float    batt_voltage;   // Tensione batteria (V)
} telemetryRecord_t;

#endif
/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
