// ======================================================================================
// === PERIODI DI ESECUZIONE TASK (ms) ================================================
// ======================================================================================

#pragma once

/* PRODUZIONE 
#define SENSOR_READ_INTERVAL_MS      60000UL    // 60 secondi → acquisizione sensori
#define GPS_READ_INTERVAL_MS         60000UL    // 60 secondi → lettura GPS
#define ML_EXEC_INTERVAL_MS          30000UL    // 30 secondi → inferenza ML
#define TELEMETRY_AGG_INTERVAL_MS   300000UL    // 5 minuti → aggregazione record
#define TX_INTERVAL_MS              600000UL    // 10 minuti → trasmissione dati
#define LOOP_DELAY_MS                   50UL    // 50 ms → ritardo tra cicli principali
*/

/* DEV */
#define SENSOR_READ_INTERVAL_MS      60000UL    // 60 secondi → acquisizione sensori
#define GPS_READ_INTERVAL_MS         60000UL    // 60 secondi → lettura GPS
#define ML_EXEC_INTERVAL_MS          30000UL    // 30 secondi → inferenza ML
#define TELEMETRY_AGG_INTERVAL_MS   120000UL    // 2 minuti → aggregazione record
#define TX_INTERVAL_MS              150000UL    // 2.5 minuti → trasmissione dati
#define LOOP_DELAY_MS                   50UL    // 50 ms → ritardo tra cicli principali