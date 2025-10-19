/******************************************************************************************
 *  config_sensors.h – Cicerone Board (Rev. Ottobre 2025)
 *  ------------------------------------------------------
 *  Configurazioni generali dei sensori, frequenze di campionamento e parametri di filtro.
 *  Ogni sensore può essere abilitato/disabilitato in modo indipendente.
 ******************************************************************************************/

#ifndef CONFIG_SENSORS_H
#define CONFIG_SENSORS_H

// ==========================================================
// === Abilitazione dei moduli ===
// ==========================================================
#define ENABLE_LIS3DH          1     // Accelerometro 3-assi
#define ENABLE_BME280          1     // Temperatura / Pressione / Umidità
#define ENABLE_MLX90614        1     // Termometro IR (ambiente + oggetto)
#define ENABLE_AD8232          1     // ECG / BPM
#define ENABLE_MAX9814         1     // Microfono – Livello sonoro
#define ENABLE_BATTERY_MONITOR 1     // Lettura tensione batteria (sensore virtuale)
#define ENABLE_GPS             1     // Modulo GPS Move-X (USART1)
#define ENABLE_SD_STORAGE      1     // Salvataggio dati su SD
#define ENABLE_LORA_TX         1     // Trasmissione LoRa SX1262
#define ENABLE_LOG_SERIAL      1     // Log seriale di debug

// ==========================================================
// === Frequenze di campionamento (Hz o secondi) ===
// ==========================================================
// Frequenze sensori fisici
#define FREQ_LIS3DH_HZ         10        // Accelerometro: 10 letture/sec
#define FREQ_BME280_HZ         1         // Ambiente: 1 lettura/sec
#define FREQ_MLX90614_HZ       1         // IR: 1 lettura/sec
#define FREQ_AD8232_HZ         100       // ECG campionamento: 100 Hz
#define FREQ_MAX9814_HZ        10000     // Audio ADC: 10 kHz
#define SOUND_WINDOW_MS        100       // Finestra RMS 100 ms

// Sensori virtuali e servizi
#define FREQ_BATTERY_HZ        0.2       // Ogni 5 s
#define FREQ_LORA_TX_SEC       30        // Trasmissione payload ogni 30 s
#define FREQ_SD_FLUSH_SEC      60        // Scrittura file su SD ogni 60 s
#define HEALTHCHECK_PERIOD_SEC 120       // Task diagnostico periodico

// ==========================================================
// === Filtri e parametri di elaborazione ===
// ==========================================================
// LIS3DH – Accelerometro
#define LIS3DH_AVG_SAMPLES     8         // Media mobile su 8 campioni

// BME280 – Sensore ambiente
#define BME280_AVG_SAMPLES     5         // Media su 5 letture

// MLX90614 – IR
#define MLX90614_AVG_SAMPLES   3         // Media su 3 letture

// AD8232 – ECG / BPM
#define AD8232_WINDOW_MS       5000      // Calcolo BPM ogni 5 s
#define AD8232_THRESHOLD_MV    200       // Soglia minima di rilevazione picco

// MAX9814 – Microfono
#define SOUND_VREF_V           1.0f      // Tensione di riferimento RMS
#define SOUND_DB_MIN           30.0f     // Livello minimo stimato dB SPL
#define SOUND_DB_MAX           120.0f    // Livello massimo stimato dB SPL

// Battery Monitor
#define VBAT_FILTER_ALPHA      0.2f      // Filtro esponenziale
#define VBAT_LOW_VOLTAGE       3.30f     // Soglia batteria scarica
#define VBAT_FULL_VOLTAGE      4.20f     // Soglia batteria piena

// ==========================================================
// === Parametri generali di sistema ===
// ==========================================================
#define LOG_BAUDRATE           115200    // Porta seriale per debug
#define GPS_BAUDRATE           9600      // Baudrate GPS Move-X (NMEA)
#define ADC_RESOLUTION_BITS    12
#define ADC_VREF_VOLTAGE       3.3f

#endif // CONFIG_SENSORS_H
/******************************************************************************************
 *  END OF CONFIG FILE
 ******************************************************************************************/
