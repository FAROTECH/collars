/******************************************************************************************
 *  Project:   CICERONE – Smart Livestock Collar Firmware
 *  File:      main.ino
 *  Author:    CShark R&D – Embedded Systems Division
 *  Platform:  Move-X Cicerone (STM32L4 + SX1262 + GPS)
 *  Version:   1.0.0
 *  Date:      2025-10-18
 *  
 *  ----------------------------------------------------------------------------
 *  OVERVIEW:
 *  This firmware manages the acquisition, processing, aggregation, storage,
 *  and LoRaWAN transmission of biometric and environmental data collected
 *  by the Cicerone collar used for monitoring sheep in extensive farming.
 *
 *  The architecture follows a structured five-layer workflow (L1–L5):
 *
 *   L1 – SENSOR SAMPLING LAYER
 *        MLX90614  → Body temperature
 *        BME280    → Ambient temperature / pressure
 *        LIS3DH    → 3-axis accelerometer
 *        AD8232    → ECG (average BPM computation)
 *        MAX9814   → Microphone for rumination (ML audio)
 *        GPS (USART1) → Position and UTC timestamp
 *        VBAT (ADC) → Battery voltage sensing
 *
 *   L2 – PROCESSING & ML LAYER
 *        Feature extraction and on-device ML inference:
 *            - Rumination detection (audio)
 *            - Motion classification (grazing, standing, walking, sitting)
 *        Average heart-rate computation (typical sheep range: 60–120 BPM)
 *
 *   L3 – AGGREGATION LAYER
 *        Populate telemetryRecord_t with latest sensor and ML data
 *        Serialize record to compact JSON via serializeTelemetryJSON()
 *
 *   L4 – TRANSMISSION LAYER
 *        LoRaWAN transmission using LibLoRaWAN (OTAA)
 *        Transmission interval: every 10 minutes (600,000 ms)
 *        Compact JSON payload (<242 bytes) – fully compliant with IoT Networks (Pongo/TTN..)
 *          Example payload:
 *          {"id":"C1","ts":1739873400,"lt":44.98513,"ln":9.71602,"tb":38.4,
 *           "ta":22.1,"h":97.4,"r":1,"g":0,"st":1,"w":0,"s":0,"b":3.84}
 *
 *   L5 – STORAGE & RECOVERY LAYER
 *        Local backup on SD card (SPI1, file: data_log.json)
 *        Automatic fallback storage if LoRaWAN transmission fails
 *
 *  ----------------------------------------------------------------------------
 *  TRANSMISSION NOTES:
 *   Transmission period: one packet every 10 minutes
 *   Average payload size: ~150 bytes
 *   EU868 LoRaWAN limit: 242 bytes @ SF7 → fully compliant
 *   Decoder (JavaScript example):
 *       function decodeUplink(input) {
 *         const text = String.fromCharCode.apply(null, input.bytes);
 *         return { data: JSON.parse(text) };
 *       }
 *
 *  ----------------------------------------------------------------------------
 *  NOTES:
 *   All LoRaWAN keys (devEUI, appEUI, appKEY, nwkKEY) are loaded from
 *     the file "lorawan.cfg" located in the SD-card root.
 *   Each sensor can be mocked for bench testing if hardware is absent.
 *
 *  ----------------------------------------------------------------------------
 *  © 2025 CShark S.r.l. – Embedded Systems Division
 *  All rights reserved.
 ******************************************************************************************/

#include "config_pins.h"
#include "config_sensors.h"
#include "data_structs.h"

// === DRIVER & MODULE LAYER ===

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MLX90614.h>
#include <TinyGPSPlus.h>
#include <LibLoRaWAN.h>

// ======================================================================================
// === VARIABILI GLOBALI ===
// ======================================================================================

telemetryRecord_t record;

uint8_t ruminate_state  = 0;
uint8_t grazing_state   = 0;
uint8_t sitting_state   = 0;
uint8_t standing_state  = 0;
uint8_t walking_state   = 0;
float   vbat = 0.0f;

// BME280  -> I2C2
TwoWire WireBME280(PIN_BME280_SDA, PIN_BME280_SCL);
Adafruit_BME280 bme;   // BME280 su WireBME280

// MLX90614 -> I2C3
TwoWire WireMLX(PIN_MLX90614_SDA, PIN_MLX90614_SCL);
Adafruit_MLX90614 mlx; // MLX90614 su WireMLX

// LIS3DH
TwoWire WireLIS(PIN_LIS3DH_SDA, PIN_LIS3DH_SCL);
Adafruit_LIS3DH lis(&WireLIS);   // Accelerometro su WireLIS3DH

// GPS
HardwareSerial GPS(Serial1);
TinyGPSPlus gps;

bool gps_fix = false;
uint32_t last_gps_fix_time = 0;

// LIS3DH buffer per ML
#define N_ACC_SAMPLES 300
float x_axis[N_ACC_SAMPLES], y_axis[N_ACC_SAMPLES], z_axis[N_ACC_SAMPLES];

// Timer
uint32_t t_lastSensors = 0;
uint32_t t_lastML      = 0;
uint32_t t_lastAgg     = 0;
uint32_t t_lastTx      = 0;
uint32_t t_lastGPS     = 0;

// Lora Keys
uint8_t devEUI[8]  = {0};
uint8_t appEUI[8]  = {0};
uint8_t appKEY[16] = {0};
uint8_t nwkKEY[16] = {0};

// ======================================================================================
// SETUP
// ======================================================================================
void setup() 
{
  // Avvia la libreria
  LoRaWAN.begin(false);   // false = no verbose debug
    
  Serial.begin(LOG_BAUDRATE);
//  while(!Serial) {};
  
  Serial.println("\n=== CICERONE Telemetry System ===");

  // --- GPS UART ---
  GPS.begin(GPS_BAUDRATE);
  Serial.println("[OK] GPS module initialized on USART1");

  // --- LIS3DH ---
  if (!lis.begin(0x18)) {
    Serial.println("[ERR] LIS3DH not found!");
  } else {
    lis.setRange(LIS3DH_RANGE_8_G);
    lis.setDataRate(LIS3DH_DATARATE_25_HZ);
    Serial.println("[OK] LIS3DH initialized");
  }

  // --- BME280 ---
  if (!bme.begin(0x77, &WireBME280)) {
    Serial.println("[ERR] BME280 not found!");
  } else Serial.println("[OK] BME280 initialized");

  // --- MLX90614 ---
  if (!mlx.begin(0x5A, &WireMLX)) {
    Serial.println("[ERR] MLX90614 not found!");
  } else Serial.println("[OK] MLX90614 initialized");

  // --- SD Card ---
  if (!SDManager_init()) {
    Serial.println("[WARN] SD non disponibile – dati non verranno salvati!");
  } else {
    Serial.println("[OK] SD pronta.");
  }

  // --- LoRaWAN Keys ---
  if (!LoRaWANManager_loadKeysFromSD()) {
    Serial.println("[WARN] LoRaWAN keys non caricate – trasmissione disabilitata!");
  } else {
    Serial.println("[OK] LoRaWAN keys pronte.");
  }

  // --- LoRaWAN Transmitter ---
  if (!LoRaWANTransmitter_init()) {
    Serial.println("[WARN] Join fallito – ritentare al prossimo ciclo.");
  } else {
    Serial.println("[OK] LoRaWAN join riuscito.");
  }

  // --- ML Modules ---
  RuminationModule_init();
  MotionModule_init();

  Serial.println("[System] Setup complete.\n");
}

// ======================================================================================
// LOOP
// ======================================================================================

void loop(){
  uint32_t now = millis();

  if (now - t_lastSensors >= 60000UL) {
    t_lastSensors = now;
    readAllSensors();
  }

  if (now - t_lastGPS >= 60000UL) {
    t_lastGPS = now;
    readGPSData();
  }

  if (now - t_lastML >= 30000UL) {
    t_lastML = now;
    performML();
  }

  if (now - t_lastAgg >= 300000UL) {
    t_lastAgg = now;
    aggregateTelemetryRecord();
  }

  if (now - t_lastTx >= 600000UL) {
    t_lastTx = now;
    Serial.println("FAKE Transmitting...Done");
    delay(50);
    transmitTelemetry();
  }

  while (GPS.available() > 0) {
    gps.encode(GPS.read());
  }

  Task_ECG();
  delay(50);
}

// ======================================================================================
// FUNZIONI
// ======================================================================================

void readAllSensors() {
  float temp_env = bme.readTemperature();
  float press = bme.readPressure() / 100.0F;
  float hum = bme.readHumidity();
  float temp_body = mlx.readObjectTempC();

  uint16_t raw = analogRead(PIN_VBAT_SENSE);
  float v_meas = (raw / 4095.0f) * 3.3f;
  const float VBAT_DIVIDER = 3.13f;
  vbat = v_meas * VBAT_DIVIDER;

  record.temp_body = temp_body;
  record.temp_ambient = temp_env;
  record.batt_voltage = vbat;

  Serial.print("[Sensors] Body=");
  Serial.print(temp_body, 2);
  Serial.print("°C  Env=");
  Serial.print(temp_env, 2);
  Serial.print("°C  Hum=");
  Serial.print(hum, 1);
  Serial.print("%  Press=");
  Serial.print(press, 1);
  Serial.print(" hPa  Batt=");
  Serial.print(vbat, 2);
  Serial.println(" V");
}

void readGPSData() {
  if (gps.location.isUpdated()) {
    gps_fix = true;
    record.latitude  = gps.location.lat();
    record.longitude = gps.location.lng();
    last_gps_fix_time = millis();
    Serial.print("[GPS] Fix OK  Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("  Lon: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("  Alt: ");
    Serial.print(gps.altitude.meters(), 1);
    Serial.print(" m  Sats: ");
    Serial.print(gps.satellites.value());
    Serial.print("  HDOP: ");
    Serial.println(gps.hdop.hdop(), 2);
  } else if (gps_fix && (millis() - last_gps_fix_time > 300000)) {
    gps_fix = false;
    Serial.println("[GPS] Lost fix");
  } else {
    Serial.println("[GPS] Waiting for fix...");
  }
}

void performML() {
  // Campionamento LIS3DH per dataset ML
  for (uint16_t i = 0; i < N_ACC_SAMPLES; i++) {
    lis.read();
    x_axis[i] = lis.x;
    y_axis[i] = lis.y;
    z_axis[i] = lis.z;
  }

  // Stub: valori ML fittizi
  ruminate_state = RuminationModule_run(NULL, 0);
  MotionModule_run(x_axis, y_axis, z_axis, N_ACC_SAMPLES,
                    grazing_state, sitting_state, standing_state, walking_state);
}

void aggregateTelemetryRecord() 
{
  record.timestamp = millis() / 1000;
  record.ruminate = ruminate_state;
  record.grazing  = grazing_state;
  record.sitting  = sitting_state;
  record.standing = standing_state;
  record.walking  = walking_state;

  // Aggiorna buffer ECG (record.heart[]) già gestito da Task_ECG()

  const char* json = serializeTelemetryJSON(&record);
  Serial.println("[Aggregator] JSON record ready:");
  Serial.println(json);

  // STORAGE su SD (L5)
  if (SDManager_isReady()) {
      SDManager_appendRecord(json);
  } else {
      Serial.println("[SD] Skip – SD non pronta.");
  }
}

void transmitTelemetry() {
    Serial.println("[LoRaTx] Invio record telemetrico...");
    const char* json = serializeTelemetryJSON(&record);

    if (!LoRaWANTransmitter_isJoined()) {
        Serial.println("[LoRaTx] Non joinato – skip invio.");
        return;
    }

    if (!LoRaWANTransmitter_send(json)) {
        Serial.println("[LoRaTx] Invio fallito – salvo su SD.");
        if (SDManager_isReady()) SDManager_appendRecord(json);
    }
}

#define ECG_SAMPLING_HZ    100
#define ECG_WINDOW_MS      5000
#define ECG_THRESHOLD_MV   200   // soglia R-wave (regolabile)
#define ECG_SAMPLES        (ECG_SAMPLING_HZ * (ECG_WINDOW_MS / 1000))

uint16_t ecg_buffer[ECG_SAMPLES];
float current_bpm = 0.0f;

void Task_ECG() {
  static uint32_t sample_interval_us = 1000000UL / ECG_SAMPLING_HZ;
  static uint32_t last_sample_us = micros();

  // Campionamento periodico
  if (micros() - last_sample_us >= sample_interval_us) {
    last_sample_us += sample_interval_us;
    uint16_t raw = analogRead(PIN_AD8232_OUTPUT);

    static uint16_t idx = 0;
    ecg_buffer[idx++] = raw;

    // Dopo 5 s di finestra (500 campioni a 100 Hz)
    if (idx >= ECG_SAMPLES) {
      idx = 0;
      calculateBPM();
    }
  }
}

//
// Calcola BPM medio su finestra di 5 s e aggiorna record.heart_bpm
//
void calculateBPM() {
  uint16_t peakCount = 0;
  bool above = false;
  const float vref = 3.3f;
  const float adc_mv = vref / 4095.0f * 1000.0f;

  for (int i = 0; i < ECG_SAMPLES; i++) {
    float mv = ecg_buffer[i] * adc_mv;
    if (!above && mv > ECG_THRESHOLD_MV) {
      peakCount++;
      above = true;
    } else if (mv < (ECG_THRESHOLD_MV * 0.5f)) {
      above = false;
    }
  }

  current_bpm = peakCount * (60.0f / (ECG_WINDOW_MS / 1000.0f));
  if (current_bpm < 40 || current_bpm > 180) current_bpm = NAN;

  Serial.print("[ECG] PeakCount=");
  Serial.print(peakCount);
  Serial.print("  BPM=");
  Serial.println(current_bpm, 1);

  static float last_bpm_values[5] = {0};
  static uint8_t ptr = 0;
  last_bpm_values[ptr++] = current_bpm;
  if (ptr >= 5) ptr = 0;

  float sum = 0; int count = 0;
  for (int i = 0; i < 5; i++) {
    if (last_bpm_values[i] > 0) { sum += last_bpm_values[i]; count++; }
  }
  float avg = (count > 0) ? sum / count : 0;

  record.heart_bpm = avg;

  Serial.print("[ECG] BPM medio finestra = ");
  Serial.println(record.heart_bpm, 1);
}
