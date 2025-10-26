/******************************************************************************************
 *  collars.ino – Cicerone (Move-X) unified firmware
 *  One-file build, non-blocking, LoRaWAN-first init to avoid stalls.
 *  Uses: Adafruit_LIS3DH, Adafruit_BME280, Adafruit_MLX90614, SdFat, TinyGPS++, LibLoRaWAN
 *
 *  Pins & buses: see config_pins.h; Timings: see config_timers.h; Features: see config_sensors.h
 ******************************************************************************************/

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <LibLoRaWAN.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MLX90614.h>

#include "config_pins.h"
#include "config_sensors.h"
#include "config_timers.h"
#include "data_structs.h"

static inline int32_t fx100(float v)  { if (!isfinite(v)) return 0; return (int32_t)lrintf(v * 100.0f); }
static inline int32_t fx10(float v)   { if (!isfinite(v)) return 0; return (int32_t)lrintf(v * 10.0f); }
static inline int32_t fx1e6(float v)  { if (!isfinite(v)) return 0; return (int32_t)lrintf(v * 1000000.0f); }

static bool have_agg = false;

// Forward declaration (usata in readAllSensors prima della definizione)
static void updateAccelFeaturesFromSample_mg(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);

// Variabili BME che usi nel packer
static float bme_pressure_hpa = 0.0f;
static float bme_altitude_m   = 0.0f;

// Fallback per RAD_TO_DEG se non definito (di solito c'è già via Arduino.h)
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f/PI)
#endif

#define ENABLE_HB 0

static void logLrwErr(const char* where, int code) {
#if ENABLE_LOG_SERIAL
  Serial.print(where); Serial.print(" -> err "); Serial.println(code);
#endif
}

static volatile bool   g_tx_req = false;
static uint8_t         g_tx_buf[64];
static size_t          g_tx_len = 0;
static const uint8_t   TELEMETRY_FPORT = 2; // distinto dall’HB (1)
static uint32_t        t_lastAnyTx = 0;

// ======================================================================================
// === I2C buses (as in your test sketches) =============================================
// ======================================================================================
TwoWire WireLIS(PIN_LIS3DH_SDA, PIN_LIS3DH_SCL);    // I2C1 LIS3DH
TwoWire WireBME(PIN_BME280_SDA, PIN_BME280_SCL);    // I2C2 BME280
TwoWire WireMLX(PIN_MLX90614_SDA, PIN_MLX90614_SCL);// I2C3 MLX90614

// ======================================================================================

#if ENABLE_LIS3DH
Adafruit_LIS3DH lis = Adafruit_LIS3DH(&WireLIS);
#endif
#if ENABLE_BME280
Adafruit_BME280 bme;
#endif
#if ENABLE_MLX90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#endif

#ifndef BME_SEA_LEVEL_HPA
#define BME_SEA_LEVEL_HPA 1013.25f
#endif

// ECG (AD8232) and MIC (MAX9814)
#define ECG_PIN      PIN_AD8232_OUTPUT
#define ECG_LO_MINUS PIN_AD8232_LO_MINUS
#define ECG_LO_PLUS  PIN_AD8232_LO_PLUS
#define MIC_PIN      PIN_MAX9814_OUT

// Battery monitor (ADC)
#define VBAT_ADC_PIN PIN_VBAT_SENSE
static const float VBAT_DIVIDER = 2.0f;   

// SD
SdFat sd;
static const char* LORAWAN_CONFIG_FILE = "lorawan.cfg";

// LoRaWAN
#define LORA_PORT 1
static bool net_joined = false;
static unsigned long lastLoraTx = 0;

// LoRa keys buffers loaded from SD
uint8_t devEUI[8]  = {0};
uint8_t appEUI[8]  = {0};
uint8_t appKEY[16] = {0};
uint8_t nwkKEY[16] = {0};

// GPS
HardwareSerial& GPS = Serial1;  // USART1: PA9/PA10
TinyGPSPlus gps;

// Telemetry buffer
static telemetryRecord_t telem;

static float mlx_ta_c = 0.0f;   // ambient MLX

// Timing
static uint32_t t_lastSensors = 0;
static uint32_t t_lastGPS     = 0;
static uint32_t t_lastML      = 0;
static uint32_t t_lastAgg     = 0;
static uint32_t t_lastTx      = 0;

// === Accel advanced features toggles (easy rollback) ===
#define ENABLE_ACC_STD     1  // deviazione standard per asse (u16 x3)
#define ENABLE_SMA         1  // Signal Magnitude Area dinamica (u16)
#define ENABLE_CHEW_FREQ   1  // frequenza masticazione stimata (u8)

// Finestra per feature accelerometriche (campioni)
#define ACC_WIN            64 // ~0.64s @100 Hz

// === Accel window state (mg) ===
static int16_t  acc_x_buf[ACC_WIN], acc_y_buf[ACC_WIN], acc_z_buf[ACC_WIN];
static uint16_t acc_idx = 0, acc_count = 0;
static int32_t  sum_x = 0, sum_y = 0, sum_z = 0;          // somma mg
static int64_t  sumxx = 0, sumyy = 0, sumzz = 0;          // somma mg^2
static uint16_t a_rms_mg = 0;                              // già nel Set MIN
static uint16_t std_x_mg = 0, std_y_mg = 0, std_z_mg = 0;  // extra
static uint16_t sma_mg = 0;                                // extra
static uint8_t  chew_f_0p1Hz = 0;                          // extra
static uint8_t  pitch_deg = 0, roll_deg = 0;               // Set MIN
static uint8_t  accel_flags = 0;                           // Set MIN (INT non abilitati -> 0)

// ======================================================================================
// === Utilities: HEX parser & key print (from your testSDCardConfig) ===================
// ======================================================================================
static void parseHex(const char* str, uint8_t* dest, int len) {
  int j = 0;
  for (int i = 0; str[i] != '\0' && j < len; i++) {
    if (isxdigit(str[i]) && isxdigit(str[i+1])) {
      char b[3] = { str[i], str[i+1], 0 };
      dest[j++] = (uint8_t)strtoul(b, NULL, 16);
      i++;
    }
  }
  while (j < len) dest[j++] = 0x00;
}
static void printKey(const char* label, const uint8_t* key, int len) {
#if ENABLE_LOG_SERIAL
  Serial.print(label); Serial.print(" = { ");
  for (int i = 0; i < len; i++) { Serial.printf("0x%02X%s", key[i], (i<len-1)?", ":""); }
  Serial.println(" }");
#endif
}

static uint8_t make_status() {
  uint8_t s = 0;
#if ENABLE_GPS
  if (gps.location.isValid()) s |= 1<<0; // GPS fix
#endif
#if ENABLE_BME280
  s |= 1<<1;
#endif
#if ENABLE_MLX90614
  s |= 1<<2;
#endif
#if ENABLE_LIS3DH
  s |= 1<<3;
#endif
#if ENABLE_AD8232
  s |= 1<<4;
#endif
#if ENABLE_SD_STORAGE
  s |= 1<<5;
#endif
#if ENABLE_BATTERY_MONITOR
  if (telem.batt_voltage > 0.1f) s |= 1<<6;
#endif
  return s;
}


static inline void wr_u16(uint8_t* p, uint16_t v){ p[0]=v&0xFF; p[1]=(v>>8)&0xFF; }
static inline void wr_i16(uint8_t* p, int16_t v){ wr_u16(p,(uint16_t)v); }
static inline void wr_u32(uint8_t* p, uint32_t v){ p[0]=v&0xFF; p[1]=(v>>8)&0xFF; p[2]=(v>>16)&0xFF; p[3]=(v>>24)&0xFF; }
static inline void wr_i32(uint8_t* p, int32_t v){ wr_u32(p,(uint32_t)v); }

static size_t buildTelemetryBinary(uint8_t* out) {
  int32_t lat_e6 = fx1e6(telem.latitude);
  int32_t lon_e6 = fx1e6(telem.longitude);
  int32_t tb_100 = fx100(telem.temp_body);
  int32_t ta_100 = fx100(telem.temp_ambient);
  int32_t h_10   = fx10(telem.humidity);
  int32_t vb_100 = fx100(telem.batt_voltage);
  uint32_t t     = telem.timestamp;
  int      bpm   = (int)lrintf(telem.heart_bpm);

  // NEW: pressione e altitudine (scalate)
  uint16_t press_dh = (uint16_t)constrain((int)lrintf(bme_pressure_hpa * 10.0f), 0, 65535);
  int16_t  alt_m    = (int16_t)constrain((int)lrintf(bme_altitude_m), -32768, 32767);

  uint8_t flags = (telem.ruminate ? 1:0)
                | (telem.grazing  ? 2:0)
                | (telem.standing ? 4:0)
                | (telem.walking  ? 8:0)
                | (telem.sitting  ?16:0);

  // Layout (little-endian) – 26 byte totali:
  // [0..3]   t (u32, s)
  // [4..7]   lat_e6 (i32)
  // [8..11]  lon_e6 (i32)
  // [12..13] tb_100 (i16)
  // [14..15] ta_100 (i16)
  // [16..17] h_10   (i16)
  // [18]     bpm    (u8)
  // [19]     flags  (u8)
  // [20..21] vb_100 (u16)
  // [22..23] press_dh (u16)     
  // [24..25] alt_m    (i16)     

  wr_u32 (&out[0],  t);
  wr_i32 (&out[4],  lat_e6);
  wr_i32 (&out[8],  lon_e6);
  wr_i16 (&out[12], (int16_t)tb_100);
  wr_i16 (&out[14], (int16_t)ta_100);
  wr_i16 (&out[16], (int16_t)h_10);
  out[18] = (uint8_t)constrain(bpm, 0, 250);
  out[19] = flags;
  wr_u16 (&out[20], (uint16_t)constrain(vb_100, 0, 65535));
  wr_u16 (&out[22], press_dh);           
  wr_i16 (&out[24], alt_m);              
// === Set MIN LIS3DH ===
  int16_t ax_mg = (int16_t)(sum_x / (acc_count?acc_count:1)); // media corrente per telemetria
  int16_t ay_mg = (int16_t)(sum_y / (acc_count?acc_count:1));
  int16_t az_mg = (int16_t)(sum_z / (acc_count?acc_count:1));

  wr_i16 (&out[26], ax_mg);
  wr_i16 (&out[28], ay_mg);
  wr_i16 (&out[30], az_mg);
  wr_u16 (&out[32], a_rms_mg);
  out[34] = pitch_deg;
  out[35] = roll_deg;
  out[36] = accel_flags;

  // === Extra compatti (status + dTa) ===
  uint8_t status = make_status();
  int8_t  dTa_mlx_bme_dec = (int8_t)constrain((int)lrintf((mlx_ta_c - telem.temp_ambient)*10.0f), -128, 127);
  out[37] = status;
  out[38] = (uint8_t)dTa_mlx_bme_dec;

  // === EXTRA “dopo ADR” (abilitati ora, ma disattivabili con macro) ===
  size_t pos = 39;

#if ENABLE_ACC_STD
  wr_u16 (&out[pos+0], std_x_mg);
  wr_u16 (&out[pos+2], std_y_mg);
  wr_u16 (&out[pos+4], std_z_mg);
  pos += 6;
#endif

#if ENABLE_SMA
  wr_u16 (&out[pos], sma_mg);
  pos += 2;
#endif

#if ENABLE_CHEW_FREQ
  out[pos] = chew_f_0p1Hz;
  pos += 1;
#endif

  return pos; // lunghezza totale
}

// ======================================================================================
// === Load LoRa keys from SD (same logic as your test) =================================
// ======================================================================================
static bool loadKeysFromSD() {
  File cfg = sd.open(LORAWAN_CONFIG_FILE, FILE_READ);
  if (!cfg) {
#if ENABLE_LOG_SERIAL
    Serial.println("[ERR] lorawan.cfg not found!");
#endif
    return false;
  }
  char line[96];
  while (cfg.available()) {
    int len = cfg.readBytesUntil('\n', line, sizeof(line)-1);
    line[len] = '\0';
    if (line[0]=='#' || line[0]=='\r' || line[0]=='\n' || line[0]=='\0') continue;
    char* cr = strchr(line, '\r'); if (cr) *cr = '\0';

    if      (strncmp(line,"devEUI=",7)==0) parseHex(line+7, devEUI, 8);
    else if (strncmp(line,"appEUI=",7)==0) parseHex(line+7, appEUI, 8);
    else if (strncmp(line,"appKEY=",7)==0) parseHex(line+7, appKEY,16);
    else if (strncmp(line,"nwkKEY=",7)==0) parseHex(line+7, nwkKEY,16);
  }
  cfg.close();

#if ENABLE_LOG_SERIAL
  Serial.println("[OK] LoRaWAN keys loaded from SD:");
  printKey("devEUI", devEUI, 8);
  printKey("appEUI", appEUI, 8);
  printKey("appKEY", appKEY,16);
  printKey("nwkKEY", nwkKEY,16);
#endif
  return true;
}

// ======================================================================================
// === Sensor helpers ===================================================================
// ======================================================================================
static float readVBat() {
  uint16_t raw = analogRead(VBAT_ADC_PIN);
  float v = (raw * (ADC_VREF_VOLTAGE / (float)((1<<ADC_RESOLUTION_BITS)-1))) * VBAT_DIVIDER;
  return v;
}

static void initSensors() {
#if ENABLE_LIS3DH
  WireLIS.begin();
  if (!lis.begin(0x18)) {
#if ENABLE_LOG_SERIAL
    Serial.println("[LIS3DH] not found!");
#endif
  } else {
    lis.setRange(LIS3DH_RANGE_2_G);
    lis.setDataRate(LIS3DH_DATARATE_100_HZ);
#if ENABLE_LOG_SERIAL
    Serial.println("[LIS3DH] OK");
#endif
  }
#endif

#if ENABLE_BME280
  WireBME.begin();
  if (!bme.begin(0x77, &WireBME)) {
#if ENABLE_LOG_SERIAL
    Serial.println("[BME280] not found!");
#endif
  } else {
#if ENABLE_LOG_SERIAL
    Serial.println("[BME280] OK");
#endif
  }
#endif

#if ENABLE_MLX90614
  WireMLX.begin();
  if (!mlx.begin(0x5A, &WireMLX)) {
#if ENABLE_LOG_SERIAL
    Serial.println("[MLX90614] not found!");
#endif
  } else {
#if ENABLE_LOG_SERIAL
    Serial.println("[MLX90614] OK");
#endif
  }
#endif

#if ENABLE_MAX9814
  analogReadResolution(ADC_RESOLUTION_BITS);
#endif

#if ENABLE_AD8232
  pinMode(ECG_LO_MINUS, INPUT_PULLUP);
  pinMode(ECG_LO_PLUS,  INPUT_PULLUP);
  pinMode(ECG_PIN, INPUT_ANALOG);
#endif
}

// ======================================================================================
// === Sensor reads (fast, non-blocking snapshots) ======================================
// ======================================================================================
static void readAllSensors() 
{
#if ENABLE_LIS3DH
  sensors_event_t ev;
  lis.getEvent(&ev);
  // Converti da m/s^2 a mg (1 g ≈ 9.80665 m/s^2; 1 g = 1000 mg)
  const float MS2_TO_MG = 1000.0f / 9.80665f;
  int16_t ax_mg = (int16_t)constrain((int)lrintf(ev.acceleration.x * MS2_TO_MG), -32768, 32767);
  int16_t ay_mg = (int16_t)constrain((int)lrintf(ev.acceleration.y * MS2_TO_MG), -32768, 32767);
  int16_t az_mg = (int16_t)constrain((int)lrintf(ev.acceleration.z * MS2_TO_MG), -32768, 32767);
  updateAccelFeaturesFromSample_mg(ax_mg, ay_mg, az_mg);
#endif

#if ENABLE_BME280
  telem.temp_ambient = bme.readTemperature();
  telem.humidity     = bme.readHumidity();
  float p_pa = bme.readPressure();
  if (isfinite(p_pa)) bme_pressure_hpa = p_pa / 100.0f;
  float alt = bme.readAltitude(BME_SEA_LEVEL_HPA);
  if (isfinite(alt)) bme_altitude_m = alt;
#endif

#if ENABLE_MLX90614
  telem.temp_body = mlx.readObjectTempC();
  float ta = mlx.readAmbientTempC();
  if (isfinite(ta)) mlx_ta_c = ta;
#endif

#if ENABLE_BATTERY_MONITOR
  telem.batt_voltage = readVBat();
#endif
}

static void updateAccelFeaturesFromSample_mg(int16_t ax_mg, int16_t ay_mg, int16_t az_mg) {
  // Rimuovi contributo del campione uscente (se buffer pieno)
  if (acc_count == ACC_WIN) {
    int16_t ox = acc_x_buf[acc_idx];
    int16_t oy = acc_y_buf[acc_idx];
    int16_t oz = acc_z_buf[acc_idx];
    sum_x  -= ox;  sum_y  -= oy;  sum_z  -= oz;
    sumxx  -= (int32_t)ox*ox;  sumyy -= (int32_t)oy*oy;  sumzz -= (int32_t)oz*oz;
    // per SMA dinamica: sottrazione sarà gestita ricalcolando con la nuova media
  } else {
    acc_count++;
  }

  // Inserisci nuovo campione in ring
  acc_x_buf[acc_idx] = ax_mg;
  acc_y_buf[acc_idx] = ay_mg;
  acc_z_buf[acc_idx] = az_mg;
  acc_idx = (acc_idx + 1) % ACC_WIN;

  sum_x += ax_mg; sum_y += ay_mg; sum_z += az_mg;
  sumxx += (int32_t)ax_mg*ax_mg; sumyy += (int32_t)ay_mg*ay_mg; sumzz += (int32_t)az_mg*az_mg;

  // Mean per asse (mg)
  float n = (float)acc_count;
  float mx = sum_x / n, my = sum_y / n, mz = sum_z / n;

  // Deviazioni standard (mg)
#if ENABLE_ACC_STD
  float vx = (sumxx / n) - (mx*mx);
  float vy = (sumyy / n) - (my*my);
  float vz = (sumzz / n) - (mz*mz);
  vx = (vx < 0) ? 0 : vx; vy = (vy < 0) ? 0 : vy; vz = (vz < 0) ? 0 : vz;
  std_x_mg = (uint16_t)constrain((int)lrintf(sqrtf(vx)), 0, 65535);
  std_y_mg = (uint16_t)constrain((int)lrintf(sqrtf(vy)), 0, 65535);
  std_z_mg = (uint16_t)constrain((int)lrintf(sqrtf(vz)), 0, 65535);
#endif

  // RMS (mg) del modulo (mean(|a|^2))^{1/2}
  float rms = sqrtf( ( (float)ax_mg*ax_mg + (float)ay_mg*ay_mg + (float)az_mg*az_mg ) / 3.0f );
  a_rms_mg = (uint16_t)constrain((int)lrintf(rms), 0, 65535);

  // SMA dinamica: media di |a - mean|
#if ENABLE_SMA
  // ricalcolo veloce su sottoinsieme (compromesso) – per robustezza potremmo iterare l'intero buffer
  float dx = fabsf(ax_mg - mx), dy = fabsf(ay_mg - my), dz = fabsf(az_mg - mz);
  float sma_inst = (dx + dy + dz);            // mg
  // IIR semplice per stabilizzare
  static float sma_f = 0.f;
  sma_f = 0.9f*sma_f + 0.1f*sma_inst;
  sma_mg = (uint16_t)constrain((int)lrintf(sma_f), 0, 65535);
#endif

  // Pitch/Roll (°) da media (orientamento quasi-statico)
  // pitch = atan2(-mx, sqrt(my^2 + mz^2)); roll = atan2(my, mz);
  float pitch = RAD_TO_DEG * atan2f(-mx, sqrtf(my*my + mz*mz));
  float roll  = RAD_TO_DEG * atan2f( my, mz);
  pitch = fminf(fmaxf(pitch, -90.f), 90.f);   // clamp
  roll  = fminf(fmaxf(roll,  -90.f), 90.f);
  pitch_deg = (uint8_t)constrain((int)lrintf(fabsf(pitch)), 0, 180); // coarse 0..180
  roll_deg  = (uint8_t)constrain((int)lrintf(fabsf(roll)),  0, 180);

  // Chewing freq (0.1 Hz) – peak detect su componente dinamica (grezza)
#if ENABLE_CHEW_FREQ
  // Usa soglie relative alla std media
  float dyn_metric = sma_mg;                  // proxy dinamico
  static bool above=false; static uint32_t lastPeak=0;
  const float TH_H = 40.0f;   // mg (tune)
  const float TH_L = 20.0f;   // mg (tune)
  uint32_t now = millis();
  if (!above && dyn_metric > TH_H) {
    above = true;
    uint32_t dt = now - lastPeak;
    if (dt > 300 && dt < 5000) {              // 0.2..3.3 Hz (range chewing)
      float f_hz = 1000.0f / (float)dt;
      chew_f_0p1Hz = (uint8_t)constrain((int)lrintf(f_hz*10.0f), 0, 255);
    }
    lastPeak = now;
  }
  if (above && dyn_metric < TH_L) above = false;
#endif

  // accel_flags rimane 0 finché non abiliti INT (tap/freefall ecc.)
}

// ======================================================================================
// === GPS ==============================================================================
static void readGPSData() {
#if ENABLE_GPS
  // Nothing blocking here; background decode is in the main loop while-available
  if (gps.location.isValid()) {
    telem.latitude  = gps.location.lat();
    telem.longitude = gps.location.lng();
  }
#endif
}

// ======================================================================================
// === AD8232 ECG task (inspired by your testAD8232) ====================================
#if ENABLE_AD8232
// Moving average & peak detect for BPM
#define ECG_FILTER_SIZE 8
static int   ecgSamples[ECG_FILTER_SIZE] = {0};
static int   ecgIndex = 0;
static long  ecgSum   = 0;
static bool  ecgAbove = false;
static unsigned long lastBeatMs = 0;

static void Task_ECG() 
{
  int raw = analogRead(ECG_PIN);
  ecgSum -= ecgSamples[ecgIndex];
  ecgSamples[ecgIndex] = raw;
  ecgSum += raw;
  ecgIndex = (ecgIndex + 1) % ECG_FILTER_SIZE;
  int filtered = ecgSum / ECG_FILTER_SIZE;

  int loM = digitalRead(ECG_LO_MINUS);
  int loP = digitalRead(ECG_LO_PLUS);
  if (loM == 1 || loP == 1) {
    telem.heart_bpm = 0;
    return;
  }

  const int THRESHOLD_HIGH = 520;
  const int THRESHOLD_LOW  = 480;
  const int MIN_INTERVAL   = 300;
  const int MAX_INTERVAL   = 2000;

  unsigned long now = millis();
  if (!ecgAbove && filtered > THRESHOLD_HIGH) {
    ecgAbove = true;
    unsigned long dt = now - lastBeatMs;
    if (dt > (unsigned)MIN_INTERVAL && dt < (unsigned)MAX_INTERVAL) {
      telem.heart_bpm = 60000.0f / (float)dt;
    }
    lastBeatMs = now;
  }
  if (ecgAbove && filtered < THRESHOLD_LOW) ecgAbove = false;
}
#else
static void Task_ECG() {}
#endif

// ======================================================================================
// === MAX9814 rough level for rumination proxy =========================================
#if ENABLE_MAX9814
static float soundSmooth = 0.0f;
static void updateMicLevel() {
  // quick tiny window
  const int SAMPLES = 60;
  long sum = 0;
  for (int i=0;i<SAMPLES;i++) sum += analogRead(MIC_PIN);
  float avg = (float)sum / SAMPLES;

  long devSum = 0;
  for (int i=0;i<SAMPLES;i++) devSum += abs((int)analogRead(MIC_PIN) - (int)avg);
  float level = (float)devSum / SAMPLES;
  soundSmooth = (1.0f - 0.1f)*soundSmooth + 0.1f*level;
}
#else
static void updateMicLevel() {}
#endif

// ======================================================================================
// === Simple ML placeholders (state from accel + audio) =================================
static void performML() {
  // place-holder rules; tune offline
  telem.ruminate = (soundSmooth > 60) ? 1 : 0;
  // simple motion flags can be set if LIS3DH variance computed; here keep zeros
  telem.grazing = 0; telem.standing = 0; telem.walking = 0; telem.sitting = 0;
}

// ======================================================================================
// === Telemetry aggregation & JSON serialization ========================================
static void aggregateTelemetryRecord() {
  telem.timestamp = (uint32_t)(millis()/1000); // replace with real epoch if GPS time used
  have_agg = true;
}

// ======================================================================================
// === LoRaWAN userloop (non-blocking) ===================================================
static void lora_userloop() {
  if (!net_joined) {
    if (LRW_OK == LoRaWAN.NetJoined()) {
      net_joined = true;
#if ENABLE_LOG_SERIAL
      Serial.println("[LoRaWAN] Joined.");
#endif
      lastLoraTx = millis();
    }
    return;
  }

  // Se c'è una richiesta di invio, fallo QUI (dove l'HB funzionava)
  if (g_tx_req) {
    int rc = LoRaWAN.Send(TELEMETRY_FPORT, LRW_UNCONFIRMED_MSG, (uint8_t*)g_tx_buf, g_tx_len);
#if ENABLE_LOG_SERIAL
    Serial.print("[TX userloop] len="); Serial.print((int)g_tx_len);
    Serial.print(" -> rc="); Serial.println(rc);
#endif
    if (rc == LRW_OK) {
      t_lastAnyTx = millis();
      g_tx_req = false;   // inviato, libera la richiesta
    }
    // Se rc != OK lasciamo g_tx_req = true, riproverà al giro successivo
  }

  // Heartbeat disabilitato
#if ENABLE_HB
  if (millis() - t_lastAnyTx >= (unsigned long)(FREQ_LORA_TX_SEC*1000UL)) {
    const char* hb = "{\"hb\":1}";
    LoRaWAN.Send(1, LRW_UNCONFIRMED_MSG, (uint8_t*)hb, strlen(hb));
    t_lastAnyTx = millis();
  }
#endif
}

// ======================================================================================
// === Transmit telemetry (non-blocking; no delay inside) ================================
static void transmitTelemetry() {
  if (!net_joined) return;

  size_t n = buildTelemetryBinary(g_tx_buf);
  g_tx_len = n;
  g_tx_req = true;

#if ENABLE_LOG_SERIAL
  Serial.print("[TX-enqueue] len="); Serial.println((int)n);
#endif
}

// ======================================================================================
// === Setup ============================================================================
// ======================================================================================
void setup() {

  LoRaWAN.attachLoop(lora_userloop);
  LoRaWAN.begin(false);                 

  Serial.begin(LOG_BAUDRATE);
  delay(400);
#if ENABLE_LOG_SERIAL
  Serial.println("\n=== CICERONE Telemetry System – unified build ===");
#endif

  // SD init
#if ENABLE_SD_STORAGE || ENABLE_LORA_TX
  pinMode(PIN_SD_CS, OUTPUT);
  if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(18))) {
#if ENABLE_LOG_SERIAL
    Serial.println("[SD] init failed!");
#endif
  } else {
#if ENABLE_LOG_SERIAL
    Serial.println("[SD] OK");
#endif
  }
#endif

  // Load LoRa keys from SD and Join
#if ENABLE_LORA_TX
  if (loadKeysFromSD()) {
    if (LRW_OK != LoRaWAN.setRegion(LRW_REGION_EU868)) Serial.println("setRegion() error!");
    if (LRW_OK != LoRaWAN.setDevEUI(devEUI))           Serial.println("setDevEUI() error!");
    if (LRW_OK != LoRaWAN.setJoinEUI(appEUI))          Serial.println("setJoinEUI() error!");
    if (LRW_OK != LoRaWAN.setAppKey(appKEY))           Serial.println("setAppKey() error!");
    if (LRW_OK != LoRaWAN.setNwkKey(nwkKEY))           Serial.println("setNwkKey() error!");
    if (LRW_OK != LoRaWAN.Join(LRW_JOIN_OTAA))         Serial.println("Join() error!");
  }
#endif

  // GPS
#if ENABLE_GPS
  GPS.begin(GPS_BAUDRATE);
#endif

  // Sensors
  initSensors();

  // ADC common
  analogReadResolution(ADC_RESOLUTION_BITS);

  // Timers
  uint32_t now = millis();
  t_lastSensors = t_lastGPS = t_lastML = t_lastAgg = t_lastTx = now;

#if ENABLE_LOG_SERIAL
  Serial.println("[System] Setup complete.");
#endif
}

// ======================================================================================
// === Main loop ========================================================================
// ======================================================================================
void loop() {
  // Must be called frequently for radio timing/state machine
  LoRaWAN.process();

  uint32_t now = millis();

  if (now - t_lastSensors >= SENSOR_READ_INTERVAL_MS) {
    t_lastSensors = now;
    readAllSensors();
  }

  if (now - t_lastGPS >= GPS_READ_INTERVAL_MS) {
    t_lastGPS = now;
    readGPSData();
  }

  if (now - t_lastML >= ML_EXEC_INTERVAL_MS) {
    t_lastML = now;
    updateMicLevel();
    performML();
  }

  if (now - t_lastAgg >= TELEMETRY_AGG_INTERVAL_MS) {
    t_lastAgg = now;
    aggregateTelemetryRecord();
  }

  if (have_agg && (now - t_lastTx >= TX_INTERVAL_MS)) {
    t_lastTx = now;
    transmitTelemetry();
  }

  // Background GPS NMEA parse
#if ENABLE_GPS
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
  }
#endif

  // Background ECG processing
  Task_ECG();

  // Keep the loop snappy but cooperative
  delay(LOOP_DELAY_MS);

  // Also call process() again after potential delays to keep MAC timing tight
  LoRaWAN.process();
}
