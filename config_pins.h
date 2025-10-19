/******************************************************************************************
Complete Hardware Pin Mapping
******************************************************************************************/

/*
| Modulo         | Segnale      | MCU Pin  | Note (Uso)                                  |
|----------------|--------------|----------|---------------------------------------------|
| LIS3DH         | SCL          | PB8      | I²C1 SCL – accelerometro                    |
|                | SDA          | PB9      | I²C1 SDA – accelerometro                    |
|----------------|--------------|----------|---------------------------------------------|
| BME280         | SCL          | PA12     | I²C2 SCL – sensore temperatura/pressione    |
|                | SDA          | PA15     | I²C2 SDA – sensore temperatura/pressione    |
|----------------|--------------|----------|---------------------------------------------|
| MLX90614       | SCL          | PB13     | I²C3 SCL – termometro IR ambiente/oggetto   |
|                | SDA          | PB14     | I²C3 SDA – termometro IR                    |
|----------------|--------------|----------|---------------------------------------------|
| AD8232         | LO-          | PA13     | ECG – lead-off negativo                     |
|                | LO+          | PA14     | ECG – lead-off positivo                     |
|                | OUTPUT       | PB3      | ECG – segnale amplificato                   |
|----------------|--------------|----------|---------------------------------------------|
| MAX9814        | OUT          | PB1      | Microfono – uscita analogica audio RMS      |
|----------------|--------------|----------|---------------------------------------------|
| SD CARD        | CS           | PA4      | SPI1_NSS – chip select SD                   |
|                | SCK          | PA5      | SPI1_SCK – clock                            |
|                | MISO         | PA6      | SPI1_MISO (DO) – dati dalla SD              |
|                | MOSI         | PA7      | SPI1_MOSI (DI) – dati verso SD              |
|----------------|--------------|----------|---------------------------------------------|
| LoRa SX1262    | NSS          | PB12     | SPI2_NSS – chip select radio                |
|                | SCK          | PB10     | SPI2_SCK – clock                            |
|                | MISO         | PB11     | SPI2_MISO – dati dalla radio                |
|                | MOSI         | PB15     | SPI2_MOSI – dati verso radio                |
|                | DIO1         | PC6      | interrupt radio                             |
|                | BUSY         | PC7      | stato radio busy                            |
|                | RESET        | PC8      | reset modulo radio                          |
|----------------|--------------|----------|---------------------------------------------|
| GPS (Move-X)   | TX           | PA9      | USART1_TX – trasmissione NMEA               |
|                | RX           | PA10     | USART1_RX – ricezione NMEA                  |
|----------------|--------------|----------|---------------------------------------------|
| Battery Monitor| VBAT_SENSE   | PC0      | Lettura tensione batteria (sensore virtuale)|
|----------------|--------------|----------|---------------------------------------------|
| Status LED     | LED          | PB0      | LED di stato (blinking / error)             |
|----------------|--------------|----------|---------------------------------------------|
*/

// === LIS3DH Accelerometer (I2C1) ===
#define PIN_LIS3DH_SCL        PB8
#define PIN_LIS3DH_SDA        PB9

// === BME280 Temperature/Pressure/Humidity (I2C2) ===
#define PIN_BME280_SCL        PA12
#define PIN_BME280_SDA        PA15

// === MLX90614 Infrared Thermometer (I2C3) ===
#define PIN_MLX90614_SCL      PB13
#define PIN_MLX90614_SDA      PB14

// === AD8232 ECG Module ===
#define PIN_AD8232_LO_MINUS   PA13
#define PIN_AD8232_LO_PLUS    PA14
#define PIN_AD8232_OUTPUT     PB3

// === MAX9814 Microphone (Analog OUT) ===
#define PIN_MAX9814_OUT       PB1

// === SD CARD (SPI1) ===
#define PIN_SD_CS             PA4
#define PIN_SD_SCK            PA5
#define PIN_SD_MISO           PA6
#define PIN_SD_MOSI           PA7

// === LoRa SX1262 Radio (SPI2) ===
#define PIN_LORA_NSS          PB12
#define PIN_LORA_SCK          PB10
#define PIN_LORA_MISO         PB11
#define PIN_LORA_MOSI         PB15
#define PIN_LORA_DIO1         PC6
#define PIN_LORA_BUSY         PC7
#define PIN_LORA_RESET        PC8

// === GPS (Move-X Integrated, USART1) ===
#define PIN_GPS_TX            PA9
#define PIN_GPS_RX            PA10

// === Battery Monitor (Virtual Sensor) ===
#define PIN_VBAT_SENSE        PC0