#pragma once
#include <Arduino.h>

struct NMEAData {
  bool fix_valid = false;     // true se RMC indica fix (A)
  bool gga_valid = false;     // true se GGA aveva fix valido
  int32_t lat_e6 = 0;         // gradi * 1e6 (positivo N)
  int32_t lon_e6 = 0;         // gradi * 1e6 (positivo E)
  int32_t alt_dm = INT32_MIN; // altitudine MSL in decimetri; INT32_MIN = non valida
  uint8_t sats = 0;           // numero satelliti da GGA
  uint32_t time_utc = 0;      // hhmmss (es. 134512 = 13:45:12)
};

class NMEASlim {
public:
  void begin(Stream* s);
  void poll();                 // chiamare spesso (loop)
  const NMEAData& data() const { return _d; }

private:
  Stream* _s = nullptr;
  char _line[96];              // NMEA tipicamente < 82 char
  uint8_t _idx = 0;
  NMEAData _d;

  void handleLine(char* line, uint8_t len);
  bool checkChecksum(const char* start, const char* star, const char* hex);
  void parseRMC(char** tok, int ntok);
  void parseGGA(char** tok, int ntok);

  static int splitCSV(char* s, char** out, int max);
  static bool parseHhMmSs(const char* f, uint32_t& hhmmss);
  static bool parseLat_e6(const char* f, const char hemi, int32_t& lat_e6);
  static bool parseLon_e6(const char* f, const char hemi, int32_t& lon_e6);
  static bool parseAlt_dm(const char* f, int32_t& alt_dm);
};
