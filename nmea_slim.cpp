#include "nmea_slim.h"

// --- Helpers ---------------------------------------------------------------

static inline int hexVal(char c){
  if(c>='0'&&c<='9') return c-'0';
  if(c>='A'&&c<='F') return 10+c-'A';
  if(c>='a'&&c<='f') return 10+c-'a';
  return -1;
}

int NMEASlim::splitCSV(char* s, char** out, int max) {
  int n = 0;
  while (*s && n < max) {
    out[n++] = s;
    while (*s && *s != ',' && *s != '*') s++;
    if (*s == ',' ) { *s++ = '\\0'; continue; }
    if (*s == '*')  { *s = '\\0';  break; }
  }
  return n;
}

bool NMEASlim::checkChecksum(const char* start, const char* star, const char* hex) {
  uint8_t x = 0;
  for (const char* p = start; p < star; ++p) x ^= (uint8_t)(*p);
  int hi = hexVal(hex[0]), lo = hexVal(hex[1]);
  if (hi < 0 || lo < 0) return false;
  return x == (uint8_t)((hi<<4)|lo);
}

bool NMEASlim::parseHhMmSs(const char* f, uint32_t& hhmmss) {
  if (!f || !*f) return false;
  int h=0,m=0,s=0;
  int len=0; while(f[len] && f[len] != '.') len++;
  if (len < 6) return false;
  h = (f[0]-'0')*10 + (f[1]-'0');
  m = (f[2]-'0')*10 + (f[3]-'0');
  s = (f[4]-'0')*10 + (f[5]-'0');
  if (h<0||h>23||m<0||m>59||s<0||s>59) return false;
  hhmmss = (uint32_t)(h*10000 + m*100 + s);
  return true;
}

static bool parseDdMm_mmmm_to_e6(const char* f, bool isLon, int32_t& deg_e6) {
  if (!f || !*f) return false;
  int dot = -1; for (int i=0; f[i]; ++i){ if (f[i]=='.'){ dot=i; break; } }
  if (dot < 0) return false;
  int pre = dot;
  int dd_digits = isLon ? 3 : 2;
  if (pre < dd_digits + 2) return false;

  int deg = 0;
  for(int i=0;i<dd_digits;i++){ if(f[i]<'0'||f[i]>'9') return false; deg = deg*10 + (f[i]-'0'); }

  int mm_i = 0;
  if (f[dd_digits]<'0'||f[dd_digits]>'9' || f[dd_digits+1]<'0'||f[dd_digits+1]>'9') return false;
  mm_i = (f[dd_digits]-'0')*10 + (f[dd_digits+1]-'0');

  int frac = 0; int scale = 1;
  for (int i=dot+1; f[i]; ++i) {
    if (f[i]<'0'||f[i]>'9') break;
    frac = frac*10 + (f[i]-'0');
    scale *= 10;
  }

  int64_t deg_e6_ll = (int64_t)deg * 1000000LL;
  int64_t min_e6 = ((int64_t)mm_i * 1000000LL) / 60LL
                 + ((int64_t)frac * 1000000LL) / ((int64_t)scale * 60LL);

  deg_e6 = (int32_t)(deg_e6_ll + min_e6);
  return true;
}

bool NMEASlim::parseLat_e6(const char* f, const char hemi, int32_t& lat_e6) {
  int32_t v=0; if(!parseDdMm_mmmm_to_e6(f,false,v)) return false;
  if (hemi=='S') v = -v; else if (hemi!='N') return false;
  lat_e6 = v; return true;
}
bool NMEASlim::parseLon_e6(const char* f, const char hemi, int32_t& lon_e6) {
  int32_t v=0; if(!parseDdMm_mmmm_to_e6(f,true,v)) return false;
  if (hemi=='W') v = -v; else if (hemi!='E') return false;
  lon_e6 = v; return true;
}

bool NMEASlim::parseAlt_dm(const char* f, int32_t& alt_dm) {
  if (!f || !*f) return false;
  int neg = 0; int i=0;
  if (f[0]=='-'){ neg=1; i=1; }
  int m_int=0;
  for (; f[i] && f[i] != '.'; ++i) {
    if (f[i]<'0'||f[i]>'9') return false;
    m_int = m_int*10 + (f[i]-'0');
    if (m_int > 2000000) break;
  }
  int dm = m_int*10;
  if (f[i]=='.') {
    ++i;
    if (f[i]>='0'&&f[i]<='9') dm += (f[i]-'0');
  }
  if (neg) dm = -dm;
  alt_dm = dm; return true;
}

// --- Core -----------------------------------------------------------------

void NMEASlim::begin(Stream* s) {
  _s = s;
  _idx = 0;
}

void NMEASlim::poll() {
  if (!_s) return;
  while (_s->available()) {
    char c = (char)_s->read();
    if (c == '\\r') continue;
    if (c == '$') { _idx = 0; _line[_idx++] = c; continue; }
    if (_idx == 0) continue;
    if (_idx < sizeof(_line)-1) _line[_idx++] = c;
    if (c == '\\n') {
      _line[_idx] = '\\0';
      handleLine(_line, _idx);
      _idx = 0;
    }
  }
}

void NMEASlim::handleLine(char* line, uint8_t len) {
  if (len < 9 || line[0]!='$') return;
  char* star = nullptr;
  for (uint8_t i=1;i<len;i++) if (line[i]=='*'){ star=&line[i]; break; }
  if (!star || (star - line) < 2 || (star - line) > len-3) return;
  if (!checkChecksum(line+1, star, star+1)) return;

  char type[6] = {0,0,0,0,0,0};
  for (int i=1;i<=5 && line[i] && line[i]!=','; ++i) type[i-1]=line[i];

  char* p = line+1;
  while (*p && *p!=',') ++p;
  if (*p==',') ++p;

  char* tok[20];
  int nt = splitCSV(p, tok, 20);

  if      (type[2]=='R' && type[3]=='M' && type[4]=='C') parseRMC(tok, nt);
  else if (type[2]=='G' && type[3]=='G' && type[4]=='A') parseGGA(tok, nt);
}

void NMEASlim::parseRMC(char** t, int n) {
  if (n < 2) return;
  bool valid = (t[1] && t[1][0]=='A');
  _d.fix_valid = valid;

  if (n >= 1) parseHhMmSs(t[0], _d.time_utc);

  if (valid && n >= 6) {
    int32_t la, lo;
    if (parseLat_e6(t[2], t[3] ? t[3][0] : '?', la) &&
        parseLon_e6(t[4], t[5] ? t[5][0] : '?', lo)) {
      _d.lat_e6 = la; _d.lon_e6 = lo;
    }
  }
}

void NMEASlim::parseGGA(char** t, int n) {
  if (n < 6) return;
  uint8_t fix = (uint8_t)(t[5] && t[5][0] ? (t[5][0]-'0') : 0);
  _d.gga_valid = (fix > 0);

  if (n >= 6) {
    int32_t la, lo;
    if (parseLat_e6(t[1], t[2] ? t[2][0] : '?', la) &&
        parseLon_e6(t[3], t[4] ? t[4][0] : '?', lo)) {
      _d.lat_e6 = la; _d.lon_e6 = lo;
    }
  }

  if (n >= 9) {
    int32_t dm;
    if (parseAlt_dm(t[8], dm)) _d.alt_dm = dm;
  }

  if (n >= 7 && t[6] && *t[6]) {
    int s = 0; for (const char* p=t[6]; *p>='0'&&*p<='9'; ++p) s = s*10 + (*p-'0');
    if (s>=0 && s<=64) _d.sats = (uint8_t)s;
  }
}
