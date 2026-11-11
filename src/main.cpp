#include <Arduino.h>
#include <esp_system.h>
#include <stdarg.h>
#include <stdint.h>

// ================= OPSI =================
#define USE_DE_RE_PIN   0      // 1 = modul MAX485 manual (DE & RE disatukan ke PIN_DE_RE)
#define SERIAL_JSON     1      // 1 = kirim JSON ke Serial USB tiap siklus
#define LOG_HTTP        0      // 1 = cetak log tiap request web
#define READ_THD        1      // 1 = baca THD I / V / U
#define READ_ENERGY     1      // 1 = baca akumulasi energi meter (Accum Wh, VARh, VAh: Del, Rec, D+R, D-R)
#define EN_WORD_BE      1      // 1 = register Int64 urutan word besar dulu (standar Schneider); 0 = dibalik
#define THD_SCAN        0      // 1 = scan register untuk mencari alamat THD (alamat sudah ketemu, default 0)
#define SCAN_START      21299
#define SCAN_WINDOWS    4

// Nilai layar meter saat scan THD (hanya dipakai jika THD_SCAN = 1)
#define REF_V1          2.88f
#define REF_V2          2.55f
#define REF_V3          2.50f
#define REF_U1          0.68f
#define REF_U2          0.79f
#define REF_U3          0.76f
#define REF_I1          0.0f
#define REF_I2          0.0f
#define REF_I3          0.0f
#define REF_TOL         0.15f

// ================= PARAMETER MODBUS =================
#define PM_SLAVE_ID     1
#define PM_BAUD         19200
#define PM_CONFIG       SERIAL_8E1     // 8 data, Even, 1 stop (default PM5300)
#define RETRIES         3              // percobaan per blok
#define GAP_MS          4              // bus dianggap sepi minimal sekian ms (3,5 karakter @19200 = ~2 ms + margin)
#define STALE_MS        3000           // data cepat (arus, tegangan, daya, THD) masih dipakai maks segini
#define STALE_ENERGY_MS 30000          // energi berubah sangat lambat, boleh lebih lama
#define POLL_MS         1000           // periode siklus baca (meter update ~1 detik)
#define THD_EVERY       2              // THD dibaca tiap N siklus
#define ENERGY_EVERY    4              // energi dibaca tiap N siklus

#include <SPI.h>
#include <Ethernet.h>
// Penutup celah kompatibilitas ESP32 core 3.x: Server::begin(uint16_t) wajib ada
class ServerCompat : public EthernetServer {
 public:
  ServerCompat(uint16_t port) : EthernetServer(port) {}
  void begin()              { EthernetServer::begin(); }
  void begin(uint16_t)      { EthernetServer::begin(); }
};

// ================= PIN =================
// W5500 (VSPI)
#define ETH_SCK   18
#define ETH_MISO  19
#define ETH_MOSI  23
#define ETH_CS     5
#define ETH_RST    4
// RS485 (Serial2)
#define RXD2      16
#define TXD2      17
#define PIN_DE_RE 25     // hanya jika USE_DE_RE_PIN = 1 (BUKAN GPIO 4: bentrok dengan ETH_RST)

// ================= JARINGAN =================
byte mac[] = {0x02, 0x53, 0x50, 0x32, 0x00, 0x01};
IPAddress ip(192, 168, 10, 2);
IPAddress dnsServer(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

// ================= REGISTER (0-based di kabel; daftar Schneider berbasis-1) =================
// Satu blok gabungan reg 3000..3111 = 56 float, urutan indeks:
//   0..5   Ia, Ib, Ic, In, Ig, Iavg
//   6..9   Unbalance I: A, B, C, Worst (%)
//   10..18 Vab,Vbc,Vca,Vll avg,Van,Vbn,Vcn,rsv,Vln avg
//   19..26 Unbalance V: Vab,Vbc,Vca,Vll worst,Van,Vbn,Vcn,Vln worst (%)
//   27..34 Pa,Pb,Pc,Ptot, Qa,Qb,Qc,Qtot
//   35..54 (celah, tidak dipakai)
//   55     Frekuensi
const uint16_t ADDR_MAIN = 2999;
const uint8_t  N_MAIN    = 27;       // dipakai hanya untuk mode cadangan (blok terpisah)
const uint8_t  N_ALL     = 56;       // reg 3000..3111
const uint16_t ADDR_PQ   = 3053;     // mode cadangan
const uint16_t ADDR_FREQ = 3109;     // mode cadangan
const uint16_t ADDR_PID  = 89;       // register 90 : Product ID

// THD (alamat 0-based = nomor register - 1). Urutan alamat HARUS: THD_I < THD_U < THD_V
const uint16_t ADDR_THD_I = 21299;   // reg 21300: THD Ia, Ib, Ic     (3 float, %)
const uint16_t ADDR_THD_U = 21321;   // reg 21322: THD Vab, Vbc, Vca  (3 float, %)
const uint16_t ADDR_THD_V = 21329;   // reg 21330: THD Van, Vbn, Vcn  (3 float, %)
constexpr uint16_t THD_OFF_U = ADDR_THD_U - ADDR_THD_I;
constexpr uint16_t THD_OFF_V = ADDR_THD_V - ADDR_THD_I;
constexpr uint16_t THD_SPAN  = THD_OFF_V + 6;     // dibaca sekaligus dalam 1 request

// Akumulasi energi: Int64 = 4 register per nilai (Del, Rec, Del+Rec, Del-Rec), 16 register per blok
//   reg 3204..3219 Wh | 3220..3235 VARh | 3236..3251 VAh  -> 48 register berurutan
const uint16_t ADDR_EN_ACT = 3203;
const uint16_t ADDR_EN_REA = 3219;
const uint16_t ADDR_EN_APP = 3235;

// ================= DATA BERSAMA =================
struct PmData {
  float cur[6];
  float iunb[4];
  float volt[9];
  float vunb[8];
  float pq[8];       // 0..3 = P1,P2,P3,Ptot ; 4..7 = Q1,Q2,Q3,Qtot
  float freq;
  float thdI[3];
  float thdV[3];
  float thdU[3];
  double en[12];     // 0..3 aktif Wh, 4..7 reaktif VARh, 8..11 semu VAh (Del,Rec,D+R,D-R)
  uint32_t tMain, tPq, tFreq, tThdI, tThdV, tThdU, tEnA, tEnR, tEnS;   // millis() terakhir sukses (0 = belum pernah)
  uint8_t  eMain, ePq, eFreq, eThd, eEn;     // kode error terakhir (0 = OK)
  uint16_t pid;
  bool     pidOk;
  uint32_t totalReq, failReq, foreign;
};

PmData shared;
SemaphoreHandle_t dataMutex;

// Hanya diakses task Modbus
uint32_t totalReq = 0, failReq = 0;
uint32_t foreignBursts = 0;             // statistik: jumlah burst dari master lain (LoRa) yang terdengar
static uint8_t toStreak = 0;            // jumlah timeout beruntun
static inline bool busDead() { return toStreak >= 3; }

ServerCompat server(80);
bool ethOK = false;
EthernetLinkStatus linkTerakhir = Unknown;
volatile bool linkOn = false;           // diubah HANYA oleh loop() (core 1)

// ================= UTIL =================
void logf(const char *fmt, ...) {
  char b[160];
  va_list a;
  va_start(a, fmt);
  vsnprintf(b, sizeof(b), fmt, a);
  va_end(a);
  Serial.print("# ");
  Serial.println(b);
}

void fillNaN(float *p, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) p[i] = NAN;
}

void fillNaND(double *p, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) p[i] = NAN;
}

void initData(PmData &d) {
  memset(&d, 0, sizeof(d));
  fillNaN(d.cur, 6);
  fillNaN(d.iunb, 4);
  fillNaN(d.volt, 9);
  fillNaN(d.vunb, 8);
  fillNaN(d.pq, 8);
  fillNaN(d.thdI, 3);
  fillNaN(d.thdV, 3);
  fillNaN(d.thdU, 3);
  fillNaND(d.en, 12);
  d.freq = NAN;
}

// Nilai di luar [lo,hi] (termasuk NaN) dijadikan NaN
void limitRange(float *p, uint8_t n, float lo, float hi) {
  for (uint8_t i = 0; i < n; i++)
    if (!(p[i] >= lo && p[i] <= hi)) p[i] = NAN;
}

float regsToFloat(uint16_t hi, uint16_t lo) {
  uint32_t raw = ((uint32_t)hi << 16) | lo;
  float f;
  memcpy(&f, &raw, sizeof(f));
  if (isnan(f) || isinf(f) || fabsf(f) > 1e30f) return NAN;
  return f;
}

// 4 register -> Int64 bertanda. 0x8000... = "tidak tersedia" -> NaN
double regsToInt64(const uint16_t *r) {
#if EN_WORD_BE
  uint64_t v = ((uint64_t)r[0] << 48) | ((uint64_t)r[1] << 32) | ((uint64_t)r[2] << 16) | (uint64_t)r[3];
#else
  uint64_t v = ((uint64_t)r[3] << 48) | ((uint64_t)r[2] << 32) | ((uint64_t)r[1] << 16) | (uint64_t)r[0];
#endif
  int64_t s = (int64_t)v;
  if (s == INT64_MIN) return NAN;
  return (double)s;
}

void snapshot(PmData &out) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  out = shared;
  xSemaphoreGive(dataMutex);
}

// ================= MODBUS RTU =================
static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

// Carrier sense yang sadar master lain.
// RS485 itu bus bersama, jadi request master lain (LoRa) ikut terdengar di sini.
// Jika terdengar request 8 byte ke slave kita, meter akan menjawab sebentar lagi,
// maka jawaban itu ditunggu sampai selesai sebelum kita bicara.
// Jeda sepi diberi jitter acak supaya dua master tidak selalu sinkron.
// true = bus bebas.
static bool waitIdle() {
  static uint8_t sn[260];
  const uint32_t t0 = millis();
  uint32_t last = t0, expDeadline = 0;
  const uint32_t need = GAP_MS + (esp_random() % 6);
  size_t n = 0, expect = 0;               // expect > 0: sedang menunggu jawaban meter untuk master lain

  while (millis() - t0 < 800) {
    bool got = false;
    while (Serial2.available()) {
      int c = Serial2.read();
      if (n < sizeof(sn)) sn[n++] = (uint8_t)c;
      got = true;
    }
    uint32_t now = millis();
    if (got) last = now;

    if (expect) {
      if (n >= expect || (n >= 5 && (sn[1] & 0x80)) || now > expDeadline) {
        expect = 0; n = 0; last = now;
      }
      delay(1);
      continue;
    }
    if (n && now - last >= GAP_MS) {       // satu burst selesai
      foreignBursts++;
      if (n == 8 && sn[0] == PM_SLAVE_ID && sn[1] == 0x03) {   // request master lain
        uint16_t q = ((uint16_t)sn[4] << 8) | sn[5];
        if (q > 125) q = 125;
        expect = 5 + 2 * (size_t)q;
        expDeadline = now + 150 + expect;  // latensi meter + waktu frame
      }
      n = 0; last = now;
    } else if (n == 0 && now - last >= need) {
      return true;
    }
    delay(1);
  }
  return false;
}

// Fungsi 0x03. Mengembalikan:
//   0 = OK | 1..4 = exception dari meter | 0xE0 ID slave salah | 0xE1 fungsi salah
//   0xE2 timeout/terpotong | 0xE3 CRC salah | 0xE4 jumlah byte tidak cocok
//   0xE5 ada byte tambahan (tabrakan) | 0xE6 bus tidak pernah bebas | 0xE7 data tidak masuk akal
// 'regs' TIDAK diubah saat gagal.
uint8_t mbRead(uint16_t addr, uint16_t qty, uint16_t *regs) {
  static uint8_t rx[260];
  uint8_t req[8] = { PM_SLAVE_ID, 0x03, (uint8_t)(addr >> 8), (uint8_t)addr,
                     (uint8_t)(qty >> 8), (uint8_t)qty, 0, 0 };
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;

  if (!waitIdle()) return 0xE6;           // bus terus ramai, jangan memaksa

#if USE_DE_RE_PIN
  digitalWrite(PIN_DE_RE, HIGH);
#endif
  Serial2.write(req, 8);
  Serial2.flush();
#if USE_DE_RE_PIN
  digitalWrite(PIN_DE_RE, LOW);
#endif

  const size_t   expect  = 5 + 2 * (size_t)qty;
  const uint32_t timeout = 150 + (expect * 6) / 10;     // latensi meter + waktu frame
  size_t n = 0;
  uint32_t t0 = millis(), lastRx = t0;

  while (millis() - t0 < timeout) {
    if (Serial2.available()) {
      while (Serial2.available() && n < sizeof(rx)) rx[n++] = Serial2.read();
      lastRx = millis();
    }
    bool isExc = (n >= 2 && (rx[1] & 0x80));
    if (isExc && n >= 5) break;
    if (n >= expect) break;
    if (n > 0 && millis() - lastRx > 10) break;         // frame terpotong: gagal cepat
    delay(1);
  }

  // byte tambahan setelah frame lengkap = tabrakan
  delay(GAP_MS - 1);
  while (Serial2.available() && n < sizeof(rx)) rx[n++] = Serial2.read();

  if (n < 5) return 0xE2;
  if (rx[0] != PM_SLAVE_ID) return 0xE0;
  if (rx[1] & 0x80) {
    if (n > 5) return 0xE5;
    uint16_t cc = crc16(rx, 3);
    if (rx[3] != (cc & 0xFF) || rx[4] != (cc >> 8)) return 0xE3;
    return rx[2] ? rx[2] : 0xE4;
  }
  if (rx[1] != 0x03) return 0xE1;
  if (rx[2] != 2 * qty) return 0xE4;                    // panjang tidak cocok = jawaban nyasar
  if (n < expect) return 0xE2;
  if (n > expect) return 0xE5;
  uint16_t cc = crc16(rx, expect - 2);
  if (rx[expect - 2] != (cc & 0xFF) || rx[expect - 1] != (cc >> 8)) return 0xE3;

  for (uint16_t i = 0; i < qty; i++)
    regs[i] = ((uint16_t)rx[3 + 2 * i] << 8) | rx[4 + 2 * i];
  return 0;
}

// Baca 'qty' register mentah dengan percobaan ulang. 'regs' TIDAK diubah saat gagal.
uint8_t readRaw(uint16_t addr, uint16_t qty, uint16_t *regs) {
  uint8_t r = 0xFF;
  for (uint8_t a = 0; a < RETRIES; a++) {
    totalReq++;
    r = mbRead(addr, qty, regs);
    if (r == 0) { toStreak = 0; return 0; }
    failReq++;
    if (r >= 0x01 && r <= 0x04) break;                  // exception meter: mengulang percuma
    if (r == 0xE2 && toStreak < 255) toStreak++;        // hanya timeout yang dihitung "meter mati"
    delay(10 + (esp_random() % 40));                    // backoff acak sebelum coba lagi
  }
  return r;
}

// Mengembalikan 0 jika sukses, selain itu kode error. 'out' TIDAK diubah saat gagal.
uint8_t readBlock(uint16_t addr, uint8_t nFloat, float *out) {
  static uint16_t regs[125];
  uint8_t r = readRaw(addr, (uint16_t)nFloat * 2, regs);
  if (r == 0) {
    for (uint8_t i = 0; i < nFloat; i++)
      out[i] = regsToFloat(regs[2 * i], regs[2 * i + 1]);
  }
  return r;
}

// ---------- Blok utama: arus, tegangan, daya, frekuensi ----------
// Utama: 1 request 112 register (reg 3000..3111). Jika meter menolak (exception, mis. ada
// register di celah yang tidak dikenal), otomatis pindah ke 3 blok terpisah dengan tata letak sama.
static uint8_t readMain(float *f, bool &combined) {
  if (combined) {
    uint8_t e = readBlock(ADDR_MAIN, N_ALL, f);
    if (e >= 0x01 && e <= 0x04) {
      combined = false;
      logf("Blok gabungan 3000..3111 ditolak meter (kode 0x%02X), pakai 3 blok terpisah", e);
    } else {
      return e;
    }
  }
  static float tmp[N_MAIN], pq[8], fr[1];
  uint8_t e = readBlock(ADDR_MAIN, N_MAIN, tmp);
  if (e) return e;
  e = readBlock(ADDR_PQ, 8, pq);
  if (e) return e;
  e = readBlock(ADDR_FREQ, 1, fr);
  if (e) return e;
  fillNaN(f, N_ALL);
  memcpy(f,      tmp, sizeof(tmp));
  memcpy(f + 27, pq,  sizeof(pq));
  f[55] = fr[0];
  return 0;
}

static bool mismatch(float a, float b, float rel, float absTol) {
  if (isnan(a) || isnan(b)) return false;               // tidak tersedia -> lewati
  return fabsf(a - b) > rel * fabsf(b) + absTol;
}

// Validasi isi: jawaban nyasar yang lolos CRC hampir pasti melanggar hubungan antar nilai ini.
// Toleransi adalah tebakan awal; longgarkan jika terlalu sering menolak data asli (kode 0xE7).
static bool sane(const float *f) {
  if (!(f[55] >= 45.0f && f[55] <= 65.0f)) return false;                          // frekuensi
  if (mismatch((f[0] + f[1] + f[2]) / 3, f[5], 0.02f, 0.05f)) return false;       // Iavg
  if (mismatch((f[10] + f[11] + f[12]) / 3, f[13], 0.01f, 1.0f)) return false;    // Vll avg
  if (mismatch((f[14] + f[15] + f[16]) / 3, f[18], 0.01f, 1.0f)) return false;    // Vln avg
  if (mismatch(f[27] + f[28] + f[29], f[30], 0.03f, 0.1f)) return false;          // P total
  return true;
}

// ---------- THD ----------
static void thdStore(PmData &t, float *a, float *b, float *u, uint32_t ts) {
  static bool logged = false;
  if (!logged) {
    logged = true;
    logf("THD I mentah: %.3f %.3f %.3f", a[0], a[1], a[2]);
    logf("THD V mentah: %.3f %.3f %.3f", b[0], b[1], b[2]);
    logf("THD U mentah: %.3f %.3f %.3f", u[0], u[1], u[2]);
  }
  limitRange(a, 3, 0, 1000);
  limitRange(b, 3, 0, 1000);
  limitRange(u, 3, 0, 1000);
  memcpy(t.thdI, a, 3 * sizeof(float));
  memcpy(t.thdV, b, 3 * sizeof(float));
  memcpy(t.thdU, u, 3 * sizeof(float));
  t.tThdI = t.tThdV = t.tThdU = ts;
}

// 1 request untuk ketiganya; jika ditolak meter, 3 request terpisah.
static uint8_t readThd(PmData &t, bool &combined) {
  float a[3], b[3], u[3];
  if (combined) {
    static uint16_t regs[THD_SPAN];
    uint8_t e = readRaw(ADDR_THD_I, THD_SPAN, regs);
    if (e == 0) {
      for (uint8_t i = 0; i < 3; i++) {
        a[i] = regsToFloat(regs[2 * i],              regs[2 * i + 1]);
        u[i] = regsToFloat(regs[THD_OFF_U + 2 * i],  regs[THD_OFF_U + 2 * i + 1]);
        b[i] = regsToFloat(regs[THD_OFF_V + 2 * i],  regs[THD_OFF_V + 2 * i + 1]);
      }
      thdStore(t, a, b, u, millis());
      return 0;
    }
    if (e >= 0x01 && e <= 0x04) {
      combined = false;
      logf("Blok THD gabungan ditolak meter (kode 0x%02X), pakai 3 blok terpisah", e);
    } else {
      return e;
    }
  }
  uint8_t e1 = readBlock(ADDR_THD_I, 3, a);
  if (e1 == 0) { limitRange(a, 3, 0, 1000); memcpy(t.thdI, a, sizeof(a)); t.tThdI = millis(); }
  uint8_t e2 = busDead() ? 0xE2 : readBlock(ADDR_THD_V, 3, b);
  if (e2 == 0) { limitRange(b, 3, 0, 1000); memcpy(t.thdV, b, sizeof(b)); t.tThdV = millis(); }
  uint8_t e3 = busDead() ? 0xE2 : readBlock(ADDR_THD_U, 3, u);
  if (e3 == 0) { limitRange(u, 3, 0, 1000); memcpy(t.thdU, u, sizeof(u)); t.tThdU = millis(); }
  return e1 ? e1 : (e2 ? e2 : e3);
}

// ---------- Energi ----------
// 1 request 48 register (reg 3204..3251); jika ditolak meter, 3 request @16 register.
static uint8_t readEnergy(double *out12, bool &combined) {
  if (combined) {
    static uint16_t r48[48];
    uint8_t e = readRaw(ADDR_EN_ACT, 48, r48);
    if (e == 0) {
      for (uint8_t i = 0; i < 12; i++) out12[i] = regsToInt64(r48 + 4 * i);
      return 0;
    }
    if (e >= 0x01 && e <= 0x04) {
      combined = false;
      logf("Blok energi gabungan ditolak meter (kode 0x%02X), pakai 3 blok terpisah", e);
    } else {
      return e;
    }
  }
  static uint16_t r16[16];
  const uint16_t base[3] = { ADDR_EN_ACT, ADDR_EN_REA, ADDR_EN_APP };
  for (uint8_t g = 0; g < 3; g++) {
    if (g && busDead()) return 0xE2;
    uint8_t e = readRaw(base[g], 16, r16);
    if (e) return e;
    for (uint8_t i = 0; i < 4; i++) out12[4 * g + i] = regsToInt64(r16 + 4 * i);
  }
  return 0;
}

// Energi kumulatif tidak boleh turun, dan D+R / D-R harus cocok dengan Del / Rec.
static bool energyOk(const double *e, const double *prev) {
  for (uint8_t g = 0; g < 3; g++) {
    const double *x = e + 4 * g;
    if (isnan(x[0]) || isnan(x[1]) || isnan(x[2]) || isnan(x[3])) continue;
    double tol = fabs(x[2]) * 0.0005 + 2.0;
    if (fabs((x[0] + x[1]) - x[2]) > tol || fabs((x[0] - x[1]) - x[3]) > tol) return false;
    const double *p = prev + 4 * g;
    if (!isnan(p[0]) && x[0] < p[0] - 1) return false;  // Del turun = data nyasar
    if (!isnan(p[1]) && x[1] < p[1] - 1) return false;  // Rec turun
  }
  return true;
}

// ================= JSON =================
static void jNum(char *buf, size_t cap, size_t &p, const char *key, float v) {
  if (p + 1 >= cap) return;
  int n = isnan(v) ? snprintf(buf + p, cap - p, "\"%s\":null,", key)
                   : snprintf(buf + p, cap - p, "\"%s\":%.2f,", key, v);
  if (n > 0) p += n;
  if (p >= cap) p = cap - 1;
}

// Bilangan bulat besar (energi), tanpa desimal
static void jInt(char *buf, size_t cap, size_t &p, const char *key, double v) {
  if (p + 1 >= cap) return;
  int n = isnan(v) ? snprintf(buf + p, cap - p, "\"%s\":null,", key)
                   : snprintf(buf + p, cap - p, "\"%s\":%.0f,", key, v);
  if (n > 0) p += n;
  if (p >= cap) p = cap - 1;
}

size_t buildJson(const PmData &in, char *buf, size_t cap) {
  PmData d = in;                                   // salinan lokal
  uint32_t now = millis();
  bool fM  = d.tMain  && (now - d.tMain)  <= STALE_MS;
  bool fP  = d.tPq    && (now - d.tPq)    <= STALE_MS;
  bool fF  = d.tFreq  && (now - d.tFreq)  <= STALE_MS;
  bool fTI = d.tThdI  && (now - d.tThdI)  <= STALE_MS;
  bool fTV = d.tThdV  && (now - d.tThdV)  <= STALE_MS;
  bool fTU = d.tThdU  && (now - d.tThdU)  <= STALE_MS;
  bool fEA = d.tEnA   && (now - d.tEnA)   <= STALE_ENERGY_MS;
  bool fER = d.tEnR   && (now - d.tEnR)   <= STALE_ENERGY_MS;
  bool fES = d.tEnS   && (now - d.tEnS)   <= STALE_ENERGY_MS;
  if (!fM) { fillNaN(d.cur, 6); fillNaN(d.iunb, 4); fillNaN(d.volt, 9); fillNaN(d.vunb, 8); }
  if (!fP) fillNaN(d.pq, 8);
  if (!fF) d.freq = NAN;
  if (!fTI) fillNaN(d.thdI, 3);
  if (!fTV) fillNaN(d.thdV, 3);
  if (!fTU) fillNaN(d.thdU, 3);
  if (!fEA) fillNaND(d.en, 4);
  if (!fER) fillNaND(d.en + 4, 4);
  if (!fES) fillNaND(d.en + 8, 4);

  size_t p = 0;
  buf[p++] = '{';

  jNum(buf, cap, p, "ia",       d.cur[0]);
  jNum(buf, cap, p, "ib",       d.cur[1]);
  jNum(buf, cap, p, "ic",       d.cur[2]);
  jNum(buf, cap, p, "in_curr",  d.cur[3]);
  jNum(buf, cap, p, "i_avg",    d.cur[5]);

  jNum(buf, cap, p, "i_unb_a",  d.iunb[0]);
  jNum(buf, cap, p, "i_unb_b",  d.iunb[1]);
  jNum(buf, cap, p, "i_unb_c",  d.iunb[2]);
  jNum(buf, cap, p, "i_unb",    d.iunb[3]);      // worst

  jNum(buf, cap, p, "van",      d.volt[4]);
  jNum(buf, cap, p, "vbn",      d.volt[5]);
  jNum(buf, cap, p, "vcn",      d.volt[6]);
  jNum(buf, cap, p, "v_ln_avg", d.volt[8]);

  jNum(buf, cap, p, "vab",      d.volt[0]);
  jNum(buf, cap, p, "vbc",      d.volt[1]);
  jNum(buf, cap, p, "vca",      d.volt[2]);
  jNum(buf, cap, p, "v_ll_avg", d.volt[3]);

  jNum(buf, cap, p, "v_unb_ab", d.vunb[0]);
  jNum(buf, cap, p, "v_unb_bc", d.vunb[1]);
  jNum(buf, cap, p, "v_unb_ca", d.vunb[2]);
  jNum(buf, cap, p, "v_unb_ll", d.vunb[3]);      // worst L-L
  jNum(buf, cap, p, "v_unb_an", d.vunb[4]);
  jNum(buf, cap, p, "v_unb_bn", d.vunb[5]);
  jNum(buf, cap, p, "v_unb_cn", d.vunb[6]);
  jNum(buf, cap, p, "v_unb_ln", d.vunb[7]);      // worst L-N

  jNum(buf, cap, p, "p1",       d.pq[0]);
  jNum(buf, cap, p, "p2",       d.pq[1]);
  jNum(buf, cap, p, "p3",       d.pq[2]);
  jNum(buf, cap, p, "p_total",  d.pq[3]);

  jNum(buf, cap, p, "q1",       d.pq[4]);
  jNum(buf, cap, p, "q2",       d.pq[5]);
  jNum(buf, cap, p, "q3",       d.pq[6]);
  jNum(buf, cap, p, "q_total",  d.pq[7]);

  jNum(buf, cap, p, "freq",     d.freq);

  jNum(buf, cap, p, "thd_ia",   d.thdI[0]);
  jNum(buf, cap, p, "thd_ib",   d.thdI[1]);
  jNum(buf, cap, p, "thd_ic",   d.thdI[2]);
  jNum(buf, cap, p, "thd_van",  d.thdV[0]);
  jNum(buf, cap, p, "thd_vbn",  d.thdV[1]);
  jNum(buf, cap, p, "thd_vcn",  d.thdV[2]);
  jNum(buf, cap, p, "thd_uab",  d.thdU[0]);
  jNum(buf, cap, p, "thd_ubc",  d.thdU[1]);
  jNum(buf, cap, p, "thd_uca",  d.thdU[2]);

  jInt(buf, cap, p, "ea_del",   d.en[0]);        // Wh
  jInt(buf, cap, p, "ea_rec",   d.en[1]);
  jInt(buf, cap, p, "ea_sum",   d.en[2]);        // D+R
  jInt(buf, cap, p, "ea_dif",   d.en[3]);        // D-R
  jInt(buf, cap, p, "er_del",   d.en[4]);        // VARh
  jInt(buf, cap, p, "er_rec",   d.en[5]);
  jInt(buf, cap, p, "er_sum",   d.en[6]);
  jInt(buf, cap, p, "er_dif",   d.en[7]);
  jInt(buf, cap, p, "es_del",   d.en[8]);        // VAh
  jInt(buf, cap, p, "es_rec",   d.en[9]);
  jInt(buf, cap, p, "es_sum",   d.en[10]);
  jInt(buf, cap, p, "es_dif",   d.en[11]);

  char pidStr[8];
  if (d.pidOk) snprintf(pidStr, sizeof(pidStr), "%u", d.pid);
  else strcpy(pidStr, "null");

  // ok = data utama segar DAN (jika energi dibaca) energi masih dalam batas basi energi
  bool okAll = fM && fP && fF && (READ_ENERGY ? fEA : true);

  // e_cur & e_volt = error blok utama (kompatibel dengan versi lama)
  int n = snprintf(buf + p, cap - p,
                   "\"ok\":%d,\"e_cur\":%u,\"e_volt\":%u,\"e_pq\":%u,\"e_freq\":%u,\"e_thd\":%u,\"e_en\":%u,\"pid\":%s,"
                   "\"req\":%lu,\"fail\":%lu,\"fb\":%lu,\"uptime_s\":%lu,\"link\":\"%s\"}",
                   okAll ? 1 : 0,
                   d.eMain, d.eMain, d.ePq, d.eFreq, d.eThd, d.eEn, pidStr,
                   (unsigned long)d.totalReq, (unsigned long)d.failReq, (unsigned long)d.foreign,
                   (unsigned long)(millis() / 1000),
                   (ethOK && linkOn) ? "ON" : "OFF");
  if (n > 0) p += n;
  if (p >= cap) p = cap - 1;
  buf[p] = 0;
  return p;
}

// ================= SCAN THD =================
#if THD_SCAN
static bool near3(const float *s, uint8_t i, float r1, float r2, float r3) {
  if (r1 <= 0 || r2 <= 0 || r3 <= 0) return false;
  return fabsf(s[i] - r1) < REF_TOL && fabsf(s[i + 1] - r2) < REF_TOL && fabsf(s[i + 2] - r3) < REF_TOL;
}

static bool nearAny(float v, float r1, float r2, float r3) {
  return (r1 > 0 && fabsf(v - r1) < REF_TOL) || (r2 > 0 && fabsf(v - r2) < REF_TOL) ||
         (r3 > 0 && fabsf(v - r3) < REF_TOL);
}

void doThdScan() {
  static float s[60];
  static uint8_t win = 0;
  uint16_t start = SCAN_START + (uint16_t)win * 120;
  win = (win + 1) % SCAN_WINDOWS;

  uint8_t e = readBlock(start, 60, s);
  if (e != 0) {
    logf("SCAN reg %u..%u gagal, kode 0x%02X", start + 1, start + 120, e);
    return;
  }
  logf("--- SCAN reg %u..%u (reg 1-based = alamat kabel + 1) ---", start + 1, start + 120);
  for (uint8_t i = 0; i < 60; i++) {
    if (isnan(s[i])) continue;
    const char *tag = "";
    if (nearAny(s[i], REF_V1, REF_V2, REF_V3)) tag = "  <== mirip THD V";
    else if (nearAny(s[i], REF_U1, REF_U2, REF_U3)) tag = "  <== mirip THD U";
    else if (nearAny(s[i], REF_I1, REF_I2, REF_I3)) tag = "  <== mirip THD I";
    logf("reg %u = %.4f%s", start + 1 + 2 * i, s[i], tag);
  }
  for (uint8_t i = 0; i + 2 < 60; i++) {
    if (isnan(s[i]) || isnan(s[i + 1]) || isnan(s[i + 2])) continue;
    uint16_t wire = start + 2 * i;
    if (near3(s, i, REF_V1, REF_V2, REF_V3)) logf("USULAN ADDR_THD_V = %u  (reg %u)", wire, wire + 1);
    if (near3(s, i, REF_U1, REF_U2, REF_U3)) logf("USULAN ADDR_THD_U = %u  (reg %u)", wire, wire + 1);
    if (near3(s, i, REF_I1, REF_I2, REF_I3)) logf("USULAN ADDR_THD_I = %u  (reg %u)", wire, wire + 1);
  }
}
#endif

// ================= TASK MODBUS (core 0) =================
void modbusTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  static char  js[1900];
  static float f[N_ALL];

  PmData t;
  initData(t);
  uint8_t pidTries = 0;
  uint8_t cyc = 0;
  bool mainCombined = true, thdCombined = true, enCombined = true;

  for (;;) {
    // --- 1 request: arus, unbalance, tegangan, daya, frekuensi (reg 3000..3111) ---
    uint8_t e = readMain(f, mainCombined);
    if (e == 0 && !sane(f)) { e = 0xE7; failReq++; }     // data masuk akal? kalau tidak, buang
    if (e == 0) {
      uint32_t now = millis();
      memcpy(t.cur,  f,      6 * sizeof(float));
      memcpy(t.iunb, f + 6,  4 * sizeof(float));
      memcpy(t.volt, f + 10, 9 * sizeof(float));
      memcpy(t.vunb, f + 19, 8 * sizeof(float));
      memcpy(t.pq,   f + 27, 8 * sizeof(float));
      t.freq = f[55];
      t.tMain = t.tPq = t.tFreq = now;
    }
    t.eMain = t.ePq = t.eFreq = e;
    static uint8_t lastEm = 0xFF;
    if (e != lastEm) { lastEm = e; logf("Blok utama: kode 0x%02X (0 = OK)", e); }

#if READ_THD
    if (!busDead() && (cyc % THD_EVERY == 0)) {
      t.eThd = readThd(t, thdCombined);
      static uint8_t lastEt = 0xFF;
      if (t.eThd != lastEt) { lastEt = t.eThd; logf("THD: kode 0x%02X (0 = OK)", t.eThd); }
    }
#endif
    cyc++;

#if READ_ENERGY
    static uint8_t ecyc = 0;
    if (!busDead() && (ecyc++ % ENERGY_EVERY == 0)) {
      double e12[12];
      uint8_t k = readEnergy(e12, enCombined);
      if (k == 0) {
        if (energyOk(e12, t.en)) {
          static bool enLogged = false;
          if (!enLogged) {
            enLogged = true;
            logf("ENERGI Wh   : del=%.0f rec=%.0f D+R=%.0f D-R=%.0f", e12[0], e12[1], e12[2], e12[3]);
            logf("ENERGI VARh : del=%.0f rec=%.0f D+R=%.0f D-R=%.0f", e12[4], e12[5], e12[6], e12[7]);
            logf("ENERGI VAh  : del=%.0f rec=%.0f D+R=%.0f D-R=%.0f", e12[8], e12[9], e12[10], e12[11]);
          }
          memcpy(t.en, e12, sizeof(e12));
          t.tEnA = t.tEnR = t.tEnS = millis();
        } else {
          k = 0xE7;                                       // tidak konsisten: buang
          failReq++;
        }
      }
      t.eEn = k;
      static uint8_t lastEe = 0xFF;
      if (k != lastEe) { lastEe = k; logf("ENERGI: kode 0x%02X (0 = OK)", k); }
    }
#endif

#if THD_SCAN
    static uint8_t scanDiv = 9;
    if (!busDead() && ++scanDiv >= 10) {       // ~tiap 10 detik, satu jendela per putaran
      scanDiv = 0;
      doThdScan();
    }
#endif

    // Diagnosa model: baca Product ID (maks 3 kali)
    if (!t.pidOk && pidTries < 3 && !busDead()) {
      pidTries++;
      uint16_t pidReg = 0;
      uint8_t r = readRaw(ADDR_PID, 1, &pidReg);
      if (r == 0) {
        t.pid = pidReg;
        t.pidOk = true;
        logf("Product ID (reg %u) = %u", ADDR_PID + 1, t.pid);
      } else {
        logf("Product ID gagal dibaca, kode 0x%02X", r);
      }
    }

    t.totalReq = totalReq;
    t.failReq  = failReq;
    t.foreign  = foreignBursts;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    shared = t;
    xSemaphoreGive(dataMutex);

#if SERIAL_JSON
    size_t n = buildJson(t, js, sizeof(js) - 1);
    js[n] = '\n';
    Serial.write((const uint8_t *)js, n + 1);     // satu kali tulis agar baris tidak terpotong
#endif

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(POLL_MS));
  }
}

// ================= WEB SERVER =================
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Power Meter Monitor</title>
<style>
body{font-family:sans-serif;margin:16px;background:#f3f7f1;color:#222}
h1{font-size:20px}
.g{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));gap:8px}
.c{background:#fff;border-radius:10px;padding:10px;box-shadow:0 1px 3px #0002}
.c b{display:block;font-size:20px}.c span{font-size:12px;color:#666}
#s{font-size:12px;color:#666;margin-top:10px}
</style></head><body>
<h1>Power Meter Monitor</h1><div class="g" id="g"></div><div id="s">menunggu data...</div>
<script>
const K=[["ia","Ia (A)"],["ib","Ib (A)"],["ic","Ic (A)"],["i_avg","I rata2 (A)"],
["van","Van (V)"],["vbn","Vbn (V)"],["vcn","Vcn (V)"],["vab","Vab (V)"],["vbc","Vbc (V)"],["vca","Vca (V)"],
["p_total","P total (kW)"],["q_total","Q total (kvar)"],["freq","Frekuensi (Hz)"],
["i_unb","Unbalance I (%)"],["v_unb_ll","Unbalance V L-L (%)"],["v_unb_ln","Unbalance V L-N (%)"],
["thd_ia","THD I Ia (%)"],["thd_ib","THD I Ib (%)"],["thd_ic","THD I Ic (%)"],
["thd_van","THD V Van (%)"],["thd_vbn","THD V Vbn (%)"],["thd_vcn","THD V Vcn (%)"],
["thd_uab","THD U Vab (%)"],["thd_ubc","THD U Vbc (%)"],["thd_uca","THD U Vca (%)"],
["ea_del","Wh Del"],["ea_rec","Wh Rec"],["er_del","VARh Del"],["er_rec","VARh Rec"],["es_del","VAh Del"],["es_rec","VAh Rec"]];
const g=document.getElementById("g"),s=document.getElementById("s");
K.forEach(k=>{g.insertAdjacentHTML("beforeend",'<div class="c"><span>'+k[1]+'</span><b id="'+k[0]+'">-</b></div>')});
async function t(){try{const r=await fetch("/data"),d=await r.json();
K.forEach(k=>{const v=d[k[0]];document.getElementById(k[0]).textContent=(v===null||v===undefined)?"-":v});
s.textContent=(d.ok?"OK":"Ada blok gagal")+" | req "+d.req+" | gagal "+d.fail+" | burst master lain "+d.fb+" | err "+[d.e_cur,d.e_thd,d.e_en].join("/")+" | uptime "+d.uptime_s+" s"}
catch(e){s.textContent="koneksi ke ESP32 terputus"}}
t();setInterval(t,1000);
</script></body></html>)HTML";

void sendChunked(EthernetClient &c, const char *data, size_t len) {
  const size_t CH = 512;
  size_t off = 0;
  while (off < len && c.connected()) {
    size_t n = min(CH, len - off);
    c.write((const uint8_t *)(data + off), n);
    off += n;
  }
}

void sendJson(EthernetClient &c) {
  static char body[1700];
  static char out[1900];                 // harus < 2 KB (buffer TX socket W5500)
  PmData d;
  snapshot(d);
  size_t bl = buildJson(d, body, sizeof(body));
  int hl = snprintf(out, sizeof(out),
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\nCache-Control: no-store\r\n"
                    "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)bl);
  if (hl + bl < sizeof(out)) {
    memcpy(out + hl, body, bl);          // header + body dikirim sekali (1 paket)
    c.write((const uint8_t *)out, hl + bl);
  }
}

void sendPage(EthernetClient &c) {
  char hdr[160];
  int hl = snprintf(hdr, sizeof(hdr),
                    "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                    "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)strlen(PAGE));
  c.write((const uint8_t *)hdr, hl);
  sendChunked(c, PAGE, strlen(PAGE));
}

void send404(EthernetClient &c) {
  c.print("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n"
          "Content-Length: 9\r\nConnection: close\r\n\r\nNot Found");
}

void handleClient(EthernetClient &c) {
  char line[96];
  line[0] = 0;
  size_t curLen = 0;
  bool first = true, done = false;
  uint32_t start = millis();

  while (c.connected() && !done && millis() - start < 500) {
    if (!c.available()) { delay(1); continue; }
    char ch = c.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (first) line[min(curLen, sizeof(line) - 1)] = 0;
      if (curLen == 0) done = true;      // baris kosong = akhir header
      first = false;
      curLen = 0;
      continue;
    }
    if (first && curLen < sizeof(line) - 1) line[curLen] = ch;
    curLen++;
  }

  if (strncmp(line, "GET /data", 9) == 0 && (line[9] == ' ' || line[9] == '?')) sendJson(c);
  else if (strncmp(line, "GET / ", 6) == 0)                                      sendPage(c);
  else                                                                           send404(c);

  c.flush();
  delay(1);
  c.stop();

#if LOG_HTTP
  logf("HTTP: %s", line);
#endif
}

// ================= SETUP =================
void setupEthernet() {
  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, LOW);
  delay(100);
  digitalWrite(ETH_RST, HIGH);
  delay(500);

  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);
  Ethernet.init(ETH_CS);
  Ethernet.begin(mac, ip, dnsServer, gateway, subnet);   // begin dulu, baru deteksi chip
  delay(500);

  if (Ethernet.hardwareStatus() != EthernetW5500) {
    logf("ERROR: W5500 tidak terdeteksi, Ethernet nonaktif (Modbus + Serial tetap jalan)");
    ethOK = false;
    return;
  }
  ethOK = true;

  uint32_t t = millis();
  while (Ethernet.linkStatus() != LinkON && millis() - t < 5000) delay(100);

  server.begin();
  logf("W5500 OK, IP %s, kabel %s",
       Ethernet.localIP().toString().c_str(),
       Ethernet.linkStatus() == LinkON ? "LINK ON" : "LINK OFF");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  logf("=== Power Meter PM53xx + W5500 (v6.0: bus 2 master, blok gabungan, carrier sense, validasi data) ===");

  initData(shared);
  dataMutex = xSemaphoreCreateMutex();

#if USE_DE_RE_PIN
  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);
#endif
  Serial2.setRxBufferSize(512);          // sebelum begin(); muat respons 112 register (229 byte)
  Serial2.begin(PM_BAUD, PM_CONFIG, RXD2, TXD2);

  setupEthernet();

  xTaskCreatePinnedToCore(modbusTask, "modbus", 8192, NULL, 1, NULL, 0);
}

// ================= LOOP (core 1) =================
void loop() {
  if (!ethOK) { delay(10); return; }

  // Semua akses W5500 hanya dari core 1 (SPI tidak thread-safe)
  static uint32_t tLink = 0;
  if (millis() - tLink >= 1000) {
    tLink = millis();
    EthernetLinkStatus link = Ethernet.linkStatus();
    linkOn = (link == LinkON);
    if (link != linkTerakhir) {
      linkTerakhir = link;
      logf(link == LinkON ? "[LINK] kabel TERHUBUNG" : "[LINK] kabel TERPUTUS");
    }
  }

  EthernetClient client = server.available();
  if (client) handleClient(client);
  else delay(1);
}
