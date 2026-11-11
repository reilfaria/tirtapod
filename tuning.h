// ============================================================
//  tuning.h - atur ruangan start, kompas, dan parameter jarak/waktu
//  lewat Serial Monitor TANPA upload ulang. Tersimpan di flash.
//
//  Serial Monitor: 38400 baud, line ending "Newline".
//  Perintah hanya diproses saat robot STANDBY (belum start).
//  Ketik "help" untuk daftar perintah.
// ============================================================
#include <EEPROM.h>
#include <ctype.h>

namespace tuning {

// ---------- Daftar parameter (arena KANAN) ----------
enum ParId {
  R1_KIRI_BUKA, R1_MUNDUR_MS, R1_MUNDUR2_MS,
  R2_DEPAN_JAUH,
  R3_SHIFT_MS, R3_DEPAN_MIN, R3_DEPAN_MAX, R3_KANAN_MIN, R3_KANAN_MAX,
  R3_ROTATE_MS, R3_MUNDUR_MS, R3_MAJU_MS, R3_GESER_MS, R3_KANAN_R4,
  R4_DEPAN, R4_MUNDUR_MS, R4_DEPAN_MIN, R4_DEPAN_MAX, R4_KANAN_MIN, R4_KANAN_MAX, R4_ROTATE_MS,
  R5_KANAN, R5_DEPAN, R5_GESER_MS, R5_PUING_DEPAN,
  R6_KANAN, R6_MUNDUR_CAM_MS, R6_GESER_H4_MS, R6_MUNDUR_CAM4_MS, R6_MUNDUR_MS,
  R6_STEP_AWAL, R6_MUNDUR_H3_MS,
  R6_TARUH_DEPAN_MIN, R6_TARUH_DEPAN_MAX, R6_TARUH_KIRI_MIN, R6_TARUH_KIRI_MAX,
  R6_TARUH_ROTATE_MS, R6_TARUH_MUNDUR_MS, R6_TARUH_MUNDUR2_MS,
  R6_BAWA_DEPAN_MIN, R6_BAWA_DEPAN_MAX,
  R6_R7_DEPAN_MIN, R6_R7_DEPAN_MAX, R6_R7_GESER_MS,
  R7_KELUAR_DEPAN, R7_KELUAR_MS, R7_DEPAN_DEKAT, R7_DEPAN_JAUH, R7_DEPAN_MIN, R7_DEPAN_MAX,
  R8_DEPAN_MIN, R8_DEPAN_MAX,
  R9_GESER_MS, R9_KIRI_MIN, R9_KIRI_MAX, R9_DEPAN_MIN, R9_DEPAN_MAX,
  N_PAR
};

struct ParDef { const char* nama; int def; };

// nama (yang diketik di Serial) dan nilai default (= nilai lama di kode)
const ParDef parDef[N_PAR] = {
  {"r1_kiri_buka", 550}, {"r1_mundur_ms", 2000}, {"r1_mundur2_ms", 2000},
  {"r2_depan_jauh", 500},
  {"r3_shift_ms", 2000}, {"r3_depan_min", 270}, {"r3_depan_max", 290}, {"r3_kanan_min", 130}, {"r3_kanan_max", 150},
  {"r3_rotate_ms", 1200}, {"r3_mundur_ms", 2000}, {"r3_maju_ms", 2000}, {"r3_geser_ms", 2000}, {"r3_kanan_r4", 130},
  {"r4_depan", 510}, {"r4_mundur_ms", 2000}, {"r4_depan_min", 220}, {"r4_depan_max", 250}, {"r4_kanan_min", 140}, {"r4_kanan_max", 170}, {"r4_rotate_ms", 1200},
  {"r5_kanan", 320}, {"r5_depan", 100}, {"r5_geser_ms", 4000}, {"r5_puing_depan", 460},
  {"r6_kanan", 380}, {"r6_mundur_cam_ms", 1000}, {"r6_geser_h4_ms", 4000}, {"r6_mundur_cam4_ms", 2000}, {"r6_mundur_ms", 3000},
  {"r6_step_awal", 0}, {"r6_mundur_h3_ms", 5000},
  {"r6_taruh_depan_min", 300}, {"r6_taruh_depan_max", 320}, {"r6_taruh_kiri_min", 140}, {"r6_taruh_kiri_max", 160},
  {"r6_taruh_rotate_ms", 1200}, {"r6_taruh_mundur_ms", 2000}, {"r6_taruh_mundur2_ms", 3000},
  {"r6_bawa_depan_min", 300}, {"r6_bawa_depan_max", 320},
  {"r6_r7_depan_min", 400}, {"r6_r7_depan_max", 460}, {"r6_r7_geser_ms", 7000},
  {"r7_keluar_depan", 350}, {"r7_keluar_ms", 2000}, {"r7_depan_dekat", 150}, {"r7_depan_jauh", 200}, {"r7_depan_min", 230}, {"r7_depan_max", 240},
  {"r8_depan_min", 400}, {"r8_depan_max", 375},
  {"r9_geser_ms", 7000}, {"r9_kiri_min", 175}, {"r9_kiri_max", 195}, {"r9_depan_min", 250}, {"r9_depan_max", 270},
};

int par[N_PAR];
bool logStandby = true;   // print kompas+TOF terus-menerus saat standby

// ---------- Titik kompas (arena KANAN) ----------
// Setiap pemanggilan radius() di arena kanan punya titik sendiri:
// target heading, mode (F/R/L) dan toleransi masing-masing.
enum KId {
  K_HOME,
  K_R1_HUMAN, K_R1_AMBIL,
  K_R2_GESER, K_R2_KE_R3,
  K_R3_LURUS, K_R3_KE_R4,
  K_R4_KAMERA, K_R4_BALIK, K_R4_KE_R5,
  K_R5_LURUS, K_R5_PUING,
  K_R6_H3, K_R6_AMBIL_H3, K_R6_TARUH_ARAH1, K_R6_TARUH_ARAH2, K_R6_H4, K_R6_AMBIL_H4, K_R6_BAWA_ARAH1, K_R6_KE_R7,
  K_R7_TANGGA, K_R7_TARUH,
  K_R8_HUMAN, K_R8_KE_R9,
  K_R9_LURUS,
  N_K
};

struct TitikKompas { int target; char mode; int tol; };
struct KDef { const char* nama; const char* ket; bool hanyaTarget; };

const KDef kDef[N_K] = {
  {"r0_k_home",      "home: putar sebelum masuk R1", false},
  {"r1_k_human",     "R1: arah ke korban 1 (sebelum mundur)", false},
  {"r1_k_ambil",     "R1: koreksi arah di gethuman_low", true},
  {"r2_k_geser",     "R2: jaga arah saat geser kanan", false},
  {"r2_k_ke_r3",     "R2: putar kanan ke arah R3", false},
  {"r3_k_lurus",     "R3: luruskan arah awal R3", false},
  {"r3_k_ke_r4",     "R3: putar kiri ke arah R4", false},
  {"r4_k_kamera",    "R4: putar kiri hadap korban 2", false},
  {"r4_k_balik",     "R4: putar kanan setelah ambil korban", false},
  {"r4_k_ke_r5",     "R4: putar kiri ke arah R5", false},
  {"r5_k_lurus",     "R5: luruskan arah setelah geser", false},
  {"r5_k_puing",     "R5: arah ke puing sebelum R6", false},
  {"r6_k_h3",        "R6: hadap korban 3 (sebelum kamera)", false},
  {"r6_k_ambil_h3",  "R6: koreksi arah gethuman6puing korban 3", true},
  {"r6_k_taruh_arah1", "R6: arah sebelum maju ke tempat taruh korban 3", false},
  {"r6_k_taruh_arah2", "R6: arah akhir sebelum taruh korban 3 (+ setelah taruh)", false},
  {"r6_k_h4",        "R6: hadap korban 4 (sebelum kamera)", false},
  {"r6_k_ambil_h4",  "R6: koreksi arah gethuman6puing korban 4", true},
  {"r6_k_bawa_arah1", "R6: arah setelah ambil korban 4 (sebelum maju)", false},
  {"r6_k_ke_r7",     "R6: arah sebelum mundur & geser ke R7", false},
  {"r7_k_tangga",    "R7: jaga arah saat naik tangga", false},
  {"r7_k_taruh",     "R7: arah sebelum taruh korban", false},
  {"r8_k_human",     "R8: hadap korban 5", false},
  {"r8_k_ke_r9",     "R8: putar kiri ke arah R9", false},
  {"r9_k_lurus",     "R9: luruskan arah setelah jembatan", false},
};

TitikKompas kp[N_K];

// default = nilai lama di kode (dibaca dari array cpos di .ino)
void isiDefaultKompas() {
  // nilai hasil tuning (dari "show" terakhir)
  kp[K_HOME]        = { 278, 'F', 50 };
  kp[K_R1_HUMAN]    = { 275, 'F', 15 };
  kp[K_R1_AMBIL]    = { 275, 'F', 15 };
  kp[K_R2_GESER]    = { 270, 'F', 25 };
  kp[K_R2_KE_R3]    = { 182, 'R', 15 };
  kp[K_R3_LURUS]    = { 182, 'F', 15 };
  kp[K_R3_KE_R4]    = { 260, 'F', 15 };
  kp[K_R4_KAMERA]   = { 45,  'L', 15 };
  kp[K_R4_BALIK]    = { 300, 'R', 15 };
  kp[K_R4_KE_R5]    = { 310, 'F', 15 };
  kp[K_R5_LURUS]    = { 285, 'F', 15 };
  kp[K_R5_PUING]    = { 313, 'F', 15 };
  kp[K_R6_H3]       = { 310, 'F', 15 };
  kp[K_R6_AMBIL_H3] = { 310, 'F', 15 };
  kp[K_R6_TARUH_ARAH1] = { 42, 'F', 5 };   // baru, belum di-tuning
  kp[K_R6_TARUH_ARAH2] = { 35, 'F', 5 };   // baru, belum di-tuning
  kp[K_R6_H4]       = { 310, 'L', 10 };
  kp[K_R6_AMBIL_H4] = { 135, 'F', 15 };
  kp[K_R6_BAWA_ARAH1] = { 42, 'F', 5 };    // baru, belum di-tuning
  kp[K_R6_KE_R7]    = { 122, 'F', 2 };
  kp[K_R7_TANGGA]   = { 110, 'F', 20 };
  kp[K_R7_TARUH]    = { 20,  'R', 5 };
  kp[K_R8_HUMAN]    = { 357, 'F', 15 };
  kp[K_R8_KE_R9]    = { 25,  'F', 5 };
  kp[K_R9_LURUS]    = { 13,  'F', 15 };
}

// ---------- Data yang disimpan ke flash ----------
const uint32_t MAGIC = 0x54504433;  // "TPD3" (ganti kalau format berubah)

struct Simpan {
  uint32_t magic;
  int nPar;
  int nK;
  int startRoom;
  TitikKompas kp[N_K];
  int par[N_PAR];
};

Simpan bawaan;  // nilai default dari kode (untuk perintah "reset")

void ambilDariGlobal(Simpan& s) {
  s.magic = MAGIC;
  s.nPar = N_PAR;
  s.nK = N_K;
  s.startRoom = TEST_START_ROOM;
  for (int i = 0; i < N_K; i++) s.kp[i] = kp[i];
  for (int i = 0; i < N_PAR; i++) s.par[i] = par[i];
}

void terapkanKeGlobal(const Simpan& s) {
  TEST_START_ROOM = s.startRoom;
  for (int i = 0; i < N_K; i++) kp[i] = s.kp[i];
  for (int i = 0; i < N_PAR; i++) par[i] = s.par[i];
}

void simpan() {
  Simpan s;
  ambilDariGlobal(s);
  EEPROM.put(0, s);
  EEPROM.commit();
  Serial.println("[tuning] TERSIMPAN ke flash");
}

// Panggil di setup() SEBELUM applyTestBypass()
void muat() {
  for (int i = 0; i < N_PAR; i++) par[i] = parDef[i].def;
  isiDefaultKompas();
  ambilDariGlobal(bawaan);  // catat default kode

  EEPROM.begin(1024);
  Simpan s;
  EEPROM.get(0, s);
  if (s.magic == MAGIC && s.nPar == N_PAR && s.nK == N_K) {
    terapkanKeGlobal(s);
    Serial.println("[tuning] setting dimuat dari flash");
  } else {
    Serial.println("[tuning] belum ada setting tersimpan, pakai default kode");
  }
}

// ---------- Tampilan ----------
int norm360(int a) { a %= 360; if (a < 0) a += 360; return a; }

void cetakKompas(int i) {
  const TitikKompas& k = kp[i];
  Serial.print(kDef[i].nama);
  Serial.print("  target=");
  Serial.print(k.target);
  if (kDef[i].hanyaTarget) {
    Serial.print("  (hanya target, mode/tol tetap F 15 di fungsi gethuman)");
  } else {
    int lo, hi;
    if (k.mode == 'R')      { lo = k.target;             hi = k.target + 2 * k.tol; }
    else if (k.mode == 'L') { lo = k.target - 2 * k.tol; hi = k.target; }
    else                    { lo = k.target - k.tol;     hi = k.target + k.tol; }
    Serial.print("  mode=");
    Serial.print(k.mode);
    Serial.print("  tol=");
    Serial.print(k.tol);
    Serial.print("  -> pas kalau heading ");
    Serial.print(norm360(lo));
    Serial.print("..");
    Serial.print(norm360(hi));
  }
  Serial.print("   | ");
  Serial.println(kDef[i].ket);
}

void cetakPar(int i) {
  Serial.print(parDef[i].nama);
  Serial.print(" = ");
  Serial.print(par[i]);
  if (par[i] != parDef[i].def) {
    Serial.print("   (default ");
    Serial.print(parDef[i].def);
    Serial.print(")");
  }
  Serial.println();
}

bool cocok(const char* nama, const String& filter) {
  return filter.length() == 0 || String(nama).startsWith(filter + "_");
}

// filter kosong = semua; "r3" = ruangan 3 saja; "k" = semua kompas
void cetak(const String& filter) {
  Serial.println();
  Serial.println("===== SETTING SEKARANG =====");
  if (filter.length() == 0) Serial.println(String() + "room (start) = " + TEST_START_ROOM);
  Serial.println("--- kompas ---");
  for (int i = 0; i < N_K; i++)
    if (filter == "k" || cocok(kDef[i].nama, filter)) cetakKompas(i);
  if (filter != "k") {
    Serial.println("--- jarak & waktu ---");
    for (int i = 0; i < N_PAR; i++)
      if (cocok(parDef[i].nama, filter)) cetakPar(i);
  }
  Serial.println("============================");
}

void bantuan() {
  Serial.println();
  Serial.println("===== PERINTAH TUNING (saat standby) =====");
  Serial.println("show                  semua setting");
  Serial.println("show r3               setting ruangan 3 saja (kompas + jarak)");
  Serial.println("show k                semua titik kompas");
  Serial.println("room N                ruangan start (0=home). Otomatis save + reboot");
  Serial.println("--- kompas (nama diawali rX_k_) ---");
  Serial.println("r3_k_lurus            lihat titik kompas");
  Serial.println("r3_k_lurus 185        ganti target");
  Serial.println("r3_k_lurus 185 F 10   ganti target, mode, toleransi sekaligus");
  Serial.println("r3_k_lurus mode R     ganti mode saja (F/R/L)");
  Serial.println("r3_k_lurus tol 10     ganti toleransi saja");
  Serial.println("catat r3_k_lurus      target = heading SEKARANG");
  Serial.println("  F = target-tol..target+tol");
  Serial.println("  R = target..target+2*tol    L = target-2*tol..target");
  Serial.println("--- jarak & waktu ---");
  Serial.println("r3_depan_min 260      set parameter");
  Serial.println("r3_depan_min          lihat parameter");
  Serial.println("--- lain-lain ---");
  Serial.println("hd / tof              lihat heading / semua TOF sekali");
  Serial.println("log                   nyala/matikan print standby");
  Serial.println("save                  simpan ke flash (WAJIB biar tidak hilang)");
  Serial.println("reset                 kembali ke default kode (lalu save)");
  Serial.println("p                     start/stop robot");
  Serial.println("r                     reboot");
  Serial.println("==========================================");
}

// ---------- Baca baris dari Serial ----------
bool bacaBaris(String& out) {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        out = buf;
        buf = "";
        out.trim();
        return true;
      }
    } else if (buf.length() < 60) {
      buf += c;
    }
  }
  return false;
}

// ambil kata ke-n (mulai 0) dari baris, dipisah spasi
String kata(const String& s, int n) {
  int mulai = 0;
  for (int k = 0; k <= n; k++) {
    while (mulai < (int)s.length() && s.charAt(mulai) == ' ') mulai++;
    int akhir = s.indexOf(' ', mulai);
    if (akhir < 0) akhir = s.length();
    if (k == n) return s.substring(mulai, akhir);
    mulai = akhir;
  }
  return "";
}

void perintah(String baris) {
  baris.toLowerCase();
  String k0 = kata(baris, 0);
  String k1 = kata(baris, 1);
  String k2 = kata(baris, 2);
  int hd = (int)compass::heading();

  if (k0 == "help") { bantuan(); return; }
  if (k0 == "show") { cetak(k1); return; }
  if (k0 == "save") { simpan(); return; }
  if (k0 == "log")  { logStandby = !logStandby; Serial.println(logStandby ? "log ON" : "log OFF"); return; }
  if (k0 == "hd")   { Serial.println(String() + "heading = " + compass::heading()); return; }
  if (k0 == "tof") {
    Serial.println(String() + "kanan=" + TOF::getkanan() + " skanan=" + TOF::getserongkanan() +
                   " depan=" + TOF::getdepan() + " skiri=" + TOF::getserongkiri() +
                   " kiri=" + TOF::getkiri() + " capit=" + TOF::getcapit());
    return;
  }
  if (k0 == "reset") {
    terapkanKeGlobal(bawaan);
    Serial.println("Kembali ke default kode. Ketik save untuk simpan.");
    return;
  }
  if (k0 == "room") {
    int n = k1.toInt();
    if (k1.length() == 0 || n < 0 || n > 9) { Serial.println("room harus 0..9"); return; }
    TEST_START_ROOM = n;
    simpan();
    Serial.println(String() + "Start dari ruangan " + n + ". Reboot...");
    delay(300);
    rp2040.reboot();
    return;
  }
  if (k0 == "catat") {
    for (int i = 0; i < N_K; i++) {
      if (k1 == kDef[i].nama) {
        kp[i].target = hd;
        cetakKompas(i);
        Serial.println("(ketik save untuk simpan)");
        return;
      }
    }
    Serial.println("contoh: catat r3_k_lurus   (lihat nama dengan: show k)");
    return;
  }

  for (int i = 0; i < N_K; i++) {
    if (k0 == kDef[i].nama) {
      String k3 = kata(baris, 3);
      if (k1.length() == 0) {
        cetakKompas(i);
        return;
      }
      if (k1 == "mode" || k1 == "tol") {
        if (kDef[i].hanyaTarget) {
          Serial.println("Titik ini hanya pakai target (mode/tol diatur di fungsi gethuman).");
          return;
        }
        if (k1 == "mode") {
          char m = k2.length() > 0 ? toupper(k2.charAt(0)) : ' ';
          if (m != 'F' && m != 'R' && m != 'L') { Serial.println("mode harus F, R, atau L"); return; }
          kp[i].mode = m;
        } else {
          int t = k2.toInt();
          if (t < 1 || t > 90) { Serial.println("tol harus 1..90"); return; }
          kp[i].tol = t;
        }
      } else {
        kp[i].target = norm360(k1.toInt());
        if (k2.length() > 0) {
          char m = toupper(k2.charAt(0));
          if (m == 'F' || m == 'R' || m == 'L') kp[i].mode = m;
          else { Serial.println("mode harus F, R, atau L"); return; }
        }
        if (k3.length() > 0) {
          int t = k3.toInt();
          if (t >= 1 && t <= 90) kp[i].tol = t;
          else { Serial.println("tol harus 1..90"); return; }
        }
      }
      cetakKompas(i);
      Serial.println("(ketik save untuk simpan)");
      return;
    }
  }

  for (int i = 0; i < N_PAR; i++) {
    if (k0 == parDef[i].nama) {
      if (k1.length() > 0) {
        par[i] = k1.toInt();
        Serial.println(String() + parDef[i].nama + " = " + par[i] + "   (ketik save untuk simpan)");
      } else {
        Serial.println(String() + parDef[i].nama + " = " + par[i]);
      }
      return;
    }
  }

  Serial.println("Perintah tidak dikenal. Ketik help.");
}

}  // namespace tuning
