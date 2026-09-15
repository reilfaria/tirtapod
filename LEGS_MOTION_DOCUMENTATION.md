# Dokumentasi Sistem Pergerakan Robot Tirtapod (legs.h)

## 📋 Ringkasan Umum
File `legs.h` adalah modul kontrol pergerakan untuk robot hexapod (6 kaki) yang menggunakan inverse kinematics untuk menghitung sudut servo. Robot menggunakan 18 servo (3 per kaki × 6 kaki) dan dikontrol melalui serial communication dengan SSC-32 servo controller.

---

## 🦾 Spesifikasi Fisik Kaki Robot

### Dimensi Segmen Kaki
```
Coxa (cx)   = 2.8 cm    (Ruas pertama dari pangkal)
Femur (fm)  = 5.5 cm    (Ruas tengah)
Tibia (tb)  = 7.4 cm    (Ruas terkait kaki)
```

### Posisi Referensi Servo (Pulse Width Modulation)
Setiap kaki memiliki 3 servo yang dikontrol dengan PWM (1000-2000 microsecond):
- **Servo 0-8**: Gamma (rotasi horizontal/base)
- **Servo 1-9**: Alpha (rotasi bahu)
- **Servo 2-10**: Beta (rotasi siku)

Posisi default (Point 0):
```
L01 = {1460, 590, 610}    // Kaki Kiri-1
L02 = {1550, 530, 480}    // Kaki Kiri-2
L03 = {1530, 550, 570}    // Kaki Kiri-3
R01 = {1540, 450, 435}    // Kaki Kanan-1
R02 = {1585, 450, 450}    // Kaki Kanan-2
R03 = {1570, 490, 560}    // Kaki Kanan-3
```

---

## 🔧 Sistem Inverse Kinematics

### Fungsi Perhitungan Sudut
Menggunakan geometri 3-link planar untuk menghitung sudut servo dari koordinat XYZ:

#### **getgamaL() / getgamaR()** — Sudut Rotasi Horizontal
```cpp
// Menghitung rotasi base (servo pertama)
float getgamaL(float x, float y, float z) {
    L1 = sqrt(x² + y²)                      // Jarak horizontal
    gama = atan(x/y) × 180/π                // Sudut dari Y-axis
    return -gama;                           // Negatif untuk kaki kiri
}
```
**Output**: Sudut dalam derajat untuk servo base

#### **getalphaL() / getalphaR()** — Sudut Servo Bahu
```cpp
// Menghitung sudut bahu menggunakan hukum cosinus
float getalphaL(float x, float y, float z) {
    L = sqrt((L1-cx)² + z²)                 // Distance ke ujung kaki
    alpha1 = acos(z/L) × 180/π              // Sudut vertikal
    alpha2 = acos(fm² + L² - tb²)           // Hukum cosinus
           / (2×fm×L) × 180/π
    alpha = alpha1 + alpha2
    return alpha;
}
```
**Output**: Sudut bahu (shoulder joint)

#### **getBetaL() / getBetaR()** — Sudut Servo Siku
```cpp
// Menghitung sudut siku dari hukum cosinus
float getBetaL(float x, float y, float z) {
    beta = acos((tb² + fm² - L²)/(2×tb×fm)) × 180/π
    return 180 - beta;
}
```
**Output**: Sudut siku (elbow joint)

---

## 🚶 Mode Pergerakan Robot

### 1. **Pergerakan Berjalan (Walking)**

#### **Forward / Forward6**
- **Deskripsi**: Robot bergerak maju menggunakan tripod gait (3 kaki bergerak sekaligus)
- **Frame Gerakan**:
  - Forward: 4 frame per siklus
  - Forward6: 6 frame per siklus (lebih halus)
- **Parameter Gerak**:
  ```
  kakiNaik = -4      // Ketinggian kaki saat diangkat (cm)
  kakiTurun = 0      // Ketinggian kaki saat di tanah
  travel = 3-4       // Jarak geser per step (cm)
  ```

**Pola Tripod Gait**:
```
Frame 0: Tripod-A (L1, L2, R3) diangkat dan maju
         Tripod-B (L3, R1, R2) tetap di tanah

Frame 1: Tripod-A turun, mulai menopang berat
Frame 2: Tripod-B diangkat dan maju
Frame 3: Semuanya kembali ke posisi awal
```

**Koordinat Pergerakan untuk Forward**:
```
Travel1 = [+4, -4]      // Gerakan X kaki depan
Travel2 = [0, 0]        // Gerakan X kaki tengah
Travel3 = [-4, +4]      // Gerakan X kaki belakang
```

#### **Backward / Backward6**
- **Deskripsi**: Robot bergerak mundur (reverse dari forward)
- **Perbedaan**: Urutan tripod terbalik, jarak gerakan negatif

---

### 2. **Pergerakan Rotasi (Rotation)**

#### **Rotate Left / Rotate Right**
- **Deskripsi**: Robot berputar pada tempatnya
- **Mekanisme**: Setiap kaki bergerak dalam radius melingkar
- **Pola Gerakan**:
  ```
  YTravel = [-Y2, -Y1]    // Gerakan Y (lateral) untuk rotasi
  Travel1 = [X1, X2]      // Gerakan X yang disesuaikan untuk rotasi
  Travel3 = [-X2, -X1]    // Gerakan berlawanan untuk torsi
  ```

**Koordinat Rotasi**:
```cpp
void rotate_coor() {
    X1 = sin(45°) × travelhalf + 2.876
    Y1 = sqrt(travelhalf² - sin(45°)² × travelhalf²) + 2.423
    X2 = X1 - sin(45°) × travel
    Y2 = Y1 - sin(45°) × travel
}
```
**Efek**: Rotasi ~45° per step

---

### 3. **Pergerakan Lateral (Shift)**

#### **Shift Left / Shift Right**
- **Deskripsi**: Robot bergerak ke samping
- **Mekanisme**: Semua kaki bergerak laterally sambil menopang berat
- **Parameter**:
  ```
  Travel1/2/3 = [4, 4]          // Jarak geser sama untuk semua
  YLTravel = [travelhalf, -travelhalf]   // Geser Y alternatif
  ```

**Pola Gerak**:
```
Frame 0: Tripod-A geser ke kiri, Tripod-B tetap
Frame 1: Tripod-A turun
Frame 2: Tripod-B geser ke kiri
Frame 3: Tripod-B turun, kembali ke awal
```

---

### 4. **Pose Khusus (Pointing)**

#### **Point Ready**
```cpp
L1 = {2.84 + C0L[0][0], -0.5 + C0L[0][1], 0 + C0L[0][2]}
// Robot siap dengan kaki membentuk posisi attack stance
```

#### **Point Forward / Point Backward**
```cpp
Point Forward:   L1={-1}, L2={-4}, L3={-7}     // Arah hadap
Point Backward:  L1={6}, L2={3}, L3={0}        // Arah mundur
```

---

### 5. **Pergerakan Khusus**

#### **Wave Hello** (Lambaian)
- **Frame**: 8 frame halus
- **Mekanisme**:
  - Kaki R1 diangkat dan diayunkan (kiri-kanan)
  - Kaki lain melakukan weight-shift untuk menjaga stabilitas
  - Shift = -1.5 (penyesuaian weight ke kiri)

**Sequence**:
```
0: Persiapan, R1 tetap di tanah
1: R1 diangkat setelah weight shift
2-6: R1 diayun-ayunkan (swing: ±2 cm)
7: R1 turun, robot kembali normal
```

#### **Dance** (Tarian)
- **Frame**: 8 frame dengan pattern halus
- **Gerakan**:
  - Swaying (goyang): ±1.5 cm lateral
  - Up-Down: ±1.5 cm vertikal
  - Tripod A dan B bergerak dengan fase berlawanan

**Pattern Dance**:
```
pat[] = [0, 0.5, 1, 0.5, 0, -0.5, -1, -0.5]  // Triangle wave
Tripod A: zA = pat[frame] × 1.5
Tripod B: zB = -pat[frame] × 1.5              // Kebalikan fase
```

#### **Sapu** (Menyapu)
- **Gerakan Khusus**: Kaki L1 melakukan gerakan menyapu
- **Posisi L1** (4 frame):
  ```
  0: Angkat (-4 Z)
  1: Turun (0 Z) - mulai nyapu
  2: Tarik (Y +5)
  3: Angkat kembali (-4 Z)
  ```
- **Kaki Lain**: Tetap di posisi READY

---

## 📊 Tabel Jenis Gerakan

| Gerakan | Alias | Frame | Pola | Kecepatan Default |
|---------|-------|-------|------|-------------------|
| Forward | eforward | 4 | Tripod | 140 ms |
| Forward (6 frame) | eforward6 | 6 | Tripod | 140 ms |
| Backward | ebackward | 4 | Tripod | 140 ms |
| Backward (6 frame) | ebackward6 | 6 | Tripod | 140 ms |
| Rotate Left | erotate_left | 4 | Radial | 140 ms |
| Rotate Left (6) | erotate6_left | 6 | Radial | 140 ms |
| Rotate Right | erotate_right | 4 | Radial | 140 ms |
| Rotate Right (6) | erotate6_right | 6 | Radial | 140 ms |
| Shift Left | eshift_left | 4 | Lateral | 140 ms |
| Shift Left (6) | eshift6_left | 6 | Lateral | 140 ms |
| Shift Right | eshift_right | 4 | Lateral | 140 ms |
| Shift Right (6) | eshift6_right | 6 | Lateral | 140 ms |
| Wave Hello | ewave_hello | 8 | Kinematis | 250 ms |
| Dance | edance | 8 | Dinamis | 250 ms |
| Sapu (Sweep) | esapu | 4 | Linear | 300 ms |

---

## 🎯 Fungsi Wrapper Pergerakan

Wrapper functions di bagian akhir file menyediakan interface tingkat tinggi:

### Pergerakan Standar
```cpp
void forward()              // Maju normal
void forward_1cm()          // Maju 1 cm
void forward_low()          // Maju rendah (kakiNaik=-1)
void forward6()             // Maju dengan 6 frame halus
```

### Pergerakan Rotasi
```cpp
void rotate_left()          // Putar kiri standar
void rotate_left_fast()     // Putar kiri cepat (travel=5)
void rotate_left_slow()     // Putar kiri lambat (travel=0.5)
void rotate_left_low()      // Putar kiri rendah
```

### Pergerakan Lateral
```cpp
void shift_left()           // Geser kiri standar
void shift_left_1cm()       // Geser kiri 1 cm
void shift_left_fast()      // Geser kiri cepat
void shift6_left_bridge()   // Geser kiri untuk jembatan (travel=6, width=2)
```

### Pose Khusus
```cpp
void point0state()          // Reset ke pose awal
void point_ready()          // Posisi siap serang
void point_rotate_left()    // Pose pointing putar kiri
void wave_hello()           // Lambaian
void dance()                // Tarian
void sapu()                 // Menyapu
```

---

## ⚙️ Konfigurasi Parameter

### Parameter Global Pergerakan
```cpp
// Di function seperti forward(), rotate_left(), dst:
kakiNaik = -4              // Ketinggian angkat kaki (cm)
kakiTurun = 0              // Ketinggian kaki di tanah (cm)
travel = 3                 // Jarak pergerakan per frame (cm)
travelhalf = travel / 2    // Setengah travel
width = 4                  // Lebar gerakan lateral (untuk shift6)
walkspeed = 140-300 ms     // Delay antar frame
```

### Variasi Konfigurasi
```cpp
// Maju lambat dan rendah
forward_low()      → kakiNaik=-1, travel=2

// Putar cepat
rotate_left_fast() → travel=5, rotate_coor() dipanggil

// Geser untuk jembatan (lebar)
shift6_left_bridge() → travel=6, width=2
```

---

## 🔄 Alur Eksekusi Gerakan

### Step-by-Step Flow untuk Forward Motion

```
1. User memanggil forward()
   ↓
2. Set parameter: kakiNaik=-4, travel=4, travelhalf=2
   ↓
3. Call movetype(eforward)
   ↓
4. Loop dalam movetype():
   - Cek waktu: millis() - state_lastMoveRecord >= walkspeed
   - Panggil forward_step()
   ↓
5. forward_step() switch case berdasarkan state_nextStep:
   - Case 0: Serial2.print(forward_ssc(0))  → Frame 0 kirim ke servo
   - Case 1: Serial2.print(forward_ssc(1))  → Frame 1
   - Case 2: Serial2.print(forward_ssc(2))  → Frame 2
   - Case 3: Serial2.print(forward_ssc(3))  → Frame 3, reset ke case 0
   ↓
6. forward_ssc(frame_number) menghitung:
   - Koordinat XYZ untuk setiap kaki
   - Inverse kinematics → sudut servo
   - Gabung string perintah servo (format: "#23 P1234 #22 P5678...")
   ↓
7. Kirim string ke Serial2 (SSC-32 servo controller)
   ↓
8. SSC-32 gerakkan servo sesuai perintah
   ↓
9. Kembali ke step 4, repeat sampai user stop
```

---

## 📡 Format Komunikasi Serial

### Format Perintah SSC-32
```
#<servo_id> P<pulsewidth> T<time> \r
```

**Contoh**:
```
#23 P1500 #22 P1400 #21 P1350 #19 P1600 #18 P1200 #17 P1250 T140 \r
│   │      │   │      │   │      │   │      │   │      │   │      │  │
└───┘ └─────┘ └───┘ └─────┘ └───┘ └─────┘ └───┘ └─────┘ └───┘ └─────┘ └──┘
  │     │        │     │        │     │        │     │        │     │      │
  │     └PWM─────┴─────┴─PWM────┴─────┴─PWM────┴─────┴─PWM────┴─────┘      │
  └─Servo 23     Servo 22      Servo 21      Servo 19     Servo 18     Time (ms)
```

**Mapping Servo ID**:
- **L1** (Kaki Kiri-1): #23 (gamma), #22 (alpha), #21 (beta)
- **L2** (Kaki Kiri-2): #19 (gamma), #18 (alpha), #17 (beta)
- **L3** (Kaki Kiri-3): #15 (gamma), #14 (alpha), #13 (beta)
- **R1** (Kaki Kanan-1): #0 (gamma), #1 (alpha), #2 (beta)
- **R2** (Kaki Kanan-2): #4 (gamma), #5 (alpha), #6 (beta)
- **R3** (Kaki Kanan-3): #8 (gamma), #9 (alpha), #10 (beta)

---

## 🐛 Debugging & Tuning

### Parameter Penyesuaian

1. **Stabilitas Miring** (weight distribution):
   ```cpp
   // Untuk wave_hello, jika jatuh ke satu arah:
   float shift = -1.5;  // Ubah tanda jika perlu
   ```

2. **Kecepatan Gerakan**:
   ```cpp
   int walkspeed = 140;  // Kurangi untuk lebih cepat, tambah untuk lebih lambat
   ```

3. **Ketinggian Langkah**:
   ```cpp
   kakiNaik = -4;    // Lebih negative = lebih tinggi
   kakiTurun = 0;    // Ubah untuk langkah yang lebih pendek/panjang
   ```

4. **Jarak Pergerakan**:
   ```cpp
   travel = 3;       // Lebih besar = langkah lebih panjang
   ```

---

## 📝 Catatan Teknis

### Inverse Kinematics Library
Menggunakan fungsi trigonometri standar Arduino:
- `sqrt()` — Akar kuadrat
- `atan()` — Arctangent (radian output)
- `acos()` — Arc cosine (radian output)
- `sin()`, `cos()` — Trigonometri
- Konversi radian ke derajat: `× 180/π`

### Koordinat Sistem
- **X**: Maju-Mundur (forward-backward)
- **Y**: Samping (lateral)
- **Z**: Vertikal (up-down)
- Titik origin di center body

### Servo PWM Range
- Min: 1000 μs
- Max: 2000 μs
- Mid: 1500 μs (netral)
- Resolution: 1 μs per unit

---

## 🎓 Kesimpulan

`legs.h` mengimplementasikan sistem pergerakan hexapod yang komprehensif dengan:
1. **Inverse kinematics** untuk konversi XYZ → sudut servo
2. **Tripod gait** untuk stabilitas dan efisiensi
3. **Multiple motion primitives** (berjalan, rotasi, lateral, pose khusus)
4. **Serial communication** ke SSC-32 servo controller
5. **Flexible parameter tuning** untuk berbagai kondisi gerakan

Sistem ini memungkinkan robot bergerak dengan cara yang natural dan stabil di berbagai medan dan situasi.
