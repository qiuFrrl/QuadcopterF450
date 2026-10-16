# Bab 1: Arsitektur & Ikhtisar Sistem

Dokumen ini menjelaskan rancangan umum sistem, spesifikasi perangkat keras (*hardware*), konfigurasi fisik frame F450 Quad-X, serta skema integrasi antara **Glove Controller** dan **Quadcopter F450**.

---

## 1.1 Filosofi Desain Sistem

Sistem ini dirancang dengan 4 pilar rekayasa utama:

1. **Kendali Satu Langkah Terukur (*Step-by-Step Gesture Control*)**  
   Pengendalian drone berbasis gestur tangan sering kali rentan terhadap kesalahan manusia (*human error*), tremor tangan, dan kelelahan otot pilot. Jika sudut kemiringan tangan langsung diterjemahkan secara kontinu (*continuous streaming*), pergerakan tangan yang tidak disengaja dapat menyebabkan tabrakan (*crash*).  
   Sistem ini menerapkan paradigma **Langkah Diskret**:
   $$\text{Tekan READY} \longrightarrow \text{Sampling 250 ms} \longrightarrow \text{Manuver Terukur 1000 ms} \longrightarrow \text{Auto-Level Netral } (0^\circ)$$

2. **Pemisahan Asimetris Multi-Inti (*Asymmetric Multiprocessing / AMP*)**  
   ESP32 pada drone memiliki dua inti CPU fisik 32-bit (Core 0 dan Core 1).
   - **Core 0** didedikasikan penuh untuk menangani protokol jaringan ESP-NOW, I/O serial, buffer komunikasi, dan telemetri.
   - **Core 1** didedikasikan murni untuk loop kendali penerbangan waktu nyata (*real-time flight control loop*) pada frekuensi $250\text{ Hz}$ ($4.000\ \mu\text{s}$) tanpa terpengaruh beban radio WiFi.

3. **Prinsip Keselamatan Utama (*Safety & Fail-Safe First*)**  
   Setiap kegagalan (kehilangan sinyal nirkabel, timeout bus sensor I2C, atau kemiringan drone di luar batas wajar) ditangani oleh mekanisme failsafe lokal di Core 1 tanpa bergantung pada perintah eksternal.

4. **Koneksi Radio Latensi Rendah & Deterministik**  
   Komunikasi nirkabel menggunakan protokol langsung **ESP-NOW** pada frekuensi 2.4 GHz tanpa overhead protokol TCP/IP atau perantara router. Seluruh mode penghematan daya (*power-save sleep*) dinonaktifkan untuk memastikan latensi paket $< 3\text{ ms}$.

---

## 1.2 Spesifikasi Perangkat Keras (Hardware)

### A. Quadcopter F450 (Wahana Terbang)
| Komponen | Spesifikasi / Tipe | Catatan Teknis |
|---|---|---|
| **Flight Controller MCU** | ESP32-WROOM-32 (DevKit V1) | Dual-Core Tensilica Xtensa LX6 @ 240 MHz, 520 KB SRAM |
| **Sistem Operasi** | FreeRTOS (ESP-IDF / Arduino) | Multitasking preemptive antar Core 0 & Core 1 |
| **Sensor IMU** | InvenSense MPU-6050 (6-DOF) | Komunikasi I2C Fast-Mode 400 kHz, rentang $\pm 500^\circ/\text{s}$ & $\pm 8\text{g}$ |
| **Electronic Speed Controller (ESC)** | 4x SimonK / BLHeli 30A Opto/UBEC | Frekuensi sinyal kendali PWM hardware 500 Hz (LEDC) |
| **Motor Brushless** | 4x A2212 1000KV | Konfigurasi Quad-X, kecepatan putar $\approx 11.100\text{ RPM}$ @ 3S |
| **Baling-Baling (Propeller)** | 1045 (10x4.5 inch) | 2x CW (Clockwise) & 2x CCW (Counter-Clockwise) |
| **Baterai Utama** | LiPo 3S 11.1V 2200–3300 mAh 25C+ | Sumber daya daya tinggi melalui Power Distribution Board (PDB) |
| **Regulator Tegangan** | Step-down Buck Converter / BEC 5V 3A | Suplai daya bersih 5V ke ESP32 dan MPU-6050 |

### B. Glove Controller (Pengontrol Sarung Tangan)
| Komponen | Spesifikasi / Tipe | Catatan Teknis |
|---|---|---|
| **MCU Pengontrol** | ESP32-C3 SuperMini | Single-Core 32-bit RISC-V @ 160 MHz, WiFi & BLE |
| **Sensor IMU Sarung Tangan** | InvenSense MPU-6050 | Membaca kemiringan pitch/roll pergelangan tangan pilot |
| **Tombol Navigasi** | 3x Tactile Push Button | Tombol ARM/LAND (GPIO1), EMERGENCY (GPIO4), READY (GPIO10) |
| **Indikator Visual** | 2x LED (Merah & Hijau) | Menampilkan status koneksi, arming, status gesture, dan error |
| **Catu Daya** | Baterai Li-Ion 1S 3.7V / Power Bank mini | Dilengkapi modul charging TP4056 atau regulator 3.3V internal |

---

## 1.3 Konfigurasi Frame F450 Quad-X

Konfigurasi fisik motor menggunakan pola **Quad-X**. Penomoran motor, orientasi putaran, dan penetapan GPIO mikrokontroler adalah sebagai berikut:

```text
                  DEPAN (HIDUNG DRONE)
                         ▲
                         │
             [M4] FL     │     [M1] FR
            (Putar: CW)  │   (Putar: CCW)
             GPIO 27     │     GPIO 25
                   \     │     /
                    \    │    /
                     \───────/
                     │ ESP32 │
                     │  FC   │
                     /───────\
                    /    │    \
                   /     │     \
             [M3] RL     │     [M2] RR
            (Putar: CCW) │   (Putar: CW)
             GPIO 12     │     GPIO 33
                         │
```

### Tabel Rincian Motor & Arah Putaran:
| Motor | Posisi | Pin ESP32 | Arah Putaran | Tipe Baling-baling | Peran Dinamika |
|:---:|:---:|:---:|:---:|:---:|:---|
| **M1 (FR)** | Depan Kanan | GPIO 25 | **CCW** (Berlawanan jarum jam) | 1045 Normal (CCW) | Menekan hidung kanan ke bawah |
| **M2 (RR)** | Belakang Kanan| GPIO 33 | **CW** (Searah jarum jam) | 1045R (CW) | Menekan ekor kanan ke bawah |
| **M3 (RL)** | Belakang Kiri | GPIO 12 | **CCW** (Berlawanan jarum jam) | 1045 Normal (CCW) | Menekan ekor kiri ke bawah |
| **M4 (FL)** | Depan Kiri | GPIO 27 | **CW** (Searah jarum jam) | 1045R (CW) | Menekan hidung kiri ke bawah |

> [!WARNING]
> **Catatan Penting GPIO 12 (MTDI Strapping Pin pada ESP32):**  
> Pin GPIO 12 adalah *strapping pin* tegangan flash SPI internal saat proses boot. Jika saat menyalakan drone, ESC menarik pin ini ke logika HIGH (akibat pull-up internal ESC), ESP32 bisa gagal boot atau mengalami *boot loop*. Pastikan koneksi aman, atau jika terjadi kegagalan boot acak, alihkan pin M3 ke GPIO 14.

---

## 1.4 Pemetaan Pin & Skema Pengkabelan (Wiring)

### A. Pengkabelan ESP32 Quadcopter F450
```text
ESP32 DevKit V1                 Komponen Eksternal
┌────────────────┐              ┌───────────────────────────┐
│        GPIO 21 ├──────────────┤ SDA (MPU-6050)            │
│        GPIO 22 ├──────────────┤ SCL (MPU-6050)            │
│            GND ├──────────────┤ GND (MPU-6050 & Semua ESC)│
│             5V ├──────────────┤ Output 5V BEC / Regulator │
│        GPIO 25 ├──────────────┤ Sinyal ESC Motor 1 (FR)   │
│        GPIO 33 ├──────────────┤ Sinyal ESC Motor 2 (RR)   │
│        GPIO 12 ├──────────────┤ Sinyal ESC Motor 3 (RL)   │
│        GPIO 27 ├──────────────┤ Sinyal ESC Motor 4 (FL)   │
└────────────────┘              └───────────────────────────┘
```

### B. Pengkabelan ESP32-C3 Glove Controller
```text
ESP32-C3 SuperMini              Komponen Eksternal
┌────────────────┐              ┌───────────────────────────┐
│         GPIO 8 ├──────────────┤ SDA (MPU-6050 Sarung Tangan)
│         GPIO 9 ├──────────────┤ SCL (MPU-6050 Sarung Tangan)
│         GPIO 1 ├──────────────┤ Tombol ARM / LAND (Pull-Up)
│         GPIO 4 ├──────────────┤ Tombol EMERGENCY (Pull-Up)│
│        GPIO 10 ├──────────────┤ Tombol READY (Pull-Up)    │
│         GPIO 5 ├──────[330Ω]──┤ LED Hijau (Status)        │
│         GPIO 3 ├──────[330Ω]──┤ LED Merah (Status/Error)  │
│            GND ├──────────────┤ GND Baterai & MPU-6050    │
│            VCC ├──────────────┤ 3.3V / VBUS Baterai       │
└────────────────┘              └───────────────────────────┘
```

> [!NOTE]
> Semua tombol menggunakan rangkaian internal/eksternal pull-up aktif LOW (`INPUT_PULLUP`). Menekan tombol berarti menghubungkan pin GPIO ke `GND`.
