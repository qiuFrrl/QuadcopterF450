# Dokumentasi Sistem Terintegrasi: Quadcopter F450 & Glove Controller

Selamat datang di repositori dokumentasi lengkap sistem **Quadcopter F450** berbasis **ESP32 Dual-Core** dan pengontrol nirkabel sarung tangan **Glove Controller** berbasis **ESP32-C3**.

Dokumentasi ini disusun secara terstruktur, mendalam, dan dilengkapi dengan diagram alur serta analisis kode agar pembaca—baik mahasiswa, teknisi, maupun pengembang sistem tertanam (*embedded system*)—dapat memahami seluruh konsep dan mekanisme kerja proyek ini secara menyeluruh dari hulu ke hilir.

---

## 📚 Struktur Dokumen

Dokumentasi ini dibagi menjadi beberapa bab terpisah yang saling melengkapi:

| No | Modul / Dokumen | Deskripsi Isi Pembahasan |
|:--:|---|---|
| **01** | [**01. Arsitektur & Ikhtisar Sistem**](01_arsitektur_dan_ikhtisar_sistem.md) | Visi sistem, filosofi kendali satu langkah (*step-by-step gesture*), perbandingan mikrokontroler (ESP32 vs ESP32-C3), diagram blok interkoneksi, dan spesifikasi komponen fisik. |
| **02** | [**02. Glove Controller (Sarung Tangan)**](02_glove_controller_sarung_tangan.md) | Arsitektur event loop 40 Hz non-blocking pada ESP32-C3, sistem tombol navigasi (ARM, EMERGENCY, READY), akuisisi IMU MPU-6050 sarung tangan, algoritma inferensi gesture, dan umpan balik LED status. |
| **03** | [**03. Protokol Komunikasi ESP-NOW**](03_protokol_komunikasi_espnow.md) | Penjelasan mendalam protokol nirkabel 2.4 GHz ESP-NOW, format biner paket (`DronePacketV2` & `DroneStatusPacket`), **investigasi akar masalah sinyal sering terputus** (WiFi Modem Sleep), solusi parameter RF, dan manajemen paket raw ring buffer. |
| **04** | [**04. Quadcopter Dual-Core FreeRTOS**](04_quadcopter_dual_core_freertos.md) | Pemisahan beban kerja Asymmetric Multiprocessing (AMP): Core 0 (CommTask 50 Hz) vs Core 1 (FlightTask 250 Hz deterministik 4.000 µs), serta mekanisme sinkronisasi IPC (*Queue* & *Spinlock Shared Memory*). |
| **05** | [**05. Estimasi Sikap IMU & Cascade PID**](05_estimasi_sikap_imu_dan_kendali_pid.md) | Pengolahan sensor MPU-6050 (DLPF 44 Hz, Fast-Mode I2C), fusi sikap *Complementary Filter* ($\alpha = 0.991$), arsitektur kontrol *Dual-Loop Cascade PID* (Angle Outer + Rate Inner), dan perumusan matriks mixer motor Quad-X. |
| **06** | [**06. Finite State Machine & Keamanan**](06_finite_state_machine_dan_keamanan.md) | Siklus hidup status drone (DISARMED $\rightarrow$ ARMED $\rightarrow$ FLYING $\rightarrow$ LANDING $\rightarrow$ EMERGENCY), logika eksekusi gesture terukur (500 ms tilt + auto level), dan sistem keselamatan failsafe bertingkat (sinyal putus, IMU timeout, kemiringan ekstrem). |
| **07** | [**07. Panduan Operasional & Pengujian**](07_panduan_operasional_dan_pengujian.md) | Langkah-langkah kompilasi/upload PlatformIO, kalibrasi sensor, pengujian meja (*bench test*) via antarmuka Serial CLI tanpa baling-baling, verifikasi respons mixer motor, prosedur uji terbang aman (*tether test*), dan tabel pemecahan masalah (*troubleshooting*). |

---

## 🗺️ Peta Alur Kerja Sistem End-to-End

Berikut adalah gambaran besar (*big picture*) alur data dari tangan pengguna hingga baling-baling drone:

```mermaid
flowchart TD
    subgraph Pilot["Pengguna (Glove Controller)"]
        A[Gerakan Tangan & Tekanan Tombol] --> B[ESP32-C3 SuperMini]
        B -->|IMU MPU6050| C[Sampling Sudut Gesture]
        B -->|Tombol Fisik| D[State Tombol: ARM / READY / EMERGENCY]
        C & D --> E[ESP-NOW Transmitter @ 40Hz]
    end

    E -->|ESP-NOW 2.4GHz Uplink Ch.1| F[ESP-NOW Receiver Drone]

    subgraph Drone["Quadcopter F450 (ESP32 WROOM)"]
        subgraph Core0["Core 0: Comm Task (50 Hz)"]
            F --> G[Raw Packet Ring Buffer]
            G --> H[Deduplikasi Seq Number & Validasi Paket]
            H -->|FreeRTOS Queue| I[Command Queue xQueueCommand]
            H -->|portMUX Spinlock| J[Update g_sharedState]
            K[Telemetry Broadcaster 10Hz] -.->|Downlink Status| B
        end

        subgraph Core1["Core 1: Flight Task (250 Hz - Deterministik 4.000µs)"]
            I --> L[Finite State Machine FSM]
            M[MPU6050 I2C 400kHz] --> N[Complementary Filter α=0.991]
            N -->|Roll, Pitch Aktual| O[Outer Loop: Angle PID]
            L -->|Target Sudut Step Gesture| O
            O -->|Target Rate dps| P[Inner Loop: Rate PID]
            M -->|Gyro Roll/Pitch/Yaw Rate| P
            P -->|Koreksi Torsi µs| Q[Motor Mixer Quad-X]
            L -->|Base Throttle| Q
            Q --> R[Driver Hardware LEDC PWM 500Hz]
        end
    end

    R --> S[4x ESC 30A & Motor Brushless 2212 1000KV]
```

---

## 💡 Konsep Inti yang Membedakan Sistem Ini

1. **Kendali Bertahap / Satu Langkah (*Step-by-Step Gesture Control*)**
   - Bukan kendali proporsional berkelanjutan yang berisiko membuat drone hilang kendali karena tangan bergetar atau lelah.
   - 1 kali tombol `READY` ditekan $\rightarrow$ membaca kemiringan tangan selama 250 ms $\rightarrow$ drone melakukan manuver terukur kemiringan $4^\circ$ selama 1.000 ms dengan kompensasi *throttle boost* otomatis $\rightarrow$ drone kembali otomatis ke posisi datar seimbang (*level hover*).
2. **Pemisahan Beban Dual-Core (Asymmetric Multiprocessing)**
   - Proses komunikasi radio, pemrosesan serial, dan broadcast telemetri tidak pernah mengganggu loop kendali penerbangan.
   - Core 1 memiliki determinisme tinggi pada interval $4.000\ \mu\text{s}$ ($250\text{ Hz}$) tanpa jitter yang disebabkan oleh radio WiFi/ESP-NOW.
3. **Keandalan Koneksi Nirkabel Tanpa Putus (*No Power-Save Lock*)**
   - Kedua mikroprosesor (ESP32 & ESP32-C3) dikunci pada mode `WIFI_PS_NONE` (modem power save dinonaktifkan) dan daya pancar maksimum $19.5\text{ dBm}$ pada Kanal 1, menjamin koneksi peer-to-peer instan dengan latensi sub-$3\text{ ms}$ tanpa kehilangan paket secara berkala.
4. **Failsafe Mandiri Multi-Tingkat**
   - Jika sinyal remote terputus selama $>2\text{ detik}$, drone tidak langsung jatuh bebas ataupun terbang tak terkendali; drone secara otomatis menurunkan throttle secara halus ($-80\ \mu\text{s/detik}$) hingga menyentuh lantai, kemudian mematikan motor secara otomatis.
