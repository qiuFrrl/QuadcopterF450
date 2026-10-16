# Bab 7: Panduan Operasional, Prosedur Uji Meja & Pemecahan Masalah

Dokumen ini berisi panduan teknis langkah demi langkah untuk melakukan kompilasi, pengunggahan firmware (*uploading*), pengujian meja (*bench testing*) tanpa baling-baling via Serial CLI, prosedur uji terbang bertahap (*tethered flight test*), serta tabel pemecahan masalah (*troubleshooting*).

---

## 7.1 Kompilasi & Pengunggahan Firmware (Flashing)

Proyek ini dikembangkan menggunakan **PlatformIO Core (CLI)** atau PlatformIO IDE pada Visual Studio Code.

### A. Upload ke Quadcopter F450 (ESP32)
1. Hubungkan board ESP32 drone ke komputer menggunakan kabel data Micro-USB.
2. Identifikasi nomor port COM (misal `COM8`).
3. Jalankan perintah kompilasi dan upload dari folder `QuadcopterF450`:
```powershell
# Kompilasi kode sumber
pio run

# Upload firmware ke ESP32 drone (ganti COM8 sesuai port Anda)
pio run --target upload --upload-port COM8

# Membuka Serial Monitor (115200 baud)
pio device monitor -b 115200
```

### B. Upload ke Glove Controller (ESP32-C3)
1. Hubungkan board ESP32-C3 SuperMini sarung tangan ke komputer via kabel Type-C.
2. Jalankan perintah dari folder `Controller`:
```powershell
# Kompilasi dan upload ke Glove Controller
pio run -d "..\Controller" --target upload

# Membuka Serial Monitor Glove Controller
pio device monitor -b 115200
```

> [!TIP]
> **Catatan USB CDC pada ESP32-C3:**  
> ESP32-C3 menggunakan USB On-The-Go (CDC) internal. Pastikan driver terdeteksi sebagai port COM. Jika papan gagal masuk mode flashing, tahan tombol **BOOT**, tekan tombol **RESET** sekali, lalu lepaskan tombol BOOT.

---

## 7.2 Protokol Pengujian Meja (Bench Test Bebas Risiko)

> [!CAUTION]
> **ATURAN KESELAMATAN MUTLAK:**  
> **SELALU LEPAS SEMUA BALING-BALING (PROPELLER)** saat melakukan pengujian di atas meja (*bench test*) atau saat menghubungkan drone ke komputer! Baling-baling berputar tanpa beban kendali dapat menyebabkan cedera fisik serius.

Flight Controller dilengkapi antarmuka interaktif **Serial CLI** pada Core 0 yang memungkinkan pengujian seluruh fungsi kendali tanpa memerlukan remote nirkabel.

### Daftar Perintah Serial Monitor (Baud Rate: 115200):
| Karakter Tombol | Perintah | Fungsi Pengujian |
|:---:|---|---|
| `?` | Diagnostik Sistem | Mencetak laporan lengkap: status FSM, sudut Roll/Pitch, output 4 motor, status IMU, jitter loop, dan pemakaian stack memory. |
| `b` | Toggle Bench Guard | Membuka/mengunci pengaman kenaikan throttle di atas meja (*bench throttle lock*). |
| `1` | BTN_ARM_LAND | Mengubah status dari `DISARMED` $\rightarrow$ `ARMED` $\rightarrow$ `FLYING` $\rightarrow$ `LANDING`. |
| `2` | BTN_EMERGENCY | Mematikan seluruh motor seketika ($1.000\ \mu\text{s}$). |
| `3` | BTN_READY | Memicu simulasi jendela gesture ready. |
| `w` / `W` | GESTURE MAJU | Menginjeksi perintah gerak maju ($+4.0^\circ$ pitch) selama 1.000 ms. |
| `s` / `S` | GESTURE MUNDUR | Menginjeksi perintah gerak mundur ($-4.0^\circ$ pitch) selama 1.000 ms. |
| `a` / `A` | GESTURE KIRI | Menginjeksi perintah gerak ke kiri ($-4.0^\circ$ roll) selama 1.000 ms. |
| `d` / `D` | GESTURE KANAN | Menginjeksi perintah gerak ke kanan ($+4.0^\circ$ roll) selama 1.000 ms. |
| `+` | THROTTLE UP | Menaikkan nilai throttle manual sebesar $+20\ \mu\text{s}$. |
| `-` | THROTTLE DOWN | Menurunkan nilai throttle manual sebesar $-20\ \mu\text{s}$. |
| `c` | KALIBRASI SENSOR | Mengambil 500 sampel offset giroskop & akselerometer saat drone diletakkan diam dan rata. |

---

## 7.3 Verifikasi Arah Putaran & Respon Motor Mixer

Sebelum memasang baling-baling, lakukan uji fisik respon kendali berikut di atas meja:

1. **Uji Idle Motor:**  
   Ketik `1` di serial monitor. Keempat motor harus mulai berputar perlahan pada kecepatan idle ($1.170\ \mu\text{s}$).
2. **Verifikasi Arah Putar Fisik Motor:**
   - Motor 1 (FR): Harus berputar **CCW** (Berlawanan arah jarum jam).
   - Motor 2 (RR): Harus berputar **CW** (Searah jarum jam).
   - Motor 3 (RL): Harus berputar **CCW** (Berlawanan arah jarum jam).
   - Motor 4 (FL): Harus berputar **CW** (Searah jarum jam).
   *(Jika ada motor yang arahnya terbalik, tukar sembarang dua dari tiga kabel antara ESC dan motor tersebut).*
3. **Verifikasi Respon Keseimbangan (Negative Feedback PID):**
   - Miringkan hidung drone ke bawah secara manual $\rightarrow$ Motor depan (M1 & M4) harus terdengar menaikkan kecepatan untuk mendorong hidung kembali ke atas.
   - Miringkan badan drone ke sisi kiri $\rightarrow$ Motor kiri (M3 & M4) harus menaikkan kecepatan untuk mendorong sayap kiri kembali ke atas.
   - Jika saat dimiringkan motor yang rendah justru melambat dan motor yang tinggi malah bertambah kencang, itu artinya tanda mixer atau orientasi sensor terbalik (*positive feedback / runaway*). Jangan pernah diterbangkan sebelum tanda mixer diperbaiki!

---

## 7.4 Prosedur Uji Terbang Bertahap (Tethered Flight Test)

Jangan langsung melepas drone di area terbuka tanpa pengujian terkendali:

```mermaid
flowchart LR
    A[Bench Test Tanpa Propeller] --> B[Tether Test Tali Pengikat 30-50 cm]
    B --> C[Uji Hover Mengambang Terkendali]
    C --> D[Uji Respon Gesture Maju/Mundur]
    D --> E[Penerbangan Bebas Terbuka]
```

1. **Uji Tambat (*Tether Test*):**  
   Ikat keempat ujung kaki frame drone menggunakan tali nilon elastis ke pemberat di lantai dengan panjang kebebasan gerak hanya $30\text{--}50\text{ cm}$.
2. **Validasi Ketinggian Hover:**  
   Amati apakah pada nilai `HOVER_THROTTLE` ($1.300\ \mu\text{s}$) badan drone mulai terasa ringan dan mengambang stabil. Jika drone belum terangkat, naikkan nilai `HOVER_THROTTLE` di `config.h` secara bertahap ($1.320$, $1.350$, dst.).
3. **Uji Pendaratan Otomatis:**  
   Saat sedang melayang pada tali tambat, tekan tombol `ARM/LAND` pada sarung tangan. Pastikan drone menurunkan ketinggian secara halus dan mematikan motor saat menyentuh lantai.

---

## 7.5 Tabel Pemecahan Masalah (Troubleshooting)

| Gejala Masalah | Kemungkinan Akar Masalah | Tindakan Solusi Rekayasa |
|---|---|---|
| **ESP-NOW sering terputus berkala** | Firmware lama masih mengaktifkan WiFi Power Save | Pastikan firmware terbaru sudah ter-flash. Cek apakah di Serial Monitor muncul log `Power Save: NONE` dan `TX Power: 19.5dBm`. |
| **Drone langsung terbalik (*flip-over*) saat lepas landas** | Arah putaran motor salah, urutan kabel ESC terbalik, atau tipe baling-baling CW/CCW tertukar | Lepas propeller. Uji arah putar masing-masing motor sesuai diagram Bab 1. Pastikan baling-baling tipe "R" (CW) dipasang pada motor CW. |
| **Glove Controller berkedip Merah cepat (5 Hz)** | Modul MPU-6050 sarung tangan tidak terdeteksi pada bus I2C | Periksa koneksi jumper SDA (GPIO 8) dan SCL (GPIO 9), pastikan modul MPU-6050 menerima tegangan 3.3V bersih. |
| **Tombol READY ditekan tetapi LED Merah berkedip 3x** | Status drone masih `DISARMED` | Drone harus di-ARM terlebih dahulu menggunakan tombol 1 (`BTN_ARM_LAND`) sebelum perintah gesture dapat diproses. |
| **Drone berosilasi cepat (*high frequency wobble*) saat terbang** | Gain PID inner loop (Rate PID) terlalu tinggi | Turunkan nilai `P_RATE_ROLL` dan `P_RATE_PITCH` dari $0.625$ menjadi $0.500$ di `config.h`. |
| **Drone condong melayang ke satu arah saat hover** | Nilai offset kalibrasi MPU-6050 belum presisi | Letakkan drone pada meja yang benar-benar rata waterpass, buka Serial Monitor, lalu ketik perintah `c` untuk kalibrasi ulang. |
| **ESP32 drone gagal boot (*boot loop* acak saat dinyalakan)** | GPIO 12 ditarik HIGH oleh rangkaian ESC | Pindahkan pin sinyal Motor 3 (RL) dari GPIO 12 ke GPIO 14 pada file `config.h` dan pengkabelan fisik. |
