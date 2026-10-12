#pragma once

// ============================================================
//  CONFIG.H — Konfigurasi Quadcopter F450
//  Referensi: FlightController.cpp (cascade PID, ±500°/s, ±8g)
// ============================================================

// -- PIN MOTOR ESC --
#define PIN_FL 27   // Front-Left  (CW)
#define PIN_FR 25   // Front-Right (CCW)
#define PIN_RL 12   // Rear-Left   (CCW)
#define PIN_RR 33   // Rear-Right  (CW)

// -- PIN I2C (MPU6050) --
#define SDA_PIN 21
#define SCL_PIN 22

// -- LED INDIKATOR --
#define LED_PIN 15

// -- MPU6050 SENSOR SCALE --
// Gyro  : ±500°/s  → 65.5 LSB/(°/s)   (reg 0x1B = 0x08)
// Accel : ±8g      → 4096 LSB/g        (reg 0x1C = 0x10)
#define GYRO_SCALE_REG    0x08
#define ACCEL_SCALE_REG   0x10
#define GYRO_SENSITIVITY  65.5f
#define ACCEL_SENSITIVITY 4096.0f

// -- MPU6050 KALIBRASI (Satuan Fisik: deg/s & G) --
// Nilai ini dikurangi SETELAH konversi ke satuan fisik
#define RateCalibrationRoll  -2.40f
#define RateCalibrationPitch -2.09f
#define RateCalibrationYaw    0.83f
#define AccXCalibration       0.08f
#define AccYCalibration      -0.01f
#define AccZCalibration       0.02f

// -- COMPLEMENTARY FILTER --
// α = 0.991 → sangat berat ke gyro, cocok di 250Hz
#define CF_ALPHA 0.991f

// -- PID GAINS: OUTER LOOP (Angle) --
// Output = desired rate (deg/s)
#define P_ANGLE_ROLL  2.0f
#define I_ANGLE_ROLL  0.5f
#define D_ANGLE_ROLL  0.007f
#define P_ANGLE_PITCH P_ANGLE_ROLL
#define I_ANGLE_PITCH I_ANGLE_ROLL
#define D_ANGLE_PITCH D_ANGLE_ROLL

// -- PID GAINS: INNER LOOP (Rate) --
// Output = motor correction (µs)
#define P_RATE_ROLL   0.625f
#define I_RATE_ROLL   2.1f
#define D_RATE_ROLL   0.0088f
#define P_RATE_PITCH  P_RATE_ROLL
#define I_RATE_PITCH  I_RATE_ROLL
#define D_RATE_PITCH  D_RATE_ROLL
#define P_RATE_YAW    4.0f
#define I_RATE_YAW    3.0f
#define D_RATE_YAW    0.0f

// -- PID LIMITS --
#define PID_LIMIT    400.0f
#define ANGLE_LIMIT  20.0f

// -- MOTOR LIMITS --
#define PWM_MIN       1000
#define PWM_MAX       1999
#define PWM_IDLE      1170
#define PWM_ARM       1000
#define THROTTLE_MAX  1800
#define THROTTLE_CUTOFF 1030  // throttle di bawah ini = disarmed

// -- LOOP --
#define LOOP_HZ 250              // target loop rate (Hz)
#define LOOP_TIME_US 4000        // 1000000 / 250 = 4000µs
#define LOOP_TIME_S  0.004f      // 1.0 / 250 = 0.004s

// -- GENTLE TILT SETTINGS (Kontrol Gerakan Halus & Stabil) --
#define GENTLE_TILT_ANGLE   4.0f   // Sudut kemiringan lembut (derajat) untuk maju/mundur/kiri/kanan
#define TILT_RAMP_RATE      0.08f  // Faktor smoothing perubahan sudut per loop

// -- ESP-NOW & FAILSAFE SETTINGS --
#define ESPNOW_WIFI_CHANNEL 1      // Kanal WiFi ESP-NOW
#define ESPNOW_TIMEOUT_MS   1000   // Failsafe timeout (ms) jika sinyal remote hilang

// -- STATE MACHINE & LANDING --
#define LANDING_THROTTLE_DROP_RATE 2   // Penurunan throttle per loop saat landing (µs/loop, ~500µs/detik)
#define LANDING_THROTTLE_FLOOR   1050  // Throttle minimum saat landing sebelum konfirmasi sentuh tanah

// -- GESTURE STEP SETTINGS --
#define GESTURE_STEP_DURATION_MS  500  // Durasi satu langkah gesture (ms) — 0.5 detik miringkan lalu kembali
#define GESTURE_TILT_ANGLE        3.0f // Kemiringan saat gesture aktif (derajat, lebih kecil dari manual)

// -- SAFETY --
#define IMU_MAX_TILT_ANGLE       45.0f // Sudut miring maksimum sebelum emergency auto-cut (derajat)
#define MOTOR_CUT_DELAY_MS       200   // Delay sebelum motor benar-benar mati saat emergency (debounce)

// -- FREERTOS DUAL-CORE TASK SETTINGS --
#define FLIGHT_TASK_HZ          250
#define FLIGHT_TASK_PERIOD_MS   (1000 / FLIGHT_TASK_HZ)  // 4ms
#define FLIGHT_TASK_STACK       4096
#define FLIGHT_TASK_PRIORITY    5

#define COMM_TASK_HZ            50
#define COMM_TASK_PERIOD_MS     (1000 / COMM_TASK_HZ)     // 20ms
#define COMM_TASK_STACK         4096
#define COMM_TASK_PRIORITY      2

#define COMMAND_QUEUE_DEPTH     4

