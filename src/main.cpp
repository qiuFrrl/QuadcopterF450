#include <Arduino.h>
#include "config.h"
#include "shared/shared_types.h"
#include "imu/imu.h"
#include "pid/pid.h"
#include "motor/motor.h"
#include "ReceiverEspNow/ReceiverEspNow.h"
#include "comm/comm_task.h"
#include "flight/flight_task.h"

// ================================================================
//  OBJEK SINKRONISASI & TUGAS FREERTOS DUAL-CORE
// ================================================================
QueueHandle_t xQueueCommand = NULL;
SharedState   g_sharedState = {PWM_ARM, 0, false, STATE_DISARMED};
portMUX_TYPE  g_sharedMux   = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t  hFlightTask   = NULL;
TaskHandle_t  hCommTask     = NULL;

// ================================================================
//  SETUP
// ================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // LED menyala solid = DISARMED (mode aman)

    Serial.println("\n========================================================");
    Serial.println("  QUADCOPTER F450 — DUAL-CORE FREERTOS ARCHITECTURE");
    Serial.println("  Core 0: Communication Task (ESP-NOW + Serial) [50Hz]");
    Serial.println("  Core 1: Flight Control Task (IMU + PID + Motor) [250Hz]");
    Serial.println("========================================================");

    // 1. Inisialisasi Driver Motor ESC (semua motor di-cut ke 1000µs)
    Serial.println("[INIT] Menginisialisasi Motor ESC...");
    motor.begin();

    // 2. Inisialisasi Sensor IMU (MPU6050)
    Serial.println("[INIT] Menginisialisasi Sensor MPU6050...");
    if (!imu.begin())
    {
        Serial.println("[FATAL ERROR] Sensor MPU6050 gagal merespons!");
        Serial.println("[FATAL ERROR] Periksa kabel SDA/SCL. Boot dihentikan demi keselamatan.");
        while (1)
        {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }

    // 3. Inisialisasi Receiver ESP-NOW
    Serial.println("[INIT] Menginisialisasi Receiver ESP-NOW...");
    if (!receiver.begin())
    {
        Serial.println("[WARN] Gagal inisialisasi ESP-NOW. Gunakan Serial Bench Test.");
    }

    // 4. Buat FreeRTOS Command Queue untuk pengiriman perintah tombol / gesture
    xQueueCommand = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(DroneCommand));
    if (xQueueCommand == NULL)
    {
        Serial.println("[FATAL ERROR] Gagal mengalokasikan Command Queue!");
        while (1);
    }

    // 5. Buat Flight Control Task (Core 1, Prioritas 5 — Real-time)
    xTaskCreatePinnedToCore(
        flightTask,
        "FlightTask",
        FLIGHT_TASK_STACK,
        NULL,
        FLIGHT_TASK_PRIORITY,
        &hFlightTask,
        APP_CPU_NUM // Core 1
    );

    // 6. Buat Communication Task (Core 0, Prioritas 2)
    xTaskCreatePinnedToCore(
        commTask,
        "CommTask",
        COMM_TASK_STACK,
        NULL,
        COMM_TASK_PRIORITY,
        &hCommTask,
        PRO_CPU_NUM // Core 0
    );

    Serial.println("\n[SISTEM SIAP] Drone berada pada status SAFE DISARMED.");
    Serial.println("=== PANDUAN PENGUJIAN AMAN (BENCH TEST TANPA BALING-BALING) ===");
    Serial.println("  '1'     : BTN_ARM_LAND (DISARMED → ARMED → LANDING → CONFIRM)");
    Serial.println("  '2'     : BTN_EMERGENCY (motor cut seketika dari state manapun)");
    Serial.println("  '3'     : BTN_READY (buka 1 sesi pengenalan gesture)");
    Serial.println("  'w/s'   : Gesture MAJU / MUNDUR (tilt target 3° selama 0.5s)");
    Serial.println("  'a/d'   : Gesture KIRI / KANAN (tilt target 3° selama 0.5s)");
    Serial.println("  '+/-'   : Naikkan / turunkan throttle simulasi (25µs)");
    Serial.println("  'c'     : Kalibrasi IMU fisik (hanya saat DISARMED)");
    Serial.println("  '?'     : Status telemetri ringkas");
    Serial.println("========================================================\n");

    // Hapus Arduino loopTask bawaan untuk menghemat memori
    vTaskDelete(NULL);
}

void loop()
{
    // Tidak digunakan karena loopTask dihapus di akhir setup()
}