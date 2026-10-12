#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "config.h"

// ================================================================
//  STATE MACHINE — Status Utama Drone
// ================================================================
enum DroneState : uint8_t
{
    STATE_DISARMED  = 0,
    STATE_ARMED     = 1,
    STATE_FLYING    = 2,
    STATE_LANDING   = 3,
    STATE_EMERGENCY = 4
};

inline const char *getDroneStateName(DroneState state)
{
    switch (state)
    {
    case STATE_DISARMED:  return "DISARMED";
    case STATE_ARMED:     return "ARMED";
    case STATE_FLYING:    return "FLYING";
    case STATE_LANDING:   return "LANDING";
    case STATE_EMERGENCY: return "EMERGENCY";
    default:              return "UNKNOWN";
    }
}

// ================================================================
//  PERINTAH DRONE — Mapping Tombol Glove Controller & Serial
// ================================================================
enum DroneCommand : uint8_t
{
    CMD_NONE          = 0,   // Idle / heartbeat
    CMD_MAJU          = 1,   // Gesture: maju
    CMD_MUNDUR        = 2,   // Gesture: mundur
    CMD_KIRI          = 3,   // Gesture: kiri
    CMD_KANAN         = 4,   // Gesture: kanan
    CMD_BTN_ARM_LAND  = 10,  // Tombol 1: ARM / LAND / CONFIRM
    CMD_BTN_EMERGENCY = 11,  // Tombol 2: EMERGENCY STOP
    CMD_BTN_READY     = 12,  // Tombol 3: Aktifkan gesture
    // Perintah khusus Serial Bench Test:
    CMD_THROTTLE_UP   = 20,  // '+' → Naikkan throttle
    CMD_THROTTLE_DN   = 21,  // '-' → Turunkan throttle
    CMD_CALIBRATE     = 22,  // 'c' → Kalibrasi IMU
    CMD_STATUS        = 23   // '?' → Tampilkan info status
};

inline const char *getDroneCommandName(DroneCommand cmd)
{
    switch (cmd)
    {
    case CMD_MAJU:          return "MAJU";
    case CMD_MUNDUR:        return "MUNDUR";
    case CMD_KIRI:          return "KIRI";
    case CMD_KANAN:         return "KANAN";
    case CMD_BTN_ARM_LAND:  return "BTN_ARM_LAND";
    case CMD_BTN_EMERGENCY: return "EMERGENCY";
    case CMD_BTN_READY:     return "BTN_READY";
    case CMD_THROTTLE_UP:   return "THROTTLE_UP";
    case CMD_THROTTLE_DN:   return "THROTTLE_DN";
    case CMD_CALIBRATE:     return "CALIBRATE";
    case CMD_STATUS:        return "STATUS";
    case CMD_NONE:
    default:                return "---";
    }
}

// ================================================================
//  PAKET KOMUNIKASI
// ================================================================

// Struktur paket biner ESP-NOW (3 bytes)
struct __attribute__((packed)) DronePacket
{
    uint8_t  command;      // DroneCommand enum
    uint16_t throttle;     // Nilai throttle (1000 - 1800 µs), 0 = abaikan
};

// Raw buffer untuk transfer aman dari ISR ESP-NOW ke commTask (Core 0)
struct RawPacket
{
    uint8_t data[32];
    int     len;
};

// ================================================================
//  DATA BERSAMA ANTAR-CORE (Thread-Safe via portMUX spinlock)
// ================================================================
struct SharedState
{
    uint16_t throttle;         // Throttle dari remote atau serial (µs)
    uint32_t lastPacketTime;   // millis() terakhir paket diterima
    bool     hasEverReceived;  // true jika pernah terima paket
    DroneState droneState;     // State saat ini (diperbarui oleh flightTask)
};

// Objek sinkronisasi global
extern QueueHandle_t xQueueCommand;
extern SharedState   g_sharedState;
extern portMUX_TYPE  g_sharedMux;
extern TaskHandle_t  hFlightTask;
extern TaskHandle_t  hCommTask;
