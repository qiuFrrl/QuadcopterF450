#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"
#include "shared/shared_types.h"

class ReceiverEspNow
{
public:
    bool begin();

    // Memproses paket mentah yang disalin oleh callback ESP-NOW.
    // Dijalankan di commTask (Core 0), BUKAN di ISR.
    // Mengembalikan true jika ada paket baru yang berhasil diproses.
    bool processIncomingPacket(DroneCommand &outCmd, uint16_t &outThrottle);

    // Callback data masuk dari ESP-NOW (context WiFi task di Core 0, bukan ISR).
    // HANYA menyalin raw bytes dan mengeset flag. Tidak ada parsing / string ops / print.
    static void IRAM_ATTR onDataRecv(const uint8_t *mac, const uint8_t *data, int len);

private:
    static volatile RawPacket _rawPacket;
    static volatile bool      _hasNewRawPacket;
    static portMUX_TYPE       _rawMux;
    static uint8_t            _lastSeq;
    static bool               _hasLastSeq;
};

extern ReceiverEspNow receiver;
