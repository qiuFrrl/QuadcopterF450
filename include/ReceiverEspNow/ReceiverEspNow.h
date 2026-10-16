#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"
#include "shared/shared_types.h"

#define RAW_QUEUE_CAPACITY 32

class ReceiverEspNow
{
public:
    bool begin();

    // Memproses paket mentah dari ring buffer.
    // Dijalankan di commTask (Core 0).
    // Mengembalikan true jika ada paket baru yang berhasil diproses.
    bool processIncomingPacket(DroneCommand &outCmd, uint16_t &outThrottle);

    // Callback data masuk dari ESP-NOW (context WiFi task di Core 0).
    static void IRAM_ATTR onDataRecv(const uint8_t *mac, const uint8_t *data, int len);

private:
    static RawPacket          _rawQueue[RAW_QUEUE_CAPACITY];
    static volatile uint8_t   _qHead;
    static volatile uint8_t   _qTail;
    static portMUX_TYPE       _rawMux;
    static uint8_t            _lastSeq;
    static bool               _hasLastSeq;
};

extern ReceiverEspNow receiver;
