#include "ReceiverEspNow/ReceiverEspNow.h"
#include <ctype.h>

ReceiverEspNow receiver;

volatile RawPacket ReceiverEspNow::_rawPacket = {{0}, 0};
volatile bool      ReceiverEspNow::_hasNewRawPacket = false;

// ================================================================
//  CALLBACK: Data masuk dari ESP-NOW (ISR WiFi Context, Core 0)
//  HANYA menyalin raw bytes ke memory, tidak memproses apapun
// ================================================================
void IRAM_ATTR ReceiverEspNow::onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len <= 0) return;

    int copyLen = (len < (int)sizeof(_rawPacket.data)) ? len : (int)sizeof(_rawPacket.data) - 1;
    memcpy((void*)_rawPacket.data, data, copyLen);
    _rawPacket.len = copyLen;
    _hasNewRawPacket = true;
}

bool ReceiverEspNow::begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.printf("[ESP-NOW] MAC Address Drone: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW ERROR] Gagal inisialisasi ESP-NOW!");
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);
    Serial.println("[ESP-NOW] Receiver aktif & siap menerima paket.");
    return true;
}

// ================================================================
//  processIncomingPacket — Dipanggil dari commTask (Core 0)
// ================================================================
bool ReceiverEspNow::processIncomingPacket(DroneCommand &outCmd, uint16_t &outThrottle)
{
    if (!_hasNewRawPacket)
    {
        return false;
    }

    // Salin lokal dengan cepat agar ISR dapat menerima paket berikutnya
    RawPacket localPacket;
    localPacket.len = _rawPacket.len;
    memcpy(localPacket.data, (const void*)_rawPacket.data, localPacket.len);
    _hasNewRawPacket = false;

    outCmd = CMD_NONE;
    outThrottle = 0;

    // 1. Format Paket Biner (DronePacket — 3 bytes)
    if (localPacket.len == sizeof(DronePacket))
    {
        const DronePacket *pkt = (const DronePacket *)localPacket.data;

        uint8_t c = pkt->command;
        if ((c >= CMD_MAJU && c <= CMD_KANAN) || (c >= CMD_BTN_ARM_LAND && c <= CMD_BTN_READY))
        {
            outCmd = (DroneCommand)c;
        }
        else
        {
            outCmd = CMD_NONE;
        }

        if (pkt->throttle >= PWM_MIN && pkt->throttle <= PWM_MAX)
        {
            outThrottle = pkt->throttle;
        }
        return true;
    }

    // 2. Format Teks String (untuk uji coba / bench test)
    char text[32];
    int copyLen = min(localPacket.len, (int)(sizeof(text) - 1));
    memcpy(text, localPacket.data, copyLen);
    text[copyLen] = '\0';

    for (int i = 0; i < copyLen; i++)
    {
        text[i] = toupper((unsigned char)text[i]);
    }

    // Tombol glove
    if (strstr(text, "ARM_LAND") || strcmp(text, "T1") == 0)
    {
        outCmd = CMD_BTN_ARM_LAND;
    }
    else if (strstr(text, "EMERGENCY") || strcmp(text, "T2") == 0 || strcmp(text, "E") == 0)
    {
        outCmd = CMD_BTN_EMERGENCY;
    }
    else if (strstr(text, "READY") || strcmp(text, "T3") == 0)
    {
        outCmd = CMD_BTN_READY;
    }
    // Gesture gerak
    else if (strstr(text, "MAJU") || strcmp(text, "F") == 0)
    {
        outCmd = CMD_MAJU;
    }
    else if (strstr(text, "MUNDUR") || strcmp(text, "B") == 0)
    {
        outCmd = CMD_MUNDUR;
    }
    else if (strstr(text, "KIRI") || strcmp(text, "L") == 0)
    {
        outCmd = CMD_KIRI;
    }
    else if (strstr(text, "KANAN") || strcmp(text, "R") == 0)
    {
        outCmd = CMD_KANAN;
    }
    else if (strstr(text, "IDLE") || strcmp(text, "I") == 0)
    {
        outCmd = CMD_NONE;
    }

    return true;
}
