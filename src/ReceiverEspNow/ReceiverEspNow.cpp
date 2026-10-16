#include "ReceiverEspNow/ReceiverEspNow.h"
#include <ctype.h>
#include <string.h>

ReceiverEspNow receiver;

RawPacket          ReceiverEspNow::_rawQueue[RAW_QUEUE_CAPACITY];
volatile uint8_t   ReceiverEspNow::_qHead = 0;
volatile uint8_t   ReceiverEspNow::_qTail = 0;
portMUX_TYPE       ReceiverEspNow::_rawMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t            ReceiverEspNow::_lastSeq = 0;
bool               ReceiverEspNow::_hasLastSeq = false;

// ================================================================
//  CALLBACK: Data masuk dari ESP-NOW (Core 0 WiFi task)
//  Menyimpan ke ring buffer agar tidak ada paket tertimpa / hilang
// ================================================================
void IRAM_ATTR ReceiverEspNow::onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len <= 0) return;

    const uint8_t gloveMac[6] = GLOVE_MAC_ADDR;
    bool macConfigured = false;
    for (int i = 0; i < 6; i++)
    {
        if (gloveMac[i] != 0x00)
        {
            macConfigured = true;
            break;
        }
    }

    if (macConfigured && (mac == nullptr || memcmp(mac, gloveMac, 6) != 0))
    {
        return;
    }

    int copyLen = (len < (int)sizeof(_rawQueue[0].data)) ? len : (int)sizeof(_rawQueue[0].data) - 1;

    portENTER_CRITICAL(&_rawMux);
    uint8_t nextHead = (_qHead + 1) % RAW_QUEUE_CAPACITY;
    if (nextHead != _qTail)
    {
        memcpy((void*)_rawQueue[_qHead].data, data, copyLen);
        _rawQueue[_qHead].len = copyLen;
        _qHead = nextHead;
    }
    portEXIT_CRITICAL(&_rawMux);
}

bool ReceiverEspNow::begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Matikan mode sleep/power save WiFi agar ESP32 tidak mematikan modul RF
    // (Penyebab utama ESP-NOW sering putus/dropped packet secara berkala)
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Kunci kanal WiFi sesuai konfigurasi
    esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (channelResult != ESP_OK)
    {
        Serial.printf("[ESP-NOW WARN] Gagal set kanal WiFi %d (err=%d)\n", ESPNOW_WIFI_CHANNEL, channelResult);
    }

    // Naikkan daya pancar WiFi ke level maksimum (19.5 dBm) untuk jangkauan & keandalan sinyal
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    uint8_t primaryChan = 0;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);

    Serial.printf("[ESP-NOW] MAC Address Drone : %s\n", WiFi.macAddress().c_str());
    Serial.printf("[ESP-NOW] WiFi Channel Aktif: %d | Power Save: NONE | Max TX Power: 19.5dBm\n", primaryChan);

    const uint8_t gloveMac[6] = GLOVE_MAC_ADDR;
    bool macConfigured = false;
    for (int i = 0; i < 6; i++)
    {
        if (gloveMac[i] != 0x00)
        {
            macConfigured = true;
            break;
        }
    }
    if (!macConfigured)
    {
        Serial.println("[ESP-NOW WARN] GLOVE_MAC_ADDR masih 00:00:00:00:00:00; semua pengirim diterima.");
    }

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW ERROR] Gagal inisialisasi ESP-NOW!");
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);

    // Daftarkan broadcast peer agar drone bisa broadcast DroneStatusPacket ke controller
    esp_now_peer_info_t bcastPeer = {};
    memset(bcastPeer.peer_addr, 0xFF, 6);
    bcastPeer.channel = ESPNOW_WIFI_CHANNEL;
    bcastPeer.encrypt = false;
    esp_now_add_peer(&bcastPeer);

    Serial.println("[ESP-NOW] Receiver aktif & siap menerima paket cepat.");
    return true;
}

bool ReceiverEspNow::processIncomingPacket(DroneCommand &outCmd, uint16_t &outThrottle)
{
    RawPacket localPacket;
    bool hasPkt = false;

    portENTER_CRITICAL(&_rawMux);
    if (_qHead != _qTail)
    {
        localPacket = _rawQueue[_qTail];
        _qTail = (_qTail + 1) % RAW_QUEUE_CAPACITY;
        hasPkt = true;
    }
    portEXIT_CRITICAL(&_rawMux);

    if (!hasPkt)
    {
        return false;
    }

    outCmd = CMD_NONE;
    outThrottle = 0;

    // 1. Format Paket Biner V2 dengan Sequence Number (DronePacketV2)
    if (localPacket.len == sizeof(DronePacketV2))
    {
        const DronePacketV2 *pkt = (const DronePacketV2 *)localPacket.data;

        uint8_t c = pkt->command;
        if ((c >= CMD_MAJU && c <= CMD_KANAN) || (c >= CMD_BTN_ARM_LAND && c <= CMD_BTN_READY))
        {
            if (_hasLastSeq && pkt->seq == _lastSeq)
            {
                outCmd = CMD_NONE;
            }
            else
            {
                outCmd = (DroneCommand)c;
            }
        }
        else
        {
            outCmd = CMD_NONE;
        }

        _lastSeq = pkt->seq;
        _hasLastSeq = true;

        if (pkt->throttle >= PWM_MIN && pkt->throttle <= THROTTLE_MAX)
        {
            outThrottle = pkt->throttle;
        }
        return true;
    }

    // 2. Format Paket Biner Standar (DronePacket)
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

        if (pkt->throttle >= PWM_MIN && pkt->throttle <= THROTTLE_MAX)
        {
            outThrottle = pkt->throttle;
        }
        return true;
    }

    return true;
}
