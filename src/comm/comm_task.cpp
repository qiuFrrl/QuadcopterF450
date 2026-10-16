#include "comm/comm_task.h"
#include "ReceiverEspNow/ReceiverEspNow.h"

static DroneCommand s_lastEspCmd   = CMD_NONE;
static uint32_t     s_lastTelemMs  = 0;

static void printDiagnosticReport()
{
    TelemetryData t;
    SharedState s;

    portENTER_CRITICAL(&g_sharedMux);
    t = g_sharedState.telemetry;
    s = g_sharedState;
    portEXIT_CRITICAL(&g_sharedMux);

    bool connected = s.hasEverReceived && ((millis() - s.lastPacketTime) < ESPNOW_TIMEOUT_MS);

    Serial.println("\n========== [DIAGNOSTIK SISTEM QUADCOPTER F450] ==========");
    Serial.printf("  Status FSM          : %s\n", getDroneStateName(t.state));
    Serial.printf("  Koneksi Remote      : %s (Paket Terakhir: %lu ms lalu)\n",
                  connected ? "TERHUBUNG" : "TERPUTUS / FAILSAFE",
                  s.hasEverReceived ? (millis() - s.lastPacketTime) : 0);
    Serial.printf("  Status IMU          : %s\n", t.imuHealthy ? "SEHAT (VALID)" : "ERROR / TIMEOUT / GAGAL");
    Serial.printf("  Sikap Roll (deg)    : Aktual = %+5.2f | Target = %+5.2f | Rate = %+5.2f dps\n",
                  t.currentRoll, t.targetRoll, t.rollRate);
    Serial.printf("  Sikap Pitch (deg)   : Aktual = %+5.2f | Target = %+5.2f | Rate = %+5.2f dps\n",
                  t.currentPitch, t.targetPitch, t.pitchRate);
    Serial.printf("  Sikap Yaw Rate      : %+5.2f dps\n", t.yawRate);
    Serial.printf("  Throttle Aktif      : %d µs (Bench Guard: %s)\n",
                  t.throttle, s.benchThrottleEnabled ? "DIBUKA" : "TERKUNCI");
    Serial.printf("  Output Motor (µs)   : FL=%4d | FR=%4d | RL=%4d | RR=%4d\n",
                  t.motorFL, t.motorFR, t.motorRL, t.motorRR);
    Serial.printf("  Gesture State       : Ready=%d | Active=%d\n",
                  t.gestureReady, t.gestureActive);
    Serial.printf("  Kinerja Flight Loop : Periode=%lu µs | Komputasi=%lu µs (Puncak=%lu µs)\n",
                  t.loopDurationUs, t.execDurationUs, t.maxExecDurationUs);
    Serial.printf("  Jitter & Deadlines  : Jitter=%lu µs | Deadline Misses=%lu\n",
                  t.jitterUs, t.deadlineMissCount);
    Serial.printf("  Stack High Water    : Flight=%lu words | Comm=%lu words\n",
                  hFlightTask ? (unsigned long)uxTaskGetStackHighWaterMark(hFlightTask) : 0,
                  hCommTask   ? (unsigned long)uxTaskGetStackHighWaterMark(hCommTask)   : 0);
    Serial.println("==========================================================\n");
}

static void handleSerialCommands()
{
    while (Serial.available())
    {
        char c = Serial.read();
        DroneCommand cmd = CMD_NONE;

        switch (c)
        {
        case '1':
            cmd = CMD_BTN_ARM_LAND;
            Serial.println("[SERIAL] BTN_ARM_LAND");
            break;
        case '2':
            cmd = CMD_BTN_EMERGENCY;
            Serial.println("[SERIAL] BTN_EMERGENCY");
            break;
        case '3':
            cmd = CMD_BTN_READY;
            Serial.println("[SERIAL] BTN_READY");
            break;
        case 'w': case 'W':
            cmd = CMD_MAJU;
            Serial.println("[SERIAL] GESTURE: MAJU");
            break;
        case 's': case 'S':
            cmd = CMD_MUNDUR;
            Serial.println("[SERIAL] GESTURE: MUNDUR");
            break;
        case 'a': case 'A':
            cmd = CMD_KIRI;
            Serial.println("[SERIAL] GESTURE: KIRI");
            break;
        case 'd': case 'D':
            cmd = CMD_KANAN;
            Serial.println("[SERIAL] GESTURE: KANAN");
            break;
        case 'b': case 'B':
        {
            portENTER_CRITICAL(&g_sharedMux);
            g_sharedState.benchThrottleEnabled = !g_sharedState.benchThrottleEnabled;
            bool en = g_sharedState.benchThrottleEnabled;
            portEXIT_CRITICAL(&g_sharedMux);
            if (en)
            {
                Serial.println("[BENCH] PERINGATAN: Kunci throttle serial DIBUKA! Pastikan baling-baling LEPAS!");
            }
            else
            {
                Serial.println("[BENCH] Kunci throttle serial DIKUNCI (Aman).");
            }
            break;
        }
        case '+':
        {
            portENTER_CRITICAL(&g_sharedMux);
            bool allowUp = g_sharedState.benchThrottleEnabled;
            portEXIT_CRITICAL(&g_sharedMux);
            if (allowUp)
            {
                cmd = CMD_THROTTLE_UP;
            }
            else
            {
                Serial.println("[SAFETY] Throttle serial terkunci. Tekan 'b' untuk membuka mode Bench Test.");
            }
            break;
        }
        case '-':
        {
            portENTER_CRITICAL(&g_sharedMux);
            bool allowDn = g_sharedState.benchThrottleEnabled;
            portEXIT_CRITICAL(&g_sharedMux);
            if (allowDn)
            {
                cmd = CMD_THROTTLE_DN;
            }
            else
            {
                Serial.println("[SAFETY] Throttle serial terkunci. Tekan 'b' untuk membuka mode Bench Test.");
            }
            break;
        }
        case 'c': case 'C':
            cmd = CMD_CALIBRATE;
            break;
        case '?':
            printDiagnosticReport();
            break;
        default:
            break;
        }

        if (cmd != CMD_NONE)
        {
            if (xQueueSend(xQueueCommand, &cmd, 0) != pdPASS)
            {
                Serial.println("[COMM WARN] Queue command penuh! Perintah diabaikan.");
            }
        }
    }
}

void commTask(void *pvParameters)
{
    Serial.printf("[COMM TASK] Berjalan pada Core %d (Prioritas %d)\n", xPortGetCoreID(), uxTaskPriorityGet(NULL));

    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Respons cepat 10ms (100Hz)
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    static bool s_isLinkConnected = false;

    for (;;)
    {
        // 1. Proses SEMUA paket masuk dari ESP-NOW di antrean secara instan
        DroneCommand espCmd = CMD_NONE;
        uint16_t espThrottle = 0;

        while (receiver.processIncomingPacket(espCmd, espThrottle))
        {
            uint32_t now = millis();
            portENTER_CRITICAL(&g_sharedMux);
            g_sharedState.lastPacketTime = now;
            g_sharedState.hasEverReceived = true;
            if (espThrottle >= PWM_MIN && espThrottle <= THROTTLE_MAX)
            {
                g_sharedState.throttle = espThrottle;
            }
            portEXIT_CRITICAL(&g_sharedMux);

            // Cetak status terhubung HANYA sekali saat transisi dari terputus -> terhubung
            if (!s_isLinkConnected)
            {
                s_isLinkConnected = true;
                Serial.println();
                Serial.println("========================================================");
                Serial.println("[ESP-NOW STATUS] >>> TERHUBUNG dengan Glove Controller! <<<");
                Serial.println("========================================================");
            }

            // Deduplikasi & eksekusi cepat perintah tombol & gestur
            if (espCmd == CMD_NONE)
            {
                s_lastEspCmd = CMD_NONE;
            }
            else
            {
                bool isNewEvent = (espCmd != s_lastEspCmd);

                if (isNewEvent)
                {
                    s_lastEspCmd = espCmd;

                    Serial.println();
                    Serial.println("========================================================");
                    Serial.printf("[ESP-NOW RECV] DITERIMA DARI CONTROLLER: >>> [ %s ] <<<\n", getDroneCommandName(espCmd));
                    Serial.printf("               Command ID: %d | Throttle: %d\n", espCmd, espThrottle);
                    Serial.println("========================================================");

                    if (xQueueSend(xQueueCommand, &espCmd, 0) != pdPASS)
                    {
                        Serial.println("[COMM WARN] Queue command penuh! Paket dilewati.");
                    }
                }
            }
        }

        // Deteksi jika koneksi terputus: HANYA cetak saat transisi dari terhubung -> terputus
        uint32_t lastPkt = 0;
        bool hasEver = false;
        portENTER_CRITICAL(&g_sharedMux);
        lastPkt = g_sharedState.lastPacketTime;
        hasEver = g_sharedState.hasEverReceived;
        portEXIT_CRITICAL(&g_sharedMux);

        if (s_isLinkConnected && hasEver && (millis() - lastPkt > ESPNOW_TIMEOUT_MS))
        {
            s_isLinkConnected = false;
            Serial.println();
            Serial.println("========================================================");
            Serial.println("[ESP-NOW STATUS] >>> TERPUTUS dari Glove Controller (Signal Timeout)! <<<");
            Serial.println("========================================================");
        }

        // 2. Broadcast status drone ke Glove Controller secara periodik (100ms / 10Hz)
        // Kecepatan 10Hz cukup responsif untuk LED controller tanpa membebani kanal RF (mencegah tabrakan sinyal)
        static uint32_t s_lastStatusTxMs = 0;
        static uint8_t  s_statusSeq = 0;
        uint32_t nowMs = millis();
        if (nowMs - s_lastStatusTxMs >= 100)
        {
            s_lastStatusTxMs = nowMs;
            DroneStatusPacket statusPkt;
            statusPkt.magic = 0xA5;
            portENTER_CRITICAL(&g_sharedMux);
            statusPkt.state = (uint8_t)g_sharedState.droneState;
            portEXIT_CRITICAL(&g_sharedMux);
            statusPkt.seq = s_statusSeq++;

            const uint8_t bcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_now_send(bcastMac, (const uint8_t *)&statusPkt, sizeof(statusPkt));
        }

        // 3. Proses perintah Serial untuk bench testing
        handleSerialCommands();

        // 4. Delay responsif 10ms
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
