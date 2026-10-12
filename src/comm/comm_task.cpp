#include "comm/comm_task.h"
#include "ReceiverEspNow/ReceiverEspNow.h"

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
            Serial.println("[SERIAL] MAJU");
            break;
        case 's': case 'S':
            cmd = CMD_MUNDUR;
            Serial.println("[SERIAL] MUNDUR");
            break;
        case 'a': case 'A':
            cmd = CMD_KIRI;
            Serial.println("[SERIAL] KIRI");
            break;
        case 'd': case 'D':
            cmd = CMD_KANAN;
            Serial.println("[SERIAL] KANAN");
            break;
        case '+':
            cmd = CMD_THROTTLE_UP;
            break;
        case '-':
            cmd = CMD_THROTTLE_DN;
            break;
        case 'c': case 'C':
            cmd = CMD_CALIBRATE;
            break;
        case '?':
            cmd = CMD_STATUS;
            break;
        default:
            break;
        }

        if (cmd != CMD_NONE)
        {
            if (xQueueSend(xQueueCommand, &cmd, 0) != pdPASS)
            {
                Serial.println("[COMM WARN] Queue command penuh!");
            }
        }
    }
}

void commTask(void *pvParameters)
{
    Serial.printf("[COMM TASK] Berjalan pada Core %d (Prioritas %d)\n",
                  xPortGetCoreID(), uxTaskPriorityGet(NULL));

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(COMM_TASK_PERIOD_MS);

    for (;;)
    {
        // 1. Proses paket masuk dari ESP-NOW
        DroneCommand espCmd = CMD_NONE;
        uint16_t espThrottle = 0;

        if (receiver.processIncomingPacket(espCmd, espThrottle))
        {
            portENTER_CRITICAL(&g_sharedMux);
            g_sharedState.lastPacketTime = millis();
            g_sharedState.hasEverReceived = true;
            if (espThrottle >= PWM_MIN && espThrottle <= PWM_MAX)
            {
                g_sharedState.throttle = espThrottle;
            }
            portEXIT_CRITICAL(&g_sharedMux);

            if (espCmd != CMD_NONE)
            {
                if (xQueueSend(xQueueCommand, &espCmd, 0) != pdPASS)
                {
                    Serial.println("[COMM WARN] Queue command penuh!");
                }
            }
        }

        // 2. Proses perintah Serial untuk bench testing
        handleSerialCommands();

        // 3. Delay hingga siklus 20ms (50Hz) berikutnya
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
