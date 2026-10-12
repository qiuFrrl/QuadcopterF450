#include "flight/flight_task.h"
#include "config.h"
#include "imu/imu.h"
#include "pid/pid.h"
#include "motor/motor.h"
#include "shared/shared_types.h"

// ================================================================
//  VARIABEL LOKAL FLIGHT TASK (Core 1 Only — No Cross-Core Races)
// ================================================================
static DroneState droneState       = STATE_DISARMED;

// Target sudut orientasi (derajat)
static float targetAnglePitch      = 0.0f;
static float targetAngleRoll       = 0.0f;
static float targetRateYaw         = 0.0f;

// Sudut terhaluskan (Smooth Ramp)
static float smoothPitch           = 0.0f;
static float smoothRoll            = 0.0f;

static int   currentThrottle       = PWM_ARM;

// Sesi pengenalan gesture satu langkah (READY -> 1 Gerakan -> Netral)
static bool     gestureReady       = false;  // True setelah tombol READY ditekan
static bool     gestureActive      = false;  // True saat gesture sedang dieksekusi
static uint32_t gestureStartTime   = 0;      // Waktu mulai gesture step

// Controlled landing tracking
static int      landingThrottle    = PWM_IDLE;
static uint32_t lastTelemetry      = 0;

// ================================================================
//  KONTROL GERAKAN SATU LANGKAH (TILT TARGET 2-3 DERAJAT)
// ================================================================
static void maju()
{
    targetAnglePitch = +GESTURE_TILT_ANGLE;
    targetAngleRoll  = 0.0f;
    targetRateYaw    = 0.0f;
}

static void mundur()
{
    targetAnglePitch = -GESTURE_TILT_ANGLE;
    targetAngleRoll  = 0.0f;
    targetRateYaw    = 0.0f;
}

static void kiri()
{
    targetAnglePitch = 0.0f;
    targetAngleRoll  = -GESTURE_TILT_ANGLE;
    targetRateYaw    = 0.0f;
}

static void kanan()
{
    targetAnglePitch = 0.0f;
    targetAngleRoll  = +GESTURE_TILT_ANGLE;
    targetRateYaw    = 0.0f;
}

static void idle()
{
    targetAnglePitch = 0.0f;
    targetAngleRoll  = 0.0f;
    targetRateYaw    = 0.0f;
}

// ================================================================
//  SAFETY CHECK SEBELUM ARM
// ================================================================
static bool safetyCheckPass(const SharedState &local)
{
    // 1. IMU harus valid dan drone berada pada posisi rata (< 30 derajat)
    if (isnan(imu.roll) || isnan(imu.pitch) || abs(imu.roll) > 30.0f || abs(imu.pitch) > 30.0f)
    {
        Serial.println("[SAFETY] GAGAL: Drone tidak rata atau data sensor invalid! Ratakan drone sebelum ARM.");
        return false;
    }

    // 2. Controller harus terhubung via ESP-NOW
    bool connected = local.hasEverReceived && ((millis() - local.lastPacketTime) < ESPNOW_TIMEOUT_MS);
    if (!connected)
    {
        Serial.println("[SAFETY] GAGAL: Controller nirkabel belum terhubung!");
        return false;
    }

    // 3. Throttle harus di posisi minimum / aman
    if (currentThrottle > PWM_IDLE + 50)
    {
        Serial.println("[SAFETY] GAGAL: Throttle terlalu tinggi! Turunkan throttle dulu.");
        return false;
    }

    Serial.println("[SAFETY] LULUS: Semua pengecekan aman.");
    return true;
}

// ================================================================
//  TRANSISI STATE
// ================================================================
static void transitionTo(DroneState newState)
{
    DroneState oldState = droneState;
    droneState = newState;

    Serial.printf("\n[STATE] %s → %s\n", getDroneStateName(oldState), getDroneStateName(newState));

    switch (newState)
    {
    case STATE_DISARMED:
        idle();
        gestureReady = false;
        gestureActive = false;
        currentThrottle = PWM_ARM;
        controller.reset();
        motor.cut();
        digitalWrite(LED_PIN, HIGH); // LED solid = DISARMED (aman)
        Serial.println("[STATUS] Motor DIMATIKAN. Drone aman.");
        break;

    case STATE_ARMED:
        idle();
        gestureReady = false;
        gestureActive = false;
        currentThrottle = PWM_IDLE;
        controller.reset();
        digitalWrite(LED_PIN, LOW); // LED mati = ARMED
        Serial.println("[STATUS] Motor ARMED (idle floor). Naikkan throttle untuk terbang.");
        break;

    case STATE_FLYING:
        idle();
        gestureReady = false;
        gestureActive = false;
        Serial.println("[STATUS] FLYING. Siap menerima satu gesture via tombol READY.");
        break;

    case STATE_LANDING:
        idle();
        gestureReady = false;
        gestureActive = false;
        landingThrottle = currentThrottle;
        Serial.println("[STATUS] Controlled Landing dimulai. Throttle diturunkan bertahap.");
        Serial.println("[PERINGATAN] Tanpa sensor ketinggian, ini BUKAN pendaratan otomatis terjamin.");
        Serial.println("[STATUS] Tekan BTN_ARM_LAND lagi untuk KONFIRMASI MANUAL setelah menyentuh tanah!");
        break;

    case STATE_EMERGENCY:
        idle();
        gestureReady = false;
        gestureActive = false;
        controller.reset();
        motor.cut();
        currentThrottle = PWM_ARM;
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("!!!     EMERGENCY: MOTOR DI-CUT!    !!!");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        break;
    }
}

// ================================================================
//  PROSES STATE MACHINE & FAILSAFE
// ================================================================
static void processStateMachine(DroneCommand cmd, const SharedState &local)
{
    // ────────────────────────────────────────────────────────────
    //  1. EMERGENCY: Prioritas utama, aktif dari state MANAPUN
    // ────────────────────────────────────────────────────────────
    if (cmd == CMD_BTN_EMERGENCY)
    {
        transitionTo(STATE_EMERGENCY);
        return;
    }

    // ────────────────────────────────────────────────────────────
    //  2. LOCAL SAFETY CHECK: Auto-cut jika orientasi ekstrim (> 45°)
    // ────────────────────────────────────────────────────────────
    if (droneState == STATE_FLYING || droneState == STATE_LANDING || droneState == STATE_ARMED)
    {
        if (isnan(imu.roll) || isnan(imu.pitch) ||
            abs(imu.roll) > IMU_MAX_TILT_ANGLE || abs(imu.pitch) > IMU_MAX_TILT_ANGLE)
        {
            Serial.println("[SAFETY] Sudut melebihi 45° atau sensor invalid! Emergency auto-cut!");
            transitionTo(STATE_EMERGENCY);
            return;
        }
    }

    // ────────────────────────────────────────────────────────────
    //  3. LOCAL FAILSAFE: Koneksi putus saat terbang
    // ────────────────────────────────────────────────────────────
    bool connected = local.hasEverReceived && ((millis() - local.lastPacketTime) < ESPNOW_TIMEOUT_MS);
    if ((droneState == STATE_FLYING || droneState == STATE_ARMED) && !connected)
    {
        Serial.println("[FAILSAFE] Sinyal controller terputus! Mengaktifkan controlled landing...");
        transitionTo(STATE_LANDING);
        return;
    }

    // ────────────────────────────────────────────────────────────
    //  4. PERINTAH BENCH TEST (SERIAL)
    // ────────────────────────────────────────────────────────────
    if (cmd == CMD_THROTTLE_UP)
    {
        if (droneState != STATE_DISARMED && droneState != STATE_EMERGENCY && droneState != STATE_LANDING)
        {
            currentThrottle = min(currentThrottle + 25, THROTTLE_MAX);
            Serial.printf("[THROTTLE] %d µs\n", currentThrottle);
        }
        return;
    }
    if (cmd == CMD_THROTTLE_DN)
    {
        if (droneState != STATE_DISARMED && droneState != STATE_EMERGENCY && droneState != STATE_LANDING)
        {
            currentThrottle = max(currentThrottle - 25, (int)PWM_IDLE);
            Serial.printf("[THROTTLE] %d µs\n", currentThrottle);
        }
        return;
    }
    if (cmd == CMD_CALIBRATE)
    {
        if (droneState == STATE_DISARMED)
        {
            Serial.println("[IMU] Mulai kalibrasi...");
            imu.calibrate();
        }
        else
        {
            Serial.println("[IMU] Kalibrasi ditolak: drone harus dalam status DISARMED!");
        }
        return;
    }
    if (cmd == CMD_STATUS)
    {
        Serial.printf("[INFO] State=%s Thr=%d Roll=%.1f Pitch=%.1f Ready=%d Active=%d Conn=%d\n",
                      getDroneStateName(droneState), currentThrottle,
                      imu.roll, imu.pitch,
                      gestureReady, gestureActive, connected);
        return;
    }

    // ────────────────────────────────────────────────────────────
    //  5. STATE MACHINE LOGIC
    // ────────────────────────────────────────────────────────────
    switch (droneState)
    {
    case STATE_DISARMED:
        motor.cut();
        // Tombol 1 saat DISARMED -> Validasi Safety -> Masuk ARMED
        if (cmd == CMD_BTN_ARM_LAND)
        {
            if (safetyCheckPass(local))
            {
                transitionTo(STATE_ARMED);
            }
        }
        break;

    case STATE_ARMED:
        // Tombol 1 saat ARMED -> Kembali ke DISARMED (batal terbang)
        if (cmd == CMD_BTN_ARM_LAND)
        {
            transitionTo(STATE_DISARMED);
            break;
        }

        // Throttle dinaikkan melewati ambang batas -> otomatis masuk FLYING
        if (currentThrottle > PWM_IDLE + 100)
        {
            transitionTo(STATE_FLYING);
        }
        break;

    case STATE_FLYING:
        // Tombol 1 saat FLYING -> Mulai Controlled Landing
        if (cmd == CMD_BTN_ARM_LAND)
        {
            transitionTo(STATE_LANDING);
            break;
        }

        // Tombol READY -> Buka SATU sesi gesture
        if (cmd == CMD_BTN_READY)
        {
            if (!gestureActive)
            {
                gestureReady = true;
                Serial.println("[GESTURE] Sesi READY aktif! Kirim satu gesture (MAJU/MUNDUR/KIRI/KANAN).");
            }
        }

        // Gesture hanya diproses jika sesi READY aktif
        if (gestureReady && !gestureActive)
        {
            bool validGesture = false;
            switch (cmd)
            {
            case CMD_MAJU:   maju();   validGesture = true; break;
            case CMD_MUNDUR: mundur(); validGesture = true; break;
            case CMD_KIRI:   kiri();   validGesture = true; break;
            case CMD_KANAN:  kanan();  validGesture = true; break;
            default: break;
            }

            if (validGesture)
            {
                gestureActive = true;
                gestureReady = false; // Kunci sesi agar hanya 1 gesture per penekanan READY
                gestureStartTime = millis();
                Serial.printf("[GESTURE] Dieksekusi: Kemiringan %.1f° selama %dms\n",
                              GESTURE_TILT_ANGLE, GESTURE_STEP_DURATION_MS);
            }
        }

        // Otomatis kembalikan target ke netral setelah durasi langkah habis
        if (gestureActive)
        {
            if (millis() - gestureStartTime >= GESTURE_STEP_DURATION_MS)
            {
                idle();
                gestureActive = false;
                Serial.println("[GESTURE] Selesai. Target orientasi kembali netral.");
            }
        }

        // Jika pilot menurunkan throttle sangat rendah -> masuk LANDING
        if (currentThrottle <= LANDING_THROTTLE_FLOOR)
        {
            Serial.println("[AUTO] Throttle sangat rendah, memulai landing...");
            transitionTo(STATE_LANDING);
        }
        break;

    case STATE_LANDING:
        // Penurunan throttle secara bertahap
        if (landingThrottle > LANDING_THROTTLE_FLOOR)
        {
            landingThrottle -= LANDING_THROTTLE_DROP_RATE;
            if (landingThrottle < LANDING_THROTTLE_FLOOR)
            {
                landingThrottle = LANDING_THROTTLE_FLOOR;
            }
        }
        currentThrottle = landingThrottle;
        idle(); // Target orientasi selalu datar saat landing

        // Tombol 1 saat LANDING -> KONFIRMASI MANUAL bahwa drone telah menyentuh tanah
        if (cmd == CMD_BTN_ARM_LAND)
        {
            Serial.println("[CONFIRM] Pilot konfirmasi: drone sudah di tanah. Motor dimatikan.");
            transitionTo(STATE_DISARMED);
        }
        break;

    case STATE_EMERGENCY:
        motor.cut();
        // Reset emergency hanya melalui tombol BTN_ARM_LAND
        if (cmd == CMD_BTN_ARM_LAND)
        {
            Serial.println("[EMERGENCY] Reset darurat. Kembali ke DISARMED.");
            transitionTo(STATE_DISARMED);
        }
        break;
    }
}

// ================================================================
//  MAIN LOOP FLIGHT TASK (Core 1, 250Hz Strict Timing)
// ================================================================
void flightTask(void *pvParameters)
{
    Serial.printf("[FLIGHT TASK] Berjalan pada Core %d (Prioritas %d)\n",
                  xPortGetCoreID(), uxTaskPriorityGet(NULL));

    uint32_t lastLoopMicros = micros();

    for (;;)
    {
        // 1. Timing ketat 250Hz (4000µs)
        while ((micros() - lastLoopMicros) < LOOP_TIME_US)
        {
            uint32_t elapsed = micros() - lastLoopMicros;
            if (LOOP_TIME_US > elapsed && (LOOP_TIME_US - elapsed) > 1500)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        uint32_t nowMicros = micros();
        float dt = (nowMicros - lastLoopMicros) * 1e-6f;
        lastLoopMicros = nowMicros;

        if (dt <= 0.0f || dt > 0.05f)
        {
            dt = LOOP_TIME_S;
        }

        // 2. Ambil perintah diskrit dari queue (non-blocking)
        DroneCommand cmd = CMD_NONE;
        xQueueReceive(xQueueCommand, &cmd, 0);

        // 3. Salin snapshot data bersama secara atomik via spinlock (< 1µs)
        SharedState localShared;
        portENTER_CRITICAL(&g_sharedMux);
        localShared = g_sharedState;
        g_sharedState.droneState = droneState; // Bagikan status ke Core 0
        portEXIT_CRITICAL(&g_sharedMux);

        // Update throttle dari remote jika diizinkan oleh state saat ini
        if (droneState == STATE_ARMED || droneState == STATE_FLYING)
        {
            if (localShared.throttle >= PWM_IDLE && localShared.throttle <= THROTTLE_MAX)
            {
                currentThrottle = localShared.throttle;
            }
        }

        // 4. Proses State Machine, Safety & Failsafe lokal
        processStateMachine(cmd, localShared);

        // 5. Update sensor IMU (MPU6050)
        imu.update(dt);

        // 6. Haluskan target kemiringan (Ramping)
        smoothPitch += (targetAnglePitch - smoothPitch) * TILT_RAMP_RATE;
        smoothRoll  += (targetAngleRoll  - smoothRoll)  * TILT_RAMP_RATE;

        // 7. Hitung koreksi Cascade PID & Mixer Motor ESC
        bool motorsActive = (droneState == STATE_ARMED ||
                             droneState == STATE_FLYING ||
                             droneState == STATE_LANDING);

        int throttleInput = motorsActive ? currentThrottle : PWM_ARM;

        if (!motorsActive || throttleInput < THROTTLE_CUTOFF)
        {
            controller.reset();
            motor.cut();
        }
        else
        {
            controller.update(smoothRoll, smoothPitch, targetRateYaw,
                              imu.roll, imu.pitch,
                              imu.rollRate, imu.pitchRate, imu.yawRate,
                              dt);

            motor.mix(throttleInput,
                      controller.outRoll,
                      controller.outPitch,
                      controller.outYaw);
        }

        // 8. Telemetri setiap 50ms (20Hz)
        if (millis() - lastTelemetry >= 50)
        {
            lastTelemetry = millis();
            Serial.printf("[%s | Thr:%4d] Tgt(P:%+4.1f R:%+4.1f) IMU(P:%+4.1f R:%+4.1f) Mot(%4d %4d %4d %4d)%s%s\n",
                          getDroneStateName(droneState),
                          throttleInput,
                          smoothPitch, smoothRoll,
                          imu.pitch, imu.roll,
                          motor.getFL(), motor.getFR(), motor.getRL(), motor.getRR(),
                          gestureReady  ? " [READY]"  : "",
                          gestureActive ? " [GESTURE]" : "");
        }
    }
}
