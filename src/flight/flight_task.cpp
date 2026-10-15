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
static float autoThrottle          = PWM_IDLE;
static bool  takeoffRequested      = false;

// Sesi pengenalan gesture satu langkah (READY -> 1 Gerakan -> Netral)
static bool     gestureReady       = false;  // True setelah tombol READY ditekan
static bool     gestureActive      = false;  // True saat gesture sedang dieksekusi
static uint32_t gestureStartTime   = 0;      // Waktu mulai gesture step
static float    landingThrottle    = PWM_IDLE;
static uint32_t landingStartTime   = 0;
static uint32_t floorHoldStartTime = 0;
static uint32_t s_armTransitionTimeMs = 0;

// Diagnostik Flight Control Loop
static uint32_t maxExecDurationUs  = 0;
static uint32_t deadlineMissCount  = 0;

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
    // 1. Cek kesehatan sensor IMU
    if (!imu.isHealthy())
    {
        Serial.println("[SAFETY] GAGAL: Sensor IMU tidak terdeteksi atau bermasalah!");
        return false;
    }

    // 2. Controller harus terhubung via ESP-NOW
    bool connected = local.hasEverReceived && ((millis() - local.lastPacketTime) < ESPNOW_TIMEOUT_MS);
    if (!connected)
    {
        Serial.println("[SAFETY] GAGAL: Controller nirkabel belum terhubung!");
        return false;
    }

    Serial.println("[SAFETY] LULUS: ARM diizinkan.");
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
        smoothPitch = 0.0f;
        smoothRoll = 0.0f;
        gestureReady = false;
        gestureActive = false;
        currentThrottle = PWM_ARM;
        autoThrottle = PWM_IDLE;
        takeoffRequested = false;
        landingStartTime = 0;
        floorHoldStartTime = 0;
        controller.reset();
        motor.cut();
        Serial.println("[STATUS] Motor DIMATIKAN. Drone aman.");
        break;

    case STATE_ARMED:
        idle();
        smoothPitch = 0.0f;
        smoothRoll = 0.0f;
        gestureReady = false;
        gestureActive = false;
        currentThrottle = PWM_IDLE;
        autoThrottle = PWM_IDLE;
        takeoffRequested = true; // Langsung spool up menuju hover untuk uji rig PID
        landingStartTime = 0;
        floorHoldStartTime = 0;
        s_armTransitionTimeMs = millis();
        controller.reset();
        Serial.printf("[STATUS] Motor ARMED @ %d us. Naik bertahap menuju HOVER %d us...\n", PWM_IDLE, HOVER_THROTTLE);
        Serial.println("[STATUS] Cascade PID aktif pada rig. Siap menerima gesture belok/arah.");
        Serial.println("[STATUS] Tekan EMERGENCY untuk mematikan semua motor.");
        break;

    case STATE_FLYING:
        idle();
        gestureReady = false;
        gestureActive = false;
        landingStartTime = 0;
        floorHoldStartTime = 0;
        Serial.println("[STATUS] FLYING. Siap menerima satu gesture via tombol READY.");
        break;

    case STATE_LANDING:
        idle();
        gestureReady = false;
        gestureActive = false;
        landingThrottle = currentThrottle;
        autoThrottle = PWM_IDLE;
        takeoffRequested = false;
        landingStartTime = millis();
        floorHoldStartTime = 0;
        Serial.println("[STATUS] Controlled Landing dimulai. Throttle diturunkan bertahap.");
        Serial.println("[STATUS] Tekan BTN_ARM_LAND untuk KONFIRMASI MANUAL setelah menyentuh tanah.");
        break;

    case STATE_EMERGENCY:
        idle();
        gestureReady = false;
        gestureActive = false;
        landingStartTime = 0;
        floorHoldStartTime = 0;
        controller.reset();
        motor.cut();
        currentThrottle = PWM_ARM;
        autoThrottle = PWM_IDLE;
        takeoffRequested = false;
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("!!!     EMERGENCY: MOTOR DI-CUT!    !!!");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        break;
    }
}

// ================================================================
//  PROSES STATE MACHINE & FAILSAFE
// ================================================================
static void processStateMachine(DroneCommand cmd, const SharedState &local, float dt)
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
    //  2. LOCAL SAFETY CHECK: Auto-cut jika orientasi ekstrim (> 45°) atau IMU mati
    // ────────────────────────────────────────────────────────────
    // IMU auto-cut dinonaktifkan untuk mode uji motor sederhana
    // Jika IMU mati, motor tetap jalan di base throttle 1050
    if (droneState == STATE_FLYING || droneState == STATE_LANDING)
    {
        if (imu.isHealthy() && !isnan(imu.roll) && !isnan(imu.pitch) &&
            (abs(imu.roll) > IMU_MAX_TILT_ANGLE || abs(imu.pitch) > IMU_MAX_TILT_ANGLE))
        {
            Serial.println("[SAFETY] Sudut melebihi 45 derajat! Emergency auto-cut!");
            transitionTo(STATE_EMERGENCY);
            return;
        }
    }

    // ────────────────────────────────────────────────────────────
    //  3. LOCAL FAILSAFE: Koneksi putus
    // ────────────────────────────────────────────────────────────
    bool connected = local.hasEverReceived && ((millis() - local.lastPacketTime) < ESPNOW_TIMEOUT_MS);
    if (!connected)
    {
        if (droneState == STATE_ARMED)
        {
            Serial.println("[FAILSAFE] Sinyal controller terputus saat ARMED! Motor dimatikan.");
            transitionTo(STATE_DISARMED);
            return;
        }
        else if (droneState == STATE_FLYING)
        {
            Serial.println("[FAILSAFE] Sinyal controller terputus saat FLYING! Mengaktifkan controlled landing...");
            transitionTo(STATE_LANDING);
            return;
        }
    }

    // ────────────────────────────────────────────────────────────
    //  4. PERINTAH BENCH TEST (SERIAL)
    // ────────────────────────────────────────────────────────────
    if (cmd == CMD_THROTTLE_UP)
    {
        if (droneState != STATE_DISARMED && droneState != STATE_EMERGENCY && droneState != STATE_LANDING)
        {
            currentThrottle = min(currentThrottle + 25, THROTTLE_MAX);
            Serial.printf("[THROTTLE BENCH] %d µs\n", currentThrottle);
        }
        return;
    }
    if (cmd == CMD_THROTTLE_DN)
    {
        if (droneState != STATE_DISARMED && droneState != STATE_EMERGENCY && droneState != STATE_LANDING)
        {
            currentThrottle = max(currentThrottle - 25, (int)PWM_IDLE);
            Serial.printf("[THROTTLE BENCH] %d µs\n", currentThrottle);
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
        // Tombol ARM lagi saat ARMED -> Kembali ke DISARMED (dengan debounce 1000ms agar burst paket tidak langsung membatalkan)
        if (cmd == CMD_BTN_ARM_LAND)
        {
            if (millis() - s_armTransitionTimeMs >= 1000)
            {
                Serial.println("[ARM] Tombol ARM ditekan kembali setelah 1 detik. Kembali ke DISARMED.");
                transitionTo(STATE_DISARMED);
            }
            break;
        }

        // Auto-throttle naik mendekati hover -> otomatis masuk FLYING
        if (currentThrottle >= PWM_IDLE + 80)
        {
            transitionTo(STATE_FLYING);
        }

        // Langsung terima gesture saat ARMED (baik sebelum maupun sesudah masuk FLYING)
        if (cmd == CMD_MAJU || cmd == CMD_MUNDUR || cmd == CMD_KIRI || cmd == CMD_KANAN)
        {
            switch (cmd)
            {
            case CMD_MAJU:   maju();   break;
            case CMD_MUNDUR: mundur(); break;
            case CMD_KIRI:   kiri();   break;
            case CMD_KANAN:  kanan();  break;
            default: break;
            }
            gestureActive = true;
            gestureStartTime = millis();
            Serial.printf("[GESTURE] Diterima saat ARMED! Target Throttle %d us (Hover+%d us) selama %d ms\n",
                          HOVER_THROTTLE + GESTURE_THROTTLE_BOOST, GESTURE_THROTTLE_BOOST, GESTURE_STEP_DURATION_MS);
        }
        break;

    case STATE_FLYING:
        // Tombol 1 saat FLYING -> Mulai Controlled Landing
        if (cmd == CMD_BTN_ARM_LAND)
        {
            transitionTo(STATE_LANDING);
            break;
        }

        // Langsung eksekusi gesture arah tanpa butuh penekanan tombol sesi terpisah
        if (cmd == CMD_MAJU || cmd == CMD_MUNDUR || cmd == CMD_KIRI || cmd == CMD_KANAN)
        {
            switch (cmd)
            {
            case CMD_MAJU:   maju();   break;
            case CMD_MUNDUR: mundur(); break;
            case CMD_KIRI:   kiri();   break;
            case CMD_KANAN:  kanan();  break;
            default: break;
            }
            gestureActive = true;
            gestureStartTime = millis();
            Serial.printf("[GESTURE] Dieksekusi: Arah %s, Target Throttle %d us (Hover+%d us) selama %d ms\n",
                          (cmd == CMD_MAJU ? "MAJU" : (cmd == CMD_MUNDUR ? "MUNDUR" : (cmd == CMD_KIRI ? "KIRI" : "KANAN"))),
                          HOVER_THROTTLE + GESTURE_THROTTLE_BOOST, GESTURE_THROTTLE_BOOST, GESTURE_STEP_DURATION_MS);
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
            landingThrottle -= LANDING_THROTTLE_DROP_PER_SEC * dt;
            if (landingThrottle < LANDING_THROTTLE_FLOOR)
            {
                landingThrottle = LANDING_THROTTLE_FLOOR;
            }
        }
        else
        {
            if (floorHoldStartTime == 0)
            {
                floorHoldStartTime = millis();
            }
        }
        currentThrottle = (int)(landingThrottle + 0.5f);
        idle(); // Target orientasi selalu datar saat landing

        // 1. Tombol 1 saat LANDING -> KONFIRMASI MANUAL bahwa drone telah menyentuh tanah
        if (cmd == CMD_BTN_ARM_LAND)
        {
            if (landingStartTime > 0 && (millis() - landingStartTime >= LANDING_MANUAL_CONFIRM_MIN_MS))
            {
                Serial.println("[CONFIRM] Pilot konfirmasi: drone sudah di tanah. Motor dimatikan.");
                transitionTo(STATE_DISARMED);
            }
            else
            {
                Serial.println("[LANDING] Konfirmasi manual diabaikan: tunggu minimal 1,5 detik sejak landing dimulai.");
            }
            break;
        }

        // 2. FAILSAFE AUTO-CUTOFF: Jika tidak ada konfirmasi manual atau sinyal hilang,
        // matikan motor setelah menahan floor atau batas waktu landing habis
        if ((floorHoldStartTime > 0 && (millis() - floorHoldStartTime >= FAILSAFE_FLOOR_HOLD_MS)) ||
            (landingStartTime > 0 && (millis() - landingStartTime >= FAILSAFE_LANDING_TIMEOUT_MS)))
        {
            Serial.println("[FAILSAFE] Batas waktu pendaratan selesai. Motor dimatikan otomatis.");
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

    // Kembalikan orientasi dan throttle ke normal setelah durasi gesture selesai (1 detik)
    if (gestureActive && (millis() - gestureStartTime >= GESTURE_STEP_DURATION_MS))
    {
        idle();
        gestureActive = false;
        Serial.println("[GESTURE] Selesai 1 detik. Orientasi netral dan throttle kembali ke hover normal.");
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
        uint32_t loopStart = micros();
        uint32_t loopDurationUs = loopStart - lastLoopMicros;
        lastLoopMicros = loopStart;

        float dt = loopDurationUs * 1e-6f;
        if (dt <= 0.0f || dt > 0.05f)
        {
            dt = LOOP_TIME_S;
        }

        uint32_t jitter = (loopDurationUs > LOOP_TIME_US) ? (loopDurationUs - LOOP_TIME_US) : (LOOP_TIME_US - loopDurationUs);
        if (loopDurationUs > (LOOP_TIME_US + 500))
        {
            deadlineMissCount++;
        }

        // 1. Ambil perintah diskrit dari queue (non-blocking)
        DroneCommand cmd = CMD_NONE;
        xQueueReceive(xQueueCommand, &cmd, 0);

        // 2. Salin snapshot data bersama secara atomik via spinlock (< 1µs)
        SharedState localShared;
        portENTER_CRITICAL(&g_sharedMux);
        localShared = g_sharedState;
        g_sharedState.droneState = droneState; // Bagikan status ke Core 0
        portEXIT_CRITICAL(&g_sharedMux);

        if (!localShared.benchThrottleEnabled &&
            (droneState == STATE_ARMED || droneState == STATE_FLYING))
        {
            // Target dasar adalah HOVER_THROTTLE (1300 us)
            // Saat gesture arah aktif, throttle otomatis dinaikkan di atas hover (1300 + 80 = 1380 us)
            float target = (float)HOVER_THROTTLE;
            if (gestureActive)
            {
                target += (float)GESTURE_THROTTLE_BOOST;
            }

            if (autoThrottle < target)
            {
                autoThrottle += TAKEOFF_RAMP_PER_SEC * dt;
                if (autoThrottle > target) autoThrottle = target;
            }
            else if (autoThrottle > target)
            {
                autoThrottle -= TAKEOFF_RAMP_PER_SEC * dt;
                if (autoThrottle < target) autoThrottle = target;
            }
            currentThrottle = (int)(autoThrottle + 0.5f);
        }

        // 3. Update sensor IMU (MPU6050) dengan pembacaan fisik terbaru
        imu.update(dt);

        // 4. Proses State Machine, Safety & Failsafe lokal dengan data sensor terkini
        processStateMachine(cmd, localShared, dt);

        // 5. Haluskan target kemiringan (Ramping)
        smoothPitch += (targetAnglePitch - smoothPitch) * TILT_RAMP_RATE;
        smoothRoll  += (targetAngleRoll  - smoothRoll)  * TILT_RAMP_RATE;

        // Batasi target sudut demi keselamatan
        smoothPitch = constrain(smoothPitch, -TARGET_ANGLE_LIMIT, TARGET_ANGLE_LIMIT);
        smoothRoll  = constrain(smoothRoll,  -TARGET_ANGLE_LIMIT, TARGET_ANGLE_LIMIT);

        // 6. Hitung koreksi Cascade PID & Mixer Motor ESC
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
            if (throttleInput < PWM_IDLE + 50)
            {
                // Saat idle awal di tanah sebelum spool up
                controller.reset();
                motor.mix(throttleInput, 0.0f, 0.0f, 0.0f);
            }
            else
            {
                // Saat terbang / hover / uji rig PID: Aktifkan kendali Cascade PID!
                controller.update(smoothRoll, smoothPitch, targetRateYaw,
                                  imu.roll, imu.pitch,
                                  imu.rollRate, imu.pitchRate, imu.yawRate,
                                  dt);

                motor.mix(throttleInput,
                          controller.outRoll,
                          controller.outPitch,
                          controller.outYaw);
            }
        }

        // 7. Ukur durasi komputasi
        uint32_t compEnd = micros();
        uint32_t execDurationUs = compEnd - loopStart;
        if (execDurationUs > maxExecDurationUs)
        {
            maxExecDurationUs = execDurationUs;
        }

        // 8. Kirim paket telemetri non-blocking ke shared state untuk dicetak Core 0
        TelemetryData telem;
        telem.state             = droneState;
        telem.throttle          = throttleInput;
        telem.targetPitch       = smoothPitch;
        telem.targetRoll        = smoothRoll;
        telem.currentPitch      = imu.pitch;
        telem.currentRoll       = imu.roll;
        telem.rollRate          = imu.rollRate;
        telem.pitchRate         = imu.pitchRate;
        telem.yawRate           = imu.yawRate;
        telem.motorFL           = motor.getFL();
        telem.motorFR           = motor.getFR();
        telem.motorRL           = motor.getRL();
        telem.motorRR           = motor.getRR();
        telem.gestureReady      = gestureReady;
        telem.gestureActive     = gestureActive;
        telem.imuHealthy        = imu.isHealthy();
        telem.loopDurationUs    = loopDurationUs;
        telem.execDurationUs    = execDurationUs;
        telem.maxExecDurationUs = maxExecDurationUs;
        telem.jitterUs          = jitter;
        telem.deadlineMissCount = deadlineMissCount;

        portENTER_CRITICAL(&g_sharedMux);
        g_sharedState.telemetry  = telem;
        g_sharedState.droneState = droneState;
        portEXIT_CRITICAL(&g_sharedMux);

        // 9. Precision Timing 250Hz (4000µs)
        while ((micros() - loopStart) < LOOP_TIME_US)
        {
            uint32_t remaining = LOOP_TIME_US - (micros() - loopStart);
            if (remaining > 2000)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
}


