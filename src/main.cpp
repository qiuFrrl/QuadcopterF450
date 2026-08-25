// ============================================================
//  MAIN.CPP — Flight Controller Entry Point
//  Stabilize mode, input via Serial (sementara, sebelum ESP-NOW)
//
//  Serial command:
//    a         → arm
//    d         → disarm
//    t<nilai>  → throttle PWM (1000-1800), contoh: t1200
//    r<nilai>  → roll setpoint deg (-20~20), contoh: r5
//    p<nilai>  → pitch setpoint deg (-20~20), contoh: p-3
//    y<nilai>  → yaw rate setpoint (-50~50), contoh: y10
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "pid/pid.h"

// PID instances
PID pidRoll(ROLL_KP, ROLL_KI, ROLL_KD);
PID pidPitch(PITCH_KP, PITCH_KI, PITCH_KD);
PID pidYaw(YAW_KP, YAW_KI, YAW_KD);

// State
bool armed = false;
int throttle = PWM_IDLE;
float setRoll = 0;
float setPitch = 0;
float setYaw = 0;

unsigned long lastUs = 0;
unsigned long lastLog = 0;

// ── Serial parser ─────────────────────────────────────────
void parseSerial()
{
  while (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0)
      return;

    char c = cmd[0];
    float val = cmd.substring(1).toFloat();

    switch (c)
    {
    case 'a':
      if (!armed)
      {
        armed = true;
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        Serial.println("[ARM]");
      }
      break;

    case 'd':
      armed = false;
      throttle = PWM_IDLE;
      setRoll = setPitch = setYaw = 0;
      motor.cut();
      Serial.println("[DISARM]");
      break;

    case 't':
      throttle = constrain((int)val, PWM_MIN, THROTTLE_MAX);
      Serial.printf("[THR] %d\n", throttle);
      break;

    case 'r':
      setRoll = constrain(val, -20.0f, 20.0f);
      Serial.printf("[ROLL sp] %.1f\n", setRoll);
      break;

    case 'p':
      setPitch = constrain(val, -20.0f, 20.0f);
      Serial.printf("[PITCH sp] %.1f\n", setPitch);
      break;

    case 'y':
      setYaw = constrain(val, -50.0f, 50.0f);
      Serial.printf("[YAW sp] %.1f\n", setYaw);
      break;

    default:
      Serial.println("cmd: a d t<pwm> r<deg> p<deg> y<rate>");
    }
  }
}

// ── Setup ─────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  delay(500);

  if (!imu.begin())
  {
    Serial.println("[ERROR] MPU6050 tidak ditemukan!");
    while (1)
      ;
  }
  Serial.println("[OK] IMU ready");

  motor.begin();
  motor.arm();
  Serial.println("[OK] ESC armed — tunggu 3 detik...");
  delay(3000);

  lastUs = micros();
  Serial.println("[READY] ketik 'a' arm, 't1200' throttle");
}

// ── Loop ──────────────────────────────────────────────────
void loop()
{
  unsigned long now = micros();
  float dt = (now - lastUs) / 1e6f;
  if (dt < (1.0f / LOOP_HZ))
    return;
  lastUs = now;

  parseSerial();
  imu.update(dt);

  // Safety: auto-disarm jika terlalu miring
  if (armed && (fabsf(imu.roll) > 60.0f || fabsf(imu.pitch) > 60.0f))
  {
    armed = false;
    motor.cut();
    Serial.println("[DISARM] angle ekstrem!");
    return;
  }

  if (!armed)
  {
    motor.cut();
    return;
  }

  // PID
  float rollOut = pidRoll.compute(setRoll, imu.roll, imu.rollRate, dt, 1.0f);
  float pitchOut = pidPitch.compute(setPitch, imu.pitch, imu.pitchRate, dt, 1.0f);
  float yawOut = pidYaw.compute(setYaw, imu.yawRate, imu.yawRate, dt, 0.5f);

  motor.mix(throttle, rollOut, pitchOut, yawOut);

  // Log 10Hz
  if (millis() - lastLog > 100)
  {
    lastLog = millis();
    Serial.printf("R:%.1f P:%.1f Y:%.1f | sp R:%.1f P:%.1f Y:%.1f | thr:%d\n",
                  imu.roll, imu.pitch, imu.yaw,
                  setRoll, setPitch, setYaw, throttle);
  }
}