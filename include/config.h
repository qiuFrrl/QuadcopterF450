#pragma once

// ============================================================
//  CONFIG.H — semua setting di satu tempat
// ============================================================

// -- PIN --
#define PIN_FL 27
#define PIN_FR 25
#define PIN_RL 12
#define PIN_RR 33
#define SDA_PIN 21
#define SCL_PIN 22

// -- MPU6050 OFFSET --
#define AX_OFFSET 749
#define AY_OFFSET -359
#define AZ_OFFSET 1204
#define GX_OFFSET 5873
#define GY_OFFSET 194
#define GZ_OFFSET -211

// -- MOUNTING ANGLE TRIM --
#define ROLL_TRIM 1.98f
#define PITCH_TRIM -2.00f

// -- COMPLEMENTARY FILTER --
#define CF_ALPHA 0.96f

// -- PID GAINS --
#define ROLL_KP 1.5f
#define ROLL_KI 0.05f
#define ROLL_KD 0.8f

#define PITCH_KP 1.5f
#define PITCH_KI 0.05f
#define PITCH_KD 0.8f

#define YAW_KP 2.0f
#define YAW_KI 0.02f
#define YAW_KD 0.0f

// -- MOTOR LIMITS --
#define PWM_MIN 1000
#define PWM_MAX 1500
#define PWM_IDLE 1150
#define PWM_ARM 1000
#define THROTTLE_MAX 1800

// -- LOOP --
#define LOOP_HZ 250 // target loop rate