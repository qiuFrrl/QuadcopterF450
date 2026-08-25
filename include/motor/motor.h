#pragma once
#include <ESP32Servo.h>
#include "config.h"

// ============================================================
//  MOTOR.H
//
//  Layout (top view):
//
//    FL (CW)  [13] ---- FR (CCW) [14]
//         |                  |
//    RL (CCW) [27] ---- RR (CW)  [26]
//
//  Motor mixing:
//    Roll+  → kiri turun  → FL/RL naik, FR/RR turun
//    Pitch+ → depan turun → FL/FR naik, RL/RR turun
//    Yaw+   → CW          → FL/RR naik, FR/RL turun
// ============================================================

class Motor
{
public:
    void begin();
    void arm();
    void cut();
    void write(int fl, int fr, int rl, int rr);
    void mix(int throttle, float rollOut, float pitchOut, float yawOut);

private:
    Servo _fl, _fr, _rl, _rr;
    int _clamp(int v) { return constrain(v, PWM_MIN, PWM_MAX); }
};

extern Motor motor;