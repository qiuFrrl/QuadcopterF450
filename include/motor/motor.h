#pragma once
#include <ESP32Servo.h>
#include "config.h"

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