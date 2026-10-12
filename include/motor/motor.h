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

    int getFL() const { return _lastFL; }
    int getFR() const { return _lastFR; }
    int getRL() const { return _lastRL; }
    int getRR() const { return _lastRR; }

private:
    Servo _fl, _fr, _rl, _rr;
    int _lastFL = PWM_ARM;
    int _lastFR = PWM_ARM;
    int _lastRL = PWM_ARM;
    int _lastRR = PWM_ARM;

    int _clamp(int v) { return constrain(v, PWM_MIN, PWM_MAX); }
};

extern Motor motor;