#include "motor/motor.h"

Motor motor;

void Motor::begin()
{
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    _fl.attach(PIN_FL, PWM_MIN, PWM_MAX);
    _fl.setPeriodHertz(500);

    _fr.attach(PIN_FR, PWM_MIN, PWM_MAX);
    _fr.setPeriodHertz(500);

    _rl.attach(PIN_RL, PWM_MIN, PWM_MAX);
    _rl.setPeriodHertz(500);

    _rr.attach(PIN_RR, PWM_MIN, PWM_MAX);
    _rr.setPeriodHertz(500);

    cut();
}

void Motor::arm()
{
    write(PWM_ARM, PWM_ARM, PWM_ARM, PWM_ARM);
}

void Motor::cut()
{
    write(PWM_ARM, PWM_ARM, PWM_ARM, PWM_ARM);
}

void Motor::write(int fl, int fr, int rl, int rr)
{
    _lastFL = _clamp(fl);
    _lastFR = _clamp(fr);
    _lastRL = _clamp(rl);
    _lastRR = _clamp(rr);

    _fl.writeMicroseconds(_lastFL);
    _fr.writeMicroseconds(_lastFR);
    _rl.writeMicroseconds(_lastRL);
    _rr.writeMicroseconds(_lastRR);
}

void Motor::mix(int throttle, float rollOut, float pitchOut, float yawOut)
{
    // Safety cut-off jika throttle di bawah ambang aman
    if (throttle < THROTTLE_CUTOFF)
    {
        cut();
        return;
    }

    throttle = min(throttle, THROTTLE_MAX);

    // Quad X motor mixing layout (sesuai referensi FlightController.cpp):
    // mot1 (FR, CCW) = throttle - roll - pitch - yaw
    // mot2 (RR, CW)  = throttle - roll + pitch + yaw
    // mot3 (RL, CCW) = throttle + roll + pitch - yaw
    // mot4 (FL, CW)  = throttle + roll - pitch + yaw
    int fr = throttle - (int)rollOut - (int)pitchOut - (int)yawOut;
    int rr = throttle - (int)rollOut + (int)pitchOut + (int)yawOut;
    int rl = throttle + (int)rollOut + (int)pitchOut - (int)yawOut;
    int fl = throttle + (int)rollOut - (int)pitchOut + (int)yawOut;

    // Batas minimum PWM_IDLE saat terbang agar motor tidak stall
    fl = constrain(fl, PWM_IDLE, PWM_MAX);
    fr = constrain(fr, PWM_IDLE, PWM_MAX);
    rl = constrain(rl, PWM_IDLE, PWM_MAX);
    rr = constrain(rr, PWM_IDLE, PWM_MAX);

    write(fl, fr, rl, rr);
}