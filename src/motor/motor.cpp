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
    float pitchMix = pitchOut * PITCH_MIX_SIGN;

    // Quad X motor mixing layout yang benar secara aerodinamika:
    // M1 (FR, CCW, Pin 25) = Throttle - Roll + Pitch - Yaw
    // M2 (RR, CW,  Pin 33) = Throttle - Roll - Pitch + Yaw
    // M3 (RL, CCW, Pin 12) = Throttle + Roll - Pitch - Yaw
    // M4 (FL, CW,  Pin 27) = Throttle + Roll + Pitch + Yaw
    int fr = throttle - (int)rollOut + (int)pitchMix - (int)yawOut;
    int rr = throttle - (int)rollOut - (int)pitchMix + (int)yawOut;
    int rl = throttle + (int)rollOut - (int)pitchMix - (int)yawOut;
    int fl = throttle + (int)rollOut + (int)pitchMix + (int)yawOut;

    // Batas minimum efektif:
    // Jika terbang normal (throttle >= PWM_IDLE), batas bawah adalah PWM_IDLE (1170)
    // agar motor tidak stall di udara dan mempertahankan otoritas gyroscopic.
    // Jika landing (throttle < PWM_IDLE), batas bawah diturunkan mengikuti throttle
    // (minimal THROTTLE_CUTOFF) agar thrust berkurang dan drone dapat mendarat.
    int minPwmFloor = (throttle >= PWM_IDLE) ? PWM_IDLE : max((int)THROTTLE_CUTOFF, throttle - 30);

    // Preservasi otoritas sikap kendali (AirMode Saturation Management)
    // 1. Jika ada motor yang melampaui batas atas PWM_MAX, turunkan throttle bersama
    //    agar selisih roll/pitch/yaw tidak terpotong (menghindari flip tak terkendali).
    int maxM = max(max(fl, fr), max(rl, rr));
    if (maxM > PWM_MAX)
    {
        int excess = maxM - PWM_MAX;
        fl -= excess;
        fr -= excess;
        rl -= excess;
        rr -= excess;
    }

    // 2. Jika ada motor di bawah batas bawah efektif, naikkan bersama
    //    agar motor tidak stall dan mempertahankan redaman gyroscopic.
    int minM = min(min(fl, fr), min(rl, rr));
    if (minM < minPwmFloor)
    {
        int deficit = minPwmFloor - minM;
        fl += deficit;
        fr += deficit;
        rl += deficit;
        rr += deficit;
    }

    // Batas pengaman final
    fl = constrain(fl, minPwmFloor, PWM_MAX);
    fr = constrain(fr, minPwmFloor, PWM_MAX);
    rl = constrain(rl, minPwmFloor, PWM_MAX);
    rr = constrain(rr, minPwmFloor, PWM_MAX);

    write(fl, fr, rl, rr);
}
