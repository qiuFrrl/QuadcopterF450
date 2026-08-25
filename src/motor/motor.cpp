#include "motor/motor.h"

Motor motor;

void Motor::begin()
{
    _fl.attach(PIN_FL, PWM_MIN, PWM_MAX);
    _fr.attach(PIN_FR, PWM_MIN, PWM_MAX);
    _rl.attach(PIN_RL, PWM_MIN, PWM_MAX);
    _rr.attach(PIN_RR, PWM_MIN, PWM_MAX);
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
    _fl.writeMicroseconds(_clamp(fl));
    _fr.writeMicroseconds(_clamp(fr));
    _rl.writeMicroseconds(_clamp(rl));
    _rr.writeMicroseconds(_clamp(rr));
}

void Motor::mix(int throttle, float rollOut, float pitchOut, float yawOut)
{
    int fl = throttle + (int)(rollOut + pitchOut - yawOut);
    int fr = throttle + (int)(-rollOut + pitchOut + yawOut);
    int rl = throttle + (int)(rollOut - pitchOut + yawOut);
    int rr = throttle + (int)(-rollOut - pitchOut - yawOut);

    // Throttle floor — jangan biarkan motor mati saat masih armed
    fl = max(fl, PWM_IDLE);
    fr = max(fr, PWM_IDLE);
    rl = max(rl, PWM_IDLE);
    rr = max(rr, PWM_IDLE);

    write(fl, fr, rl, rr);
}