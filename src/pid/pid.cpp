#include "pid.h"
#include <Arduino.h>

float PID::compute(float setpoint, float measured, float rate,
                   float dt, float deadzone)
{
    float err = setpoint - measured;

    if (fabsf(err) < deadzone)
    {
        err = 0;
        integral = 0;
    }

    integral += err * dt;
    integral = constrain(integral, -iLimit, iLimit);

    // Derivative: pakai gyro rate langsung (D-on-measurement)
    float derivative = -rate;

    prevErr = err;
    return kp * err + ki * integral + kd * derivative;
}

void PID::reset()
{
    integral = 0;
    prevErr = 0;
}