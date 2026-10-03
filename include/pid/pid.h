#pragma once

class PID
{
public:
    float kp, ki, kd;
    float integral = 0;
    float prevErr = 0;
    float iLimit = 50.0f;

    PID(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    // rate = gyro rate langsung (lebih smooth dari derivative error)
    float compute(float setpoint, float measured, float rate,
                  float dt, float deadzone = 0.0f);

    void reset();
};