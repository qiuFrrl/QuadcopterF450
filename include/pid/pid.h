#pragma once
#include <Arduino.h>
#include "config.h"

class PID
{
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float iLimit = PID_LIMIT;
    float outLimit = PID_LIMIT;

    float prevError = 0.0f;
    float iterm = 0.0f;

    PID() = default;
    PID(float p, float i, float d, float limit = PID_LIMIT)
        : kp(p), ki(i), kd(d), iLimit(limit), outLimit(limit) {}

    float compute(float error, float dt);
    void reset();
};

class CascadeController
{
public:
    // Outer Loop: Angle PID
    PID angleRoll;
    PID anglePitch;

    // Inner Loop: Rate PID
    PID rateRoll;
    PID ratePitch;
    PID rateYaw;

    // Output koreksi motor (µs)
    float outRoll = 0.0f;
    float outPitch = 0.0f;
    float outYaw = 0.0f;

    CascadeController();

    void update(float targetAngleRoll, float targetAnglePitch, float targetRateYaw,
                float measuredAngleRoll, float measuredAnglePitch,
                float measuredRateRoll, float measuredRatePitch, float measuredRateYaw,
                float dt);

    void reset();
};

extern CascadeController controller;