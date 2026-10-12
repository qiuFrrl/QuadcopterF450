#include "pid/pid.h"
#include <Arduino.h>

CascadeController controller;

float PID::compute(float error, float dt)
{
    float pterm = kp * error;

    float dterm = 0.0f;
    if (dt > 0.00001f)
    {
        dterm = kd * ((error - prevError) / dt);
    }

    // Clamping Anti-Windup: Hanya integrasikan jika output belum jenuh
    // atau jika arah perubahan error mengembalikan output ke daerah linier.
    float prospectiveOut = pterm + iterm + dterm;
    bool saturatedHigh = (prospectiveOut >= outLimit && error > 0);
    bool saturatedLow  = (prospectiveOut <= -outLimit && error < 0);

    if (!saturatedHigh && !saturatedLow)
    {
        iterm += ki * (error + prevError) * (dt * 0.5f);
        iterm = constrain(iterm, -iLimit, iLimit);
    }

    float output = pterm + iterm + dterm;
    output = constrain(output, -outLimit, outLimit);

    prevError = error;
    return output;
}

void PID::reset()
{
    prevError = 0.0f;
    iterm = 0.0f;
}

CascadeController::CascadeController()
    : angleRoll(P_ANGLE_ROLL, I_ANGLE_ROLL, D_ANGLE_ROLL, PID_LIMIT),
      anglePitch(P_ANGLE_PITCH, I_ANGLE_PITCH, D_ANGLE_PITCH, PID_LIMIT),
      rateRoll(P_RATE_ROLL, I_RATE_ROLL, D_RATE_ROLL, PID_LIMIT),
      ratePitch(P_RATE_PITCH, I_RATE_PITCH, D_RATE_PITCH, PID_LIMIT),
      rateYaw(P_RATE_YAW, I_RATE_YAW, D_RATE_YAW, PID_LIMIT)
{
}

void CascadeController::update(float targetAngleRoll, float targetAnglePitch, float targetRateYaw,
                                float measuredAngleRoll, float measuredAnglePitch,
                                float measuredRateRoll, float measuredRatePitch, float measuredRateYaw,
                                float dt)
{
    // 1. Outer Loop (Angle Error -> Desired Angular Rate)
    float errAngleRoll  = targetAngleRoll  - measuredAngleRoll;
    float errAnglePitch = targetAnglePitch - measuredAnglePitch;

    float desiredRateRoll  = angleRoll.compute(errAngleRoll, dt);
    float desiredRatePitch = anglePitch.compute(errAnglePitch, dt);

    // 2. Inner Loop (Angular Rate Error -> Motor Correction)
    float errRateRoll  = desiredRateRoll  - measuredRateRoll;
    float errRatePitch = desiredRatePitch - measuredRatePitch;
    float errRateYaw   = targetRateYaw    - measuredRateYaw;

    outRoll  = rateRoll.compute(errRateRoll, dt);
    outPitch = ratePitch.compute(errRatePitch, dt);
    outYaw   = rateYaw.compute(errRateYaw, dt);
}

void CascadeController::reset()
{
    angleRoll.reset();
    anglePitch.reset();
    rateRoll.reset();
    ratePitch.reset();
    rateYaw.reset();
    outRoll = 0.0f;
    outPitch = 0.0f;
    outYaw = 0.0f;
}