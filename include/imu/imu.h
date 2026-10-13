#pragma once
#include <Arduino.h>
#include "config.h"

class IMU
{
public:
    // Physical rates (deg/s)
    float rollRate = 0.0f;
    float pitchRate = 0.0f;
    float yawRate = 0.0f;

    // Physical acceleration (g)
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;

    // Filtered attitude angles (deg)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    bool begin();
    bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz);
    void update(float dt);
    void calibrate();

    bool isHealthy() const { return _healthy; }
    uint32_t getFailCount() const { return _failCount; }

private:
    uint8_t  _addr = 0x68;
    bool     _healthy = false;
    bool     _angleInitialized = false;
    uint32_t _failCount = 0;
};

extern IMU imu;
