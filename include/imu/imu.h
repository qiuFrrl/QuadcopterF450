#pragma once
#include <Wire.h>
#include "config.h"

class IMU
{
public:
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    float rollRate = 0;
    float pitchRate = 0;
    float yawRate = 0;

    bool begin();
    void update(float dt);

private:
    void readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz);
};

extern IMU imu;