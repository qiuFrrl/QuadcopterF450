#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "imu/imu.h"

#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

IMU imu;

bool IMU::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0)
        return false;
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

    return true;
}

void IMU::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &gx, int16_t &gy, int16_t &gz)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

void IMU::update(float dt)
{
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    readRaw(ax_r, ay_r, az_r, gx_r, gy_r, gz_r);

    ax_r -= AX_OFFSET;
    ay_r -= AY_OFFSET;
    az_r -= AZ_OFFSET;
    gx_r -= GX_OFFSET;
    gy_r -= GY_OFFSET;
    gz_r -= GZ_OFFSET;

    float ax_g = ax_r / 16384.0f;
    float ay_g = ay_r / 16384.0f;
    float az_g = az_r / 16384.0f;
    float gx_d = gx_r / 131.0f;
    float gy_d = gy_r / 131.0f;
    float gz_d = gz_r / 131.0f;

    rollRate = gx_d;
    pitchRate = gy_d;
    yawRate = gz_d;

    float aRoll = atan2f(ay_g, az_g) * 180.0f / M_PI;
    float aPitch = -asinf(ax_g < -1.0f ? -1.0f : ax_g > 1.0f ? 1.0f
                                                             : ax_g) *
                   180.0f / M_PI;

    roll = CF_ALPHA * (roll + gx_d * dt) + (1.0f - CF_ALPHA) * aRoll;
    pitch = CF_ALPHA * (pitch + gy_d * dt) + (1.0f - CF_ALPHA) * aPitch;
    yaw += gz_d * dt;

    roll -= ROLL_TRIM;
    pitch -= PITCH_TRIM;
}