#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "imu/imu.h"

#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

static uint8_t mpu_address = 0x68;

IMU imu;

bool IMU::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // 100kHz untuk init
    delay(100);

    // Auto-detect alamat MPU6050 (0x68 jika AD0 LOW, 0x69 jika AD0 HIGH)
    uint8_t addrs[] = {0x68, 0x69};
    bool found = false;

    for (uint8_t a : addrs)
    {
        Wire.beginTransmission(a);
        Wire.write(0x75); // WHO_AM_I
        if (Wire.endTransmission(false) == 0)
        {
            if (Wire.requestFrom((uint8_t)a, (uint8_t)1) == 1)
            {
                uint8_t id = Wire.read();
                Serial.printf("[IMU] Ditemukan pada alamat 0x%02X (WHO_AM_I = 0x%02X)\n", a, id);
                mpu_address = a;
                found = true;
                break;
            }
        }
    }

    if (!found)
    {
        Serial.println("[IMU ERROR] MPU6050 tidak merespon pada 0x68 maupun 0x69!");
        return false;
    }

    // Reset PWR_MGMT_1 (Wakeup sensor, internal 8MHz osc)
    Wire.beginTransmission(mpu_address);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);

    // Config Gyro (±250deg/s)
    Wire.beginTransmission(mpu_address);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Config Accel (±2g)
    Wire.beginTransmission(mpu_address);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    // Config DLPF ~44Hz
    Wire.beginTransmission(mpu_address);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.setClock(400000); // Fast mode I2C
    return true;
}

void IMU::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &gx, int16_t &gy, int16_t &gz)
{
    // Baca 14 byte sekaligus (Accel X,Y,Z + Temp + Gyro X,Y,Z) dalam 1 transaksi I2C
    Wire.beginTransmission(mpu_address);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0)
    {
        return; // Mencegah freeze jika I2C terputus
    }

    if (Wire.requestFrom((uint8_t)mpu_address, (uint8_t)14) == 14)
    {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temp (2 byte)
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
    }
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