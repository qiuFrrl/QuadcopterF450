#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "imu/imu.h"

#define PWR_MGMT_1   0x6B
#define CONFIG_REG   0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

IMU imu;

bool IMU::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    delay(100);

    // 1. Cek alamat 0x68 / 0x69
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0)
    {
        _addr = 0x68;
        Serial.println("[IMU] MPU6050 ditemukan di 0x68");
    }
    else
    {
        Wire.beginTransmission(0x69);
        if (Wire.endTransmission() == 0)
        {
            _addr = 0x69;
            Serial.println("[IMU] MPU6050 ditemukan di 0x69");
        }
        else
        {
            Serial.println("[IMU ERROR] MPU6050 TIDAK ditemukan!");
            _healthy = false;
            return false;
        }
    }

    // 2. Wake up MPU6050
    Wire.beginTransmission(_addr);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("[IMU ERROR] Gagal wake up MPU6050!");
        _healthy = false;
        return false;
    }
    delay(100);

    // 3. Konfigurasi DLPF (~44Hz / 0x03)
    Wire.beginTransmission(_addr);
    Wire.write(CONFIG_REG);
    Wire.write(0x03);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("[IMU ERROR] Gagal konfigurasi DLPF!");
        _healthy = false;
        return false;
    }

    // 4. Konfigurasi Gyro Full Scale: ±500°/s (0x08) -> 65.5 LSB/(°/s)
    Wire.beginTransmission(_addr);
    Wire.write(GYRO_CONFIG);
    Wire.write(GYRO_SCALE_REG);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("[IMU ERROR] Gagal konfigurasi Gyro Full Scale!");
        _healthy = false;
        return false;
    }

    // 5. Konfigurasi Accel Full Scale: ±8g (0x10) -> 4096 LSB/g
    Wire.beginTransmission(_addr);
    Wire.write(ACCEL_CONFIG);
    Wire.write(ACCEL_SCALE_REG);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("[IMU ERROR] Gagal konfigurasi Accel Full Scale!");
        _healthy = false;
        return false;
    }

    _healthy = true;
    _failCount = 0;
    Serial.println("[IMU] MPU6050 terkonfigurasi (Gyro ±500°/s, Accel ±8g)");
    return true;
}

bool IMU::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &gx, int16_t &gy, int16_t &gz)
{
    ax = ay = az = gx = gy = gz = 0;

    Wire.beginTransmission(_addr);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    if (Wire.requestFrom((uint8_t)_addr, (uint8_t)14) == 14)
    {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature (2 byte)
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
        return true;
    }

    return false;
}

void IMU::calibrate()
{
    Serial.println("[IMU] Mengukur offset fisik gyro & accelerometer (2000 sampel)...");
    Serial.println("[IMU] Pastikan drone berada pada permukaan datar dan diam!");

    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int samples = 2000;
    int validSamples = 0;

    for (int i = 0; i < samples; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        if (readRaw(ax, ay, az, gx, gy, gz))
        {
            ax_sum += (float)ax / ACCEL_SENSITIVITY;
            ay_sum += (float)ay / ACCEL_SENSITIVITY;
            az_sum += (float)az / ACCEL_SENSITIVITY;

            gx_sum += (float)gx / GYRO_SENSITIVITY;
            gy_sum += (float)gy / GYRO_SENSITIVITY;
            gz_sum += (float)gz / GYRO_SENSITIVITY;
            validSamples++;
        }

        delayMicroseconds(1000);
    }

    if (validSamples < 1000)
    {
        Serial.println("[IMU ERROR] Kalibrasi gagal: terlalu banyak sampel I2C korup!");
        return;
    }

    float calRoll  = gx_sum / validSamples;
    float calPitch = gy_sum / validSamples;
    float calYaw   = gz_sum / validSamples;
    float calAx    = ax_sum / validSamples;
    float calAy    = ay_sum / validSamples;
    float calAz    = (az_sum / validSamples) - 1.0f; // 1.0g gravitasi normal bumi

    Serial.println("[IMU] Kalibrasi selesai! Format untuk config.h:");
    Serial.printf("#define RateCalibrationRoll  %.2ff\n", calRoll);
    Serial.printf("#define RateCalibrationPitch %.2ff\n", calPitch);
    Serial.printf("#define RateCalibrationYaw    %.2ff\n", calYaw);
    Serial.printf("#define AccXCalibration       %.2ff\n", calAx);
    Serial.printf("#define AccYCalibration       %.2ff\n", calAy);
    Serial.printf("#define AccZCalibration       %.2ff\n", calAz);
}

void IMU::update(float dt)
{
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    if (!readRaw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw))
    {
        _failCount++;
        if (_failCount >= MAX_IMU_FAIL_COUNT)
        {
            _healthy = false;
        }
        return; // Jangan memproses nilai sampah jika pembacaan I2C gagal
    }

    // Pembacaan berhasil
    _failCount = 0;
    _healthy = true;

    // 1. Konversi ke satuan fisik (deg/s & g)
    float rawRateRoll  = (float)gx_raw / GYRO_SENSITIVITY;
    float rawRatePitch = (float)gy_raw / GYRO_SENSITIVITY;
    float rawRateYaw   = (float)gz_raw / GYRO_SENSITIVITY;

    float rawAccX = (float)ax_raw / ACCEL_SENSITIVITY;
    float rawAccY = (float)ay_raw / ACCEL_SENSITIVITY;
    float rawAccZ = (float)az_raw / ACCEL_SENSITIVITY;

    // 2. Koreksi kalibrasi pada satuan fisik (sesuai FlightController reference)
    rollRate  = rawRateRoll  - RateCalibrationRoll;
    pitchRate = rawRatePitch - RateCalibrationPitch;
    yawRate   = rawRateYaw   - RateCalibrationYaw;

    accX = rawAccX - AccXCalibration;
    accY = rawAccY - AccYCalibration;
    accZ = rawAccZ - AccZCalibration;

    // Proteksi data invalid (NaN / Inf)
    if (isnan(rollRate) || isnan(pitchRate) || isnan(yawRate) ||
        isnan(accX)     || isnan(accY)      || isnan(accZ))
    {
        _healthy = false;
        return;
    }

    // 3. Estimasi sudut dari accelerometer (deg)
    float normRoll  = sqrtf(accX * accX + accZ * accZ);
    float normPitch = sqrtf(accY * accY + accZ * accZ);
    float accelAngleRoll  = (normRoll > 0.001f)  ? (atanf(accY / normRoll) * 57.29578f) : 0.0f;
    float accelAnglePitch = (normPitch > 0.001f) ? (-atanf(accX / normPitch) * 57.29578f) : 0.0f;

    // 4. Complementary Filter (α = 0.991, 1 - α = 0.009)
    roll  = CF_ALPHA * (roll + rollRate * dt)   + (1.0f - CF_ALPHA) * accelAngleRoll;
    pitch = CF_ALPHA * (pitch + pitchRate * dt) + (1.0f - CF_ALPHA) * accelAnglePitch;
    yaw  += yawRate * dt;

    // 5. Clamping batas sudut matematis agar tidak terjadi singularitas,
    //    tetapi TETAP mempertahankan sudut riil di atas 45° agar auto-cut proteksi kemiringan dapat memicu!
    roll  = constrain(roll, -ATTITUDE_ESTIMATE_LIMIT, ATTITUDE_ESTIMATE_LIMIT);
    pitch = constrain(pitch, -ATTITUDE_ESTIMATE_LIMIT, ATTITUDE_ESTIMATE_LIMIT);
}