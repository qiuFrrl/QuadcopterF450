#pragma once
#include <Arduino.h>
#include "shared/shared_types.h"

// Flight Control Task (Core 1)
// Tanggung jawab: Sensor IMU 250Hz, Cascade PID, Motor Mixer, State Machine, Failsafe lokal
void flightTask(void *pvParameters);
