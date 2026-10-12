#pragma once
#include <Arduino.h>
#include "shared/shared_types.h"

// Communication Task (Core 0)
// Tanggung jawab: ESP-NOW packet processing, serial command parser, update shared state
void commTask(void *pvParameters);
