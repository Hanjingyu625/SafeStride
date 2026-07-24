#pragma once

#include <Arduino.h>

// Kept in a header so Arduino's generated sketch prototypes can see the type.
enum class ControllerState : uint8_t {
  BOOT = 0U,
  DISARMED = 1U,
  ARMED = 2U,
  SAFE_STOP = 3U,
  ESTOP = 4U,
  FAULT = 5U,
};
