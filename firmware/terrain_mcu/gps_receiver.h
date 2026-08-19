#pragma once

#include <Arduino.h>

struct GpsSample {
  int32_t latitude_e7;
  int32_t longitude_e7;
  uint32_t speed_mm_s;
  uint8_t flags;
  uint8_t satellites;
};

class GpsReceiver {
 public:
  static const uint8_t FLAG_FIX_VALID = 1U << 0U;
  static const uint8_t FLAG_SPEED_VALID = 1U << 1U;

  void begin();
  void poll();
  GpsSample sample(uint32_t now_ms) const;
};
