#pragma once

#include <Arduino.h>

enum class PressureAlert : uint8_t {
  NORMAL = 0U,
  WARNING = 1U,
  HANDS_OFF = 2U,
};

class PressureSensorPair {
 public:
  PressureSensorPair();

  void begin(uint32_t now_ms);
  void update(uint32_t now_ms);

  bool bothHandsPresent() const;
  bool initialized() const;
  PressureAlert alert() const;
  float leftFiltered() const;
  float rightFiltered() const;
  float difference() const;
  float maximumDelta() const;

 private:
  bool initialized_;
  uint32_t last_sample_ms_;
  float left_;
  float right_;
  float previous_left_;
  float previous_right_;
  float difference_;
  float maximum_delta_;
  PressureAlert alert_;

  void sample();
  void writeLeds();
};
