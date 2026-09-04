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
  bool leftPresent() const;
  bool rightPresent() const;
  bool initialized() const;
  bool calibrated() const;
  PressureAlert alert() const;
  uint16_t leftRaw() const;
  uint16_t rightRaw() const;
  float leftFiltered() const;
  float rightFiltered() const;
  float difference() const;
  float maximumDelta() const;

 private:
  bool initialized_;
  uint32_t last_sample_ms_;
  uint16_t left_raw_;
  uint16_t right_raw_;
  float left_;
  float right_;
  float previous_left_;
  float previous_right_;
  float difference_;
  float maximum_delta_;
  bool left_present_;
  bool right_present_;
  uint8_t left_release_samples_;
  uint8_t right_release_samples_;
  PressureAlert alert_;

  uint16_t readAveraged(uint8_t pin) const;
  void sample();
  void updatePresence();
  static void updateChannelPresence(
      bool sample_present,
      bool& present,
      uint8_t& release_samples);
};
