#pragma once

#include <Arduino.h>

enum class TofAlert : uint8_t {
  NORMAL = 0U,
  CANDIDATE_RAISED = 1U,
  CANDIDATE_DROP = 2U,
  RAISED = 3U,
  DROP = 4U,
  INVALID = 5U,
};

class Tof10120Sensor {
 public:
  Tof10120Sensor();

  void begin(uint32_t now_ms);
  void update(uint32_t now_ms);

  bool valid() const;
  uint16_t distanceMm() const;
  float filteredDistanceMm() const;
  float referenceDistanceMm() const;
  float errorMm() const;
  float changeMm() const;
  TofAlert alert() const;

 private:
  bool initialized_;
  bool valid_;
  bool red_hold_active_;
  uint8_t baseline_count_;
  uint8_t consecutive_count_;
  int8_t candidate_direction_;
  uint32_t last_sample_ms_;
  uint32_t last_red_ms_;
  uint16_t distance_mm_;
  float filtered_mm_;
  float reference_mm_;
  float error_mm_;
  float change_mm_;
  TofAlert alert_;
  TofAlert last_hazard_alert_;

  uint16_t readDistanceI2c();
  void classify(uint32_t now_ms, uint16_t distance_mm);
};
