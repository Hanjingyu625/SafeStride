#pragma once

#include <Arduino.h>

// Single active-low, open-collector Hall input. Direction is not measured.
class DigitalHallSensor {
 public:
  void begin(uint32_t now_us);
  void update(uint32_t now_us);
  uint32_t pulseCount() const { return pulse_count_; }
  uint32_t periodUs() const { return period_us_; }
  uint32_t ageUs(uint32_t now_us) const;
  bool magnetPresent() const { return magnet_present_; }

 private:
  static DigitalHallSensor* instance_;
  static void onChange();
  volatile uint32_t isr_count_ = 0UL;
  volatile uint32_t isr_last_us_ = 0UL;
  volatile uint32_t isr_period_us_ = 0UL;
  volatile bool isr_have_pulse_ = false;
  volatile bool isr_present_ = false;
  uint32_t pulse_count_ = 0UL;
  uint32_t period_us_ = 0UL;
  uint32_t age_us_ = 0xFFFFFFFFUL;
  uint32_t snapshot_us_ = 0UL;
  bool magnet_present_ = false;
};
