#pragma once

#include <Arduino.h>

class AnalogHallSensor {
 public:
  AnalogHallSensor();

  void begin(uint32_t now_us);
  void update(uint32_t now_us);

  uint32_t pulseCount() const;
  uint32_t periodUs() const;
  uint32_t ageUs(uint32_t now_us) const;
  uint16_t rawAdc() const;
  uint16_t baselineAdc() const;
  bool magnetPresent() const;

 private:
  uint16_t readAveraged() const;

  uint32_t pulse_count_;
  uint32_t last_pulse_us_;
  uint32_t period_us_;
  uint32_t last_sample_us_;
  int32_t baseline_q8_;
  uint16_t raw_adc_;
  bool magnet_present_;
};
