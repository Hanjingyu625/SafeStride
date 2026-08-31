#include "pressure_sensor.h"

#include <math.h>

#include "config.h"

namespace cfg = safestride_config;

namespace {

bool channelPresent(
    float filtered_value,
    float raw_value,
    bool was_present,
    bool active_high,
    float threshold) {
  const float hysteresis = cfg::PRESSURE_PRESENT_HYSTERESIS;
  if (active_high) {
    if (was_present) {
      return raw_value >= threshold - hysteresis;
    }
    return filtered_value >= threshold;
  }
  if (was_present) {
    return raw_value <= threshold + hysteresis;
  }
  return filtered_value <= threshold;
}

}  // namespace

PressureSensorPair::PressureSensorPair()
    : initialized_(false),
      last_sample_ms_(0UL),
      left_raw_(0U),
      right_raw_(0U),
      left_(0.0F),
      right_(0.0F),
      previous_left_(0.0F),
      previous_right_(0.0F),
      difference_(0.0F),
      maximum_delta_(0.0F),
      left_present_(false),
      right_present_(false),
      alert_(PressureAlert::HANDS_OFF) {}

void PressureSensorPair::begin(uint32_t now_ms) {
  left_raw_ = readAveraged(cfg::PRESSURE_LEFT_PIN);
  right_raw_ = readAveraged(cfg::PRESSURE_RIGHT_PIN);
  left_ = static_cast<float>(left_raw_);
  right_ = static_cast<float>(right_raw_);
  previous_left_ = left_;
  previous_right_ = right_;
  difference_ = fabsf(left_ - right_);
  maximum_delta_ = 0.0F;
  initialized_ = true;
  updatePresence();
  alert_ = bothHandsPresent()
      ? PressureAlert::NORMAL
      : PressureAlert::HANDS_OFF;
  last_sample_ms_ = now_ms;
}

uint16_t PressureSensorPair::readAveraged(uint8_t pin) const {
  // Discard the first conversion after an AVR ADC mux change so charge left
  // by the other pressure channel does not leak into this reading.
  (void)analogRead(pin);
  uint32_t total = 0UL;
  for (uint8_t index = 0U; index < cfg::PRESSURE_ADC_SAMPLES; ++index) {
    int reading = analogRead(pin);
    if (reading < 0) {
      reading = 0;
    } else if (reading > 1023) {
      reading = 1023;
    }
    total += static_cast<uint16_t>(reading);
  }
  return static_cast<uint16_t>(
      (total + cfg::PRESSURE_ADC_SAMPLES / 2U) /
      cfg::PRESSURE_ADC_SAMPLES);
}

void PressureSensorPair::update(uint32_t now_ms) {
  if (!initialized_) {
    begin(now_ms);
    return;
  }
  if (now_ms - last_sample_ms_ < cfg::PRESSURE_SAMPLE_PERIOD_MS) {
    return;
  }
  last_sample_ms_ = now_ms;
  sample();
}

void PressureSensorPair::sample() {
  left_raw_ = readAveraged(cfg::PRESSURE_LEFT_PIN);
  right_raw_ = readAveraged(cfg::PRESSURE_RIGHT_PIN);
  const float raw_left = static_cast<float>(left_raw_);
  const float raw_right = static_cast<float>(right_raw_);
  const float alpha = cfg::PRESSURE_FILTER_ALPHA;

  left_ = alpha * raw_left + (1.0F - alpha) * left_;
  right_ = alpha * raw_right + (1.0F - alpha) * right_;

  const float left_delta = fabsf(left_ - previous_left_);
  const float right_delta = fabsf(right_ - previous_right_);
  maximum_delta_ = left_delta > right_delta ? left_delta : right_delta;
  difference_ = fabsf(left_ - right_);
  updatePresence();

  if (!bothHandsPresent()) {
    alert_ = PressureAlert::HANDS_OFF;
  } else if (
      difference_ > cfg::PRESSURE_IMBALANCE_THRESHOLD ||
      maximum_delta_ > cfg::PRESSURE_SUDDEN_CHANGE_THRESHOLD) {
    alert_ = PressureAlert::WARNING;
  } else {
    alert_ = PressureAlert::NORMAL;
  }

  previous_left_ = left_;
  previous_right_ = right_;
}

void PressureSensorPair::updatePresence() {
  left_present_ = channelPresent(
      left_,
      static_cast<float>(left_raw_),
      left_present_,
      cfg::PRESSURE_LEFT_ACTIVE_HIGH,
      cfg::PRESSURE_LEFT_PRESENT_THRESHOLD);
  right_present_ = channelPresent(
      right_,
      static_cast<float>(right_raw_),
      right_present_,
      cfg::PRESSURE_RIGHT_ACTIVE_HIGH,
      cfg::PRESSURE_RIGHT_PRESENT_THRESHOLD);
}

bool PressureSensorPair::bothHandsPresent() const {
  return leftPresent() && rightPresent();
}

bool PressureSensorPair::leftPresent() const {
  return initialized_ && left_present_;
}

bool PressureSensorPair::rightPresent() const {
  return initialized_ && right_present_;
}

bool PressureSensorPair::initialized() const {
  return initialized_;
}

bool PressureSensorPair::calibrated() const {
  return initialized_ && cfg::PRESSURE_THRESHOLDS_CALIBRATED;
}

PressureAlert PressureSensorPair::alert() const {
  return alert_;
}

uint16_t PressureSensorPair::leftRaw() const {
  return left_raw_;
}

uint16_t PressureSensorPair::rightRaw() const {
  return right_raw_;
}

float PressureSensorPair::leftFiltered() const {
  return left_;
}

float PressureSensorPair::rightFiltered() const {
  return right_;
}

float PressureSensorPair::difference() const {
  return difference_;
}

float PressureSensorPair::maximumDelta() const {
  return maximum_delta_;
}
