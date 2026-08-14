#include "pressure_sensor.h"

#include <math.h>

#include "config.h"

namespace cfg = safestride_config;

namespace {

bool channelPresent(float value, bool active_high, float threshold) {
  return active_high ? value >= threshold : value <= threshold;
}

}  // namespace

PressureSensorPair::PressureSensorPair()
    : initialized_(false),
      last_sample_ms_(0UL),
      left_(0.0F),
      right_(0.0F),
      previous_left_(0.0F),
      previous_right_(0.0F),
      difference_(0.0F),
      maximum_delta_(0.0F),
      alert_(PressureAlert::HANDS_OFF) {}

void PressureSensorPair::begin(uint32_t now_ms) {
  left_ = static_cast<float>(analogRead(cfg::PRESSURE_LEFT_PIN));
  right_ = static_cast<float>(analogRead(cfg::PRESSURE_RIGHT_PIN));
  previous_left_ = left_;
  previous_right_ = right_;
  difference_ = fabsf(left_ - right_);
  maximum_delta_ = 0.0F;
  initialized_ = true;
  alert_ = bothHandsPresent()
      ? PressureAlert::NORMAL
      : PressureAlert::HANDS_OFF;
  last_sample_ms_ = now_ms;
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
  const float raw_left =
      static_cast<float>(analogRead(cfg::PRESSURE_LEFT_PIN));
  const float raw_right =
      static_cast<float>(analogRead(cfg::PRESSURE_RIGHT_PIN));
  const float alpha = cfg::PRESSURE_FILTER_ALPHA;

  left_ = alpha * raw_left + (1.0F - alpha) * left_;
  right_ = alpha * raw_right + (1.0F - alpha) * right_;

  const float left_delta = fabsf(left_ - previous_left_);
  const float right_delta = fabsf(right_ - previous_right_);
  maximum_delta_ = left_delta > right_delta ? left_delta : right_delta;
  difference_ = fabsf(left_ - right_);

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

bool PressureSensorPair::bothHandsPresent() const {
  return leftPresent() && rightPresent();
}

bool PressureSensorPair::leftPresent() const {
  return initialized_ && channelPresent(
      left_,
      cfg::PRESSURE_LEFT_ACTIVE_HIGH,
      cfg::PRESSURE_LEFT_PRESENT_THRESHOLD);
}

bool PressureSensorPair::rightPresent() const {
  return initialized_ && channelPresent(
      right_,
      cfg::PRESSURE_RIGHT_ACTIVE_HIGH,
      cfg::PRESSURE_RIGHT_PRESENT_THRESHOLD);
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
