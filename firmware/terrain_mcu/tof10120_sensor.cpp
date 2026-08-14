#include "tof10120_sensor.h"

#include <Wire.h>

#include "config.h"

namespace {

namespace cfg = safestride_terrain_config;

}  // namespace

Tof10120Sensor::Tof10120Sensor()
    : initialized_(false),
      valid_(false),
      red_hold_active_(false),
      consecutive_count_(0U),
      last_sample_ms_(0UL),
      last_red_ms_(0UL),
      distance_mm_(0xFFFFU),
      filtered_mm_(0.0F),
      reference_mm_(0.0F),
      error_mm_(0.0F),
      change_mm_(0.0F),
      alert_(TofAlert::INVALID) {}

void Tof10120Sensor::begin(uint32_t now_ms) {
  last_sample_ms_ = now_ms - cfg::TOF_SAMPLE_PERIOD_MS;
}

void Tof10120Sensor::update(uint32_t now_ms) {
  if (now_ms - last_sample_ms_ < cfg::TOF_SAMPLE_PERIOD_MS) {
    return;
  }
  last_sample_ms_ = now_ms;

  const uint16_t distance = readDistanceI2c();
  if (distance < cfg::TOF_MIN_VALID_DISTANCE_MM ||
      distance > cfg::TOF_MAX_VALID_DISTANCE_MM) {
    valid_ = false;
    consecutive_count_ = 0U;
    alert_ = TofAlert::INVALID;
    return;
  }

  classify(now_ms, distance);
}

uint16_t Tof10120Sensor::readDistanceI2c() {
  Wire.beginTransmission(cfg::TOF_I2C_ADDRESS);
  Wire.write(cfg::TOF_DISTANCE_REGISTER);
  if (Wire.endTransmission() != 0U) {
    return 0xFFFFU;
  }

  delayMicroseconds(50U);

  if (Wire.requestFrom(cfg::TOF_I2C_ADDRESS, static_cast<uint8_t>(2U)) !=
      2U) {
    return 0xFFFFU;
  }
  if (Wire.available() < 2) {
    return 0xFFFFU;
  }

  const uint8_t high_byte = static_cast<uint8_t>(Wire.read());
  const uint8_t low_byte = static_cast<uint8_t>(Wire.read());
  return static_cast<uint16_t>(
      (static_cast<uint16_t>(high_byte) << 8U) | low_byte);
}

void Tof10120Sensor::classify(
    uint32_t now_ms,
    uint16_t distance_mm) {
  distance_mm_ = distance_mm;
  valid_ = true;

  if (!initialized_) {
    filtered_mm_ = static_cast<float>(distance_mm);
    reference_mm_ = filtered_mm_;
    error_mm_ = 0.0F;
    change_mm_ = 0.0F;
    initialized_ = true;
    alert_ = TofAlert::NORMAL;
    return;
  }

  const float previous_filtered = filtered_mm_;
  filtered_mm_ = cfg::TOF_FILTER_ALPHA * static_cast<float>(distance_mm) +
                 (1.0F - cfg::TOF_FILTER_ALPHA) * filtered_mm_;
  reference_mm_ = cfg::TOF_REFERENCE_ALPHA * filtered_mm_ +
                  (1.0F - cfg::TOF_REFERENCE_ALPHA) * reference_mm_;
  error_mm_ = filtered_mm_ - reference_mm_;
  change_mm_ = filtered_mm_ - previous_filtered;

  if (error_mm_ > cfg::TOF_ERROR_THRESHOLD_MM &&
      change_mm_ > cfg::TOF_CHANGE_THRESHOLD_MM) {
    if (consecutive_count_ < cfg::TOF_REQUIRED_FRAMES) {
      ++consecutive_count_;
    }
  } else {
    consecutive_count_ = 0U;
  }

  if (consecutive_count_ >= cfg::TOF_REQUIRED_FRAMES) {
    red_hold_active_ = true;
    last_red_ms_ = now_ms;
  }
  if (red_hold_active_ &&
      now_ms - last_red_ms_ >= cfg::TOF_RED_HOLD_MS) {
    red_hold_active_ = false;
  }

  if (red_hold_active_) {
    alert_ = TofAlert::STEP;
  } else if (error_mm_ > cfg::TOF_ERROR_THRESHOLD_MM) {
    alert_ = TofAlert::CANDIDATE;
  } else {
    alert_ = TofAlert::NORMAL;
  }
}

bool Tof10120Sensor::valid() const {
  return valid_;
}

uint16_t Tof10120Sensor::distanceMm() const {
  return valid_ ? distance_mm_ : 0xFFFFU;
}

float Tof10120Sensor::filteredDistanceMm() const {
  return filtered_mm_;
}

float Tof10120Sensor::referenceDistanceMm() const {
  return reference_mm_;
}

float Tof10120Sensor::errorMm() const {
  return error_mm_;
}

float Tof10120Sensor::changeMm() const {
  return change_mm_;
}

TofAlert Tof10120Sensor::alert() const {
  return alert_;
}
