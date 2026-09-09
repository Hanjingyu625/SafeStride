// 양손 압력 센서 처리: ADC 평균 → 저역통과 필터 → 손 접촉 판정 → 해제 디바운스.
// 압력 크기로 목표속도를 정하지 않는다. 양손 접촉 여부가 주행 허용(dead-man) 조건이다.

#include "pressure_sensor.h"

#include <math.h>

#include "config.h"

namespace cfg = safestride_config;

namespace {

// 접촉 진입은 raw와 필터값이 함께 임계값을 넘어야 한다. 잔류 필터값만으로 재접촉하지 않게 한다.
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
    // A released channel can retain a high filtered value for several
    // samples. Require a live raw crossing as well so that filter decay
    // cannot create a false re-grasp after the release debounce expires.
    return raw_value >= threshold && filtered_value >= threshold;
  }
  if (was_present) {
    return raw_value <= threshold + hysteresis;
  }
  return raw_value <= threshold && filtered_value <= threshold;
}

// 평소에는 EMA로 흔들림을 줄이되, 손 해제 값은 즉시 필터 상태에 반영한다.
float filterChannel(
    float raw_value,
    float previous_filtered_value,
    bool was_present,
    bool active_high,
    float threshold) {
  const float alpha = cfg::PRESSURE_FILTER_ALPHA;
  const float filtered_value =
      alpha * raw_value + (1.0F - alpha) * previous_filtered_value;
  if (!was_present) {
    return filtered_value;
  }

  const float hysteresis = cfg::PRESSURE_PRESENT_HYSTERESIS;
  const bool released = active_high
      ? raw_value < threshold - hysteresis
      : raw_value > threshold + hysteresis;
  // Dropping the dead-man is safety-critical. Reset stale EMA state on a
  // confirmed release so reacquisition starts from the released reading.
  return released ? raw_value : filtered_value;
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
      left_release_samples_(0U),
      right_release_samples_(0U),
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

// ADC 채널 전환 직후 첫 변환은 버리고 여러 번 평균하여 다른 채널의 잔류 영향을 줄인다.
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

// 손 접촉 판정과 불균형/급변 경고를 따로 계산한다. WARNING 자체가 손 해제를 뜻하지는 않는다.
void PressureSensorPair::sample() {
  left_raw_ = readAveraged(cfg::PRESSURE_LEFT_PIN);
  right_raw_ = readAveraged(cfg::PRESSURE_RIGHT_PIN);
  const float raw_left = static_cast<float>(left_raw_);
  const float raw_right = static_cast<float>(right_raw_);

  left_ = filterChannel(
      raw_left,
      left_,
      left_present_,
      cfg::PRESSURE_LEFT_ACTIVE_HIGH,
      cfg::PRESSURE_LEFT_PRESENT_THRESHOLD);
  right_ = filterChannel(
      raw_right,
      right_,
      right_present_,
      cfg::PRESSURE_RIGHT_ACTIVE_HIGH,
      cfg::PRESSURE_RIGHT_PRESENT_THRESHOLD);

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
  const bool left_sample_present = channelPresent(
      left_,
      static_cast<float>(left_raw_),
      left_present_,
      cfg::PRESSURE_LEFT_ACTIVE_HIGH,
      cfg::PRESSURE_LEFT_PRESENT_THRESHOLD);
  const bool right_sample_present = channelPresent(
      right_,
      static_cast<float>(right_raw_),
      right_present_,
      cfg::PRESSURE_RIGHT_ACTIVE_HIGH,
      cfg::PRESSURE_RIGHT_PRESENT_THRESHOLD);
  updateChannelPresence(
      left_sample_present, left_present_, left_release_samples_);
  updateChannelPresence(
      right_sample_present, right_present_, right_release_samples_);
}

// 해제 샘플이 연속 2번이어야 접촉을 해제한다. 샘플 주기는 100ms이므로 한 번의 순간 하락을 무시한다.
void PressureSensorPair::updateChannelPresence(
    bool sample_present,
    bool& present,
    uint8_t& release_samples) {
  if (sample_present) {
    present = true;
    release_samples = 0U;
    return;
  }
  if (!present) {
    release_samples = 0U;
    return;
  }
  if (release_samples < cfg::PRESSURE_RELEASE_DEBOUNCE_SAMPLES) {
    ++release_samples;
  }
  if (release_samples >= cfg::PRESSURE_RELEASE_DEBOUNCE_SAMPLES) {
    present = false;
    release_samples = 0U;
  }
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
