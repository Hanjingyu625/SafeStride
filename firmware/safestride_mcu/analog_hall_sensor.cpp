// A3 아날로그 Hall 신호를 자석 통과 이벤트로 변환한다.
// 기준 ADC와의 차이로 진입/이탈을 판단하며, 속도 계산은 motor_control.cpp에서 수행한다.

#include "analog_hall_sensor.h"

#include "config.h"

namespace cfg = safestride_config;

namespace {

uint16_t magnitude(uint16_t value, uint16_t reference) {
  return value >= reference ? value - reference : reference - value;
}

}  // namespace

AnalogHallSensor::AnalogHallSensor()
    : pulse_count_(0UL),
      last_pulse_us_(0UL),
      period_us_(0UL),
      last_sample_us_(0UL),
      baseline_q8_(0L),
      raw_adc_(0U),
      magnet_present_(false) {}

// 부팅 시 ADC 평균을 기준값으로 잡는다. 이때 자석 위치가 기준값에 영향을 줄 수 있다.
void AnalogHallSensor::begin(uint32_t now_us) {
  pinMode(cfg::HALL_ANALOG_PIN, INPUT);

  uint32_t total = 0UL;
  for (uint8_t sample = 0U; sample < cfg::HALL_BASELINE_SAMPLES; ++sample) {
    total += static_cast<uint16_t>(analogRead(cfg::HALL_ANALOG_PIN));
    delayMicroseconds(cfg::HALL_BASELINE_SAMPLE_DELAY_US);
  }

  raw_adc_ = static_cast<uint16_t>(total / cfg::HALL_BASELINE_SAMPLES);
  baseline_q8_ = static_cast<int32_t>(raw_adc_) << 8U;
  pulse_count_ = 0UL;
  last_pulse_us_ = 0UL;
  period_us_ = 0UL;
  last_sample_us_ = now_us;
  magnet_present_ = false;
}

void AnalogHallSensor::update(uint32_t now_us) {
  if (now_us - last_sample_us_ < cfg::HALL_SAMPLE_PERIOD_US) {
    return;
  }
  last_sample_us_ = now_us;
  raw_adc_ = readAveraged();

  const uint16_t baseline_adc = baselineAdc();
  const uint16_t delta_adc = magnitude(raw_adc_, baseline_adc);
  // 자석이 감지된 동안 재계수하지 않는다. 해제 임계값 안으로 돌아와야 다음 펄스를 셀 수 있다.
  if (magnet_present_) {
    if (delta_adc <= cfg::HALL_RELEASE_DELTA_ADC) {
      magnet_present_ = false;
    }
    return;
  }

  // 기준에서 충분히 벗어나면 한 펄스. 최소 간격은 짧은 잡음 펄스의 중복 계수를 줄인다.
  if (delta_adc >= cfg::HALL_TRIGGER_DELTA_ADC) {
    const uint32_t elapsed_us = now_us - last_pulse_us_;
    if (last_pulse_us_ == 0UL ||
        elapsed_us >= cfg::HALL_MIN_PULSE_INTERVAL_US) {
      if (last_pulse_us_ != 0UL) {
        period_us_ = elapsed_us;
      }
      last_pulse_us_ = now_us;
      ++pulse_count_;
    }
    magnet_present_ = true;
    return;
  }

  // 자석이 없는 것으로 보이는 구간에서만 기준을 천천히 추적한다. Q8은 정수에 소수 8비트를 둔 표현이다.
  if (delta_adc <= cfg::HALL_RELEASE_DELTA_ADC) {
    const int32_t sample_q8 = static_cast<int32_t>(raw_adc_) << 8U;
    baseline_q8_ +=
        (sample_q8 - baseline_q8_) / cfg::HALL_BASELINE_TRACK_DIVISOR;
  }
}

uint32_t AnalogHallSensor::pulseCount() const {
  return pulse_count_;
}

uint32_t AnalogHallSensor::periodUs() const {
  return period_us_;
}

uint32_t AnalogHallSensor::ageUs(uint32_t now_us) const {
  return last_pulse_us_ == 0UL ? 0xFFFFFFFFUL : now_us - last_pulse_us_;
}

uint16_t AnalogHallSensor::rawAdc() const {
  return raw_adc_;
}

uint16_t AnalogHallSensor::baselineAdc() const {
  return static_cast<uint16_t>((baseline_q8_ + 128L) >> 8U);
}

bool AnalogHallSensor::magnetPresent() const {
  return magnet_present_;
}

uint16_t AnalogHallSensor::readAveraged() const {
  uint32_t total = 0UL;
  for (uint8_t sample = 0U; sample < cfg::HALL_ADC_SAMPLES; ++sample) {
    total += static_cast<uint16_t>(analogRead(cfg::HALL_ANALOG_PIN));
  }
  return static_cast<uint16_t>(total / cfg::HALL_ADC_SAMPLES);
}
