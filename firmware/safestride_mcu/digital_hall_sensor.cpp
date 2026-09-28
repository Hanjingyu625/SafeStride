#include "digital_hall_sensor.h"
#include "config.h"

#if defined(__AVR__)
#include <util/atomic.h>
#endif

namespace cfg = safestride_config;
DigitalHallSensor* DigitalHallSensor::instance_ = nullptr;

void DigitalHallSensor::begin(uint32_t now_us) {
  pinMode(cfg::HALL_DIGITAL_PIN, INPUT_PULLUP);
  noInterrupts();
  instance_ = this;
  isr_count_ = 0UL;
  isr_last_us_ = 0UL;
  isr_period_us_ = 0UL;
  isr_have_pulse_ = false;
  isr_present_ = digitalRead(cfg::HALL_DIGITAL_PIN) == LOW;
  pulse_count_ = 0UL;
  period_us_ = 0UL;
  age_us_ = 0xFFFFFFFFUL;
  snapshot_us_ = now_us;
  magnet_present_ = isr_present_;
  attachInterrupt(digitalPinToInterrupt(cfg::HALL_DIGITAL_PIN), onChange, CHANGE);
  interrupts();
}

void DigitalHallSensor::onChange() {
  DigitalHallSensor* self = instance_;
  if (self == nullptr) return;
  const bool present = digitalRead(cfg::HALL_DIGITAL_PIN) == LOW;
  const bool entering = present && !self->isr_present_;
  self->isr_present_ = present;
  if (!entering) return;
  const uint32_t now_us = micros();
  const uint32_t elapsed = now_us - self->isr_last_us_;
  if (self->isr_have_pulse_ && elapsed < cfg::HALL_MIN_PULSE_INTERVAL_US) return;
  self->isr_period_us_ = self->isr_have_pulse_ ? elapsed : 0UL;
  self->isr_last_us_ = now_us;
  self->isr_have_pulse_ = true;
  ++self->isr_count_;
}

void DigitalHallSensor::update(uint32_t) {
  // AVR cannot copy 32-bit ISR fields atomically. Keep one coherent snapshot.
#if defined(__AVR__)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#else
  noInterrupts();
  {
#endif
    const uint32_t now_us = micros();
    if (isr_have_pulse_ && (age_us_ == 0xFFFFFFFFUL || pulse_count_ != isr_count_)) {
      age_us_ = now_us - isr_last_us_;
    } else if (isr_have_pulse_) {
      const uint32_t elapsed = now_us - snapshot_us_;
      age_us_ = elapsed >= 0xFFFFFFFEUL - age_us_
          ? 0xFFFFFFFEUL : age_us_ + elapsed;
    }
    snapshot_us_ = now_us;
    pulse_count_ = isr_count_;
    period_us_ = isr_period_us_;
    magnet_present_ = isr_present_;
  }
#if !defined(__AVR__)
  interrupts();
#endif
}

uint32_t DigitalHallSensor::ageUs(uint32_t now_us) const {
  if (age_us_ == 0xFFFFFFFFUL) return age_us_;
  // The control loop's timestamp may precede an interrupt/snapshot by a few us.
  const int32_t elapsed = static_cast<int32_t>(now_us - snapshot_us_);
  if (elapsed <= 0) return age_us_;
  const uint32_t extra = static_cast<uint32_t>(elapsed);
  return extra >= 0xFFFFFFFEUL - age_us_ ? 0xFFFFFFFEUL : age_us_ + extra;
}
