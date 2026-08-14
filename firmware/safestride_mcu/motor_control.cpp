#include "motor_control.h"

#include <math.h>

#include "config.h"

namespace cfg = safestride_config;

namespace {

float clampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

int32_t roundedInt32(float value) {
  if (value >= 2147483647.0F) {
    return 2147483647L;
  }
  if (value <= -2147483648.0F) {
    return (-2147483647L - 1L);
  }
  return static_cast<int32_t>(lroundf(value));
}

uint32_t magnitudeInt32(int32_t value) {
  if (value >= 0L) {
    return static_cast<uint32_t>(value);
  }
  if (value == (-2147483647L - 1L)) {
    return 0x80000000UL;
  }
  return static_cast<uint32_t>(-value);
}

void updateTimer(
    bool condition,
    uint32_t elapsed_us,
    uint32_t& accumulated_us) {
  if (!condition) {
    accumulated_us = 0UL;
    return;
  }
  if (0xFFFFFFFFUL - accumulated_us < elapsed_us) {
    accumulated_us = 0xFFFFFFFFUL;
  } else {
    accumulated_us += elapsed_us;
  }
}

}  // namespace

DriveController::DriveController()
    : feedback_initialized_(false),
      feedback_sample_count_(0U),
      previous_left_pulse_count_(0UL),
      previous_right_pulse_count_(0UL),
      left_position_bits_(0UL),
      right_position_bits_(0UL),
      feedback_direction_(1),
      filtered_left_mrad_s_(0.0F),
      filtered_right_mrad_s_(0.0F),
      applied_target_mrad_s_(0.0F),
      motor_pid_{0.0F, 0.0F},
      left_hall_monitor_{false, 0UL, 0UL, 0UL},
      right_hall_monitor_{false, 0UL, 0UL, 0UL},
      hall_fault_mask_(0U) {}

void DriveController::begin() {
  digitalWrite(cfg::MOTOR_PWM_PIN, LOW);
  digitalWrite(cfg::MOTOR_IN1_PIN, LOW);
  digitalWrite(cfg::MOTOR_IN2_PIN, LOW);
  pinMode(cfg::MOTOR_PWM_PIN, OUTPUT);
  pinMode(cfg::MOTOR_IN1_PIN, OUTPUT);
  pinMode(cfg::MOTOR_IN2_PIN, OUTPUT);
  analogWrite(cfg::MOTOR_PWM_PIN, 0);
}

void DriveController::disableImmediately() {
  analogWrite(cfg::MOTOR_PWM_PIN, 0);
  digitalWrite(cfg::MOTOR_IN1_PIN, LOW);
  digitalWrite(cfg::MOTOR_IN2_PIN, LOW);
  applied_target_mrad_s_ = 0.0F;
  motor_pid_ = {0.0F, 0.0F};
}

float DriveController::rampTarget(
    float current,
    float requested,
    float dt_seconds) {
  const bool increasing_magnitude =
      current == 0.0F ||
      (current * requested > 0.0F && fabsf(requested) > fabsf(current));
  const float rate = increasing_magnitude
      ? static_cast<float>(cfg::MAX_ACCEL_MRAD_S2)
      : static_cast<float>(cfg::MAX_DECEL_MRAD_S2);
  const float maximum_step = rate * dt_seconds;
  return current + clampFloat(
      requested - current, -maximum_step, maximum_step);
}

float DriveController::calculatePid(
    float target_mrad_s,
    float measured_mrad_s,
    float dt_seconds,
    PidState& state) {
  const float target_rad_s = target_mrad_s / 1000.0F;
  const float measured_rad_s = measured_mrad_s / 1000.0F;
  const float error = target_rad_s - measured_rad_s;

  state.integral += error * dt_seconds;
  state.integral = clampFloat(
      state.integral,
      -cfg::PID_INTEGRAL_LIMIT,
      cfg::PID_INTEGRAL_LIMIT);
  const float derivative = dt_seconds > 0.0F
      ? (error - state.previous_error) / dt_seconds
      : 0.0F;
  state.previous_error = error;

  if (fabsf(target_rad_s) < 0.02F && fabsf(measured_rad_s) < 0.05F) {
    state.integral = 0.0F;
    state.previous_error = 0.0F;
    return 0.0F;
  }

  return cfg::MOTOR_FEEDFORWARD * target_rad_s +
         cfg::MOTOR_PID_KP * error +
         cfg::MOTOR_PID_KI * state.integral +
         cfg::MOTOR_PID_KD * derivative;
}

void DriveController::writeMotor(float pwm) {
  float signed_pwm = pwm * static_cast<float>(cfg::MOTOR_SIGN);
  signed_pwm = clampFloat(
      signed_pwm,
      -static_cast<float>(cfg::MAX_PWM),
      static_cast<float>(cfg::MAX_PWM));
  const uint8_t magnitude = static_cast<uint8_t>(
      lroundf(fabsf(signed_pwm)));
  if (signed_pwm > 0.0F) {
    digitalWrite(cfg::MOTOR_IN1_PIN, HIGH);
    digitalWrite(cfg::MOTOR_IN2_PIN, LOW);
  } else if (signed_pwm < 0.0F) {
    digitalWrite(cfg::MOTOR_IN1_PIN, LOW);
    digitalWrite(cfg::MOTOR_IN2_PIN, HIGH);
  } else {
    digitalWrite(cfg::MOTOR_IN1_PIN, LOW);
    digitalWrite(cfg::MOTOR_IN2_PIN, LOW);
  }
  analogWrite(cfg::MOTOR_PWM_PIN, magnitude);
}

float DriveController::hallSpeedMagnitude(
    const HallSample& sample,
    uint32_t pulse_delta,
    uint32_t elapsed_us) {
  if (sample.age_us >= cfg::HALL_ZERO_TIMEOUT_US) {
    return 0.0F;
  }
  const float mrad_per_pulse =
      2000.0F * static_cast<float>(PI) /
      static_cast<float>(cfg::HALL_PULSES_PER_WHEEL_REV);
  if (sample.period_us >= cfg::HALL_MIN_PULSE_INTERVAL_US) {
    return mrad_per_pulse * 1000000.0F /
        static_cast<float>(sample.period_us);
  }
  if (pulse_delta == 0UL || elapsed_us == 0UL) {
    return 0.0F;
  }
  return static_cast<float>(pulse_delta) * mrad_per_pulse * 1000000.0F /
      static_cast<float>(elapsed_us);
}

void DriveController::updateHallFeedback(
    uint32_t elapsed_us,
    const HallSample& left_hall,
    const HallSample& right_hall) {
  if (!feedback_initialized_) {
    previous_left_pulse_count_ = left_hall.pulse_count;
    previous_right_pulse_count_ = right_hall.pulse_count;
    feedback_initialized_ = true;
  }

  const uint32_t left_delta =
      left_hall.pulse_count - previous_left_pulse_count_;
  const uint32_t right_delta =
      right_hall.pulse_count - previous_right_pulse_count_;
  previous_left_pulse_count_ = left_hall.pulse_count;
  previous_right_pulse_count_ = right_hall.pulse_count;

  if (applied_target_mrad_s_ > 20.0F) {
    feedback_direction_ = 1;
  } else if (applied_target_mrad_s_ < -20.0F) {
    feedback_direction_ = -1;
  }
  if (feedback_direction_ > 0) {
    left_position_bits_ += left_delta;
    right_position_bits_ += right_delta;
  } else {
    left_position_bits_ -= left_delta;
    right_position_bits_ -= right_delta;
  }

  const float direction = static_cast<float>(feedback_direction_);
  const float raw_left = direction * hallSpeedMagnitude(
      left_hall, left_delta, elapsed_us);
  const float raw_right = direction * hallSpeedMagnitude(
      right_hall, right_delta, elapsed_us);
  const float alpha = clampFloat(
      cfg::VELOCITY_FILTER_ALPHA, 0.0F, 1.0F);

  if (left_hall.age_us >= cfg::HALL_ZERO_TIMEOUT_US) {
    filtered_left_mrad_s_ = 0.0F;
  } else {
    filtered_left_mrad_s_ +=
        alpha * (raw_left - filtered_left_mrad_s_);
  }
  if (right_hall.age_us >= cfg::HALL_ZERO_TIMEOUT_US) {
    filtered_right_mrad_s_ = 0.0F;
  } else {
    filtered_right_mrad_s_ +=
        alpha * (raw_right - filtered_right_mrad_s_);
  }
  if (feedback_sample_count_ < 2U) {
    ++feedback_sample_count_;
  }
}

void DriveController::update(
    uint32_t elapsed_us,
    const HallSample& left_hall,
    const HallSample& right_hall,
    int32_t requested_mrad_s,
    bool output_allowed) {
  if (elapsed_us == 0UL) {
    return;
  }
  const float dt_seconds = static_cast<float>(elapsed_us) / 1000000.0F;
  updateHallFeedback(elapsed_us, left_hall, right_hall);

  if (!output_allowed) {
    updateHallPlausibility(
        left_hall, right_hall, elapsed_us, false);
    disableImmediately();
    return;
  }

  const float limited_target = clampFloat(
      static_cast<float>(requested_mrad_s),
      -static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S));
  applied_target_mrad_s_ = rampTarget(
      applied_target_mrad_s_, limited_target, dt_seconds);

  updateHallPlausibility(
      left_hall, right_hall, elapsed_us, true);
  if (hall_fault_mask_ != 0U) {
    disableImmediately();
    return;
  }

  const float measured_average =
      0.5F * (filtered_left_mrad_s_ + filtered_right_mrad_s_);
  writeMotor(calculatePid(
      applied_target_mrad_s_, measured_average, dt_seconds, motor_pid_));
}

void DriveController::updateMagnetBench(
    uint32_t elapsed_us,
    const HallSample& left_hall,
    const HallSample& right_hall,
    int32_t requested_mrad_s,
    bool output_allowed) {
  if (elapsed_us == 0UL) {
    return;
  }

  const float limited_target = clampFloat(
      static_cast<float>(requested_mrad_s),
      -static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S));
  applied_target_mrad_s_ = limited_target;
  updateHallFeedback(elapsed_us, left_hall, right_hall);

  // Calibration-dependent stall and overspeed checks are meaningless while
  // a hand-held magnet, rather than a rotating wheel, produces the pulses.
  hall_fault_mask_ = 0U;
  left_hall_monitor_ = {false, 0UL, 0UL, 0UL};
  right_hall_monitor_ = {false, 0UL, 0UL, 0UL};
  motor_pid_ = {0.0F, 0.0F};

  if (!output_allowed || limited_target == 0.0F) {
    disableImmediately();
    return;
  }
  writeMotor(
      limited_target > 0.0F
          ? static_cast<float>(cfg::MAGNET_BENCH_PWM)
          : -static_cast<float>(cfg::MAGNET_BENCH_PWM));
}

int32_t DriveController::leftVelocityMradS() const {
  return roundedInt32(filtered_left_mrad_s_);
}

int32_t DriveController::rightVelocityMradS() const {
  return roundedInt32(filtered_right_mrad_s_);
}

int32_t DriveController::appliedTargetMradS() const {
  return roundedInt32(applied_target_mrad_s_);
}

int32_t DriveController::leftHallPulsePosition() const {
  return static_cast<int32_t>(left_position_bits_);
}

int32_t DriveController::rightHallPulsePosition() const {
  return static_cast<int32_t>(right_position_bits_);
}

bool DriveController::feedbackReady() const {
  return feedback_sample_count_ >= 2U;
}

uint8_t DriveController::hallFaultMask() const {
  return hall_fault_mask_;
}

bool DriveController::updateHallMonitor(
    HallMonitorState& state,
    uint32_t pulse_count,
    int32_t target_mrad_s,
    int32_t measured_mrad_s,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (!state.initialized) {
    state.initialized = true;
    state.previous_count = pulse_count;
  }
  const bool pulse_seen = pulse_count != state.previous_count;
  state.previous_count = pulse_count;

  if (!output_allowed) {
    state.no_pulse_us = 0UL;
    state.overspeed_us = 0UL;
    return false;
  }

  const uint32_t target_magnitude = magnitudeInt32(target_mrad_s);
  const uint32_t measured_magnitude = magnitudeInt32(measured_mrad_s);
  const bool target_requests_motion =
      target_magnitude >= static_cast<uint32_t>(
          cfg::HALL_STALL_TARGET_MIN_MRAD_S);
  updateTimer(
      target_requests_motion && !pulse_seen,
      elapsed_us,
      state.no_pulse_us);
  updateTimer(
      measured_magnitude > static_cast<uint32_t>(
          cfg::HALL_MAX_PLAUSIBLE_MRAD_S),
      elapsed_us,
      state.overspeed_us);

  return (
      state.no_pulse_us >=
          static_cast<uint32_t>(cfg::HALL_STALL_TIMEOUT_MS) * 1000UL ||
      state.overspeed_us >=
          static_cast<uint32_t>(cfg::HALL_OVERSPEED_TIMEOUT_MS) * 1000UL);
}

void DriveController::updateHallPlausibility(
    const HallSample& left_hall,
    const HallSample& right_hall,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (hall_fault_mask_ != 0U) {
    return;
  }
  if (updateHallMonitor(
          left_hall_monitor_,
          left_hall.pulse_count,
          appliedTargetMradS(),
          leftVelocityMradS(),
          elapsed_us,
          output_allowed)) {
    hall_fault_mask_ |= HALL_FAULT_LEFT;
  }
  if (updateHallMonitor(
          right_hall_monitor_,
          right_hall.pulse_count,
          appliedTargetMradS(),
          rightVelocityMradS(),
          elapsed_us,
          output_allowed)) {
    hall_fault_mask_ |= HALL_FAULT_RIGHT;
  }
}
