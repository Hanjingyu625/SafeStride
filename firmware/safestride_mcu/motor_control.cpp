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

int32_t wrappingDelta(uint32_t current, uint32_t previous) {
  const uint32_t difference = current - previous;
  if ((difference & 0x80000000UL) == 0UL) {
    return static_cast<int32_t>(difference);
  }
  const uint32_t magnitude = (~difference) + 1UL;
  if (magnitude == 0x80000000UL) {
    return (-2147483647L - 1L);
  }
  return -static_cast<int32_t>(magnitude);
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
      previous_left_count_(0),
      previous_right_count_(0),
      filtered_left_mrad_s_(0.0F),
      filtered_right_mrad_s_(0.0F),
      applied_left_mrad_s_(0.0F),
      applied_right_mrad_s_(0.0F),
      left_pid_{0.0F, 0.0F},
      right_pid_{0.0F, 0.0F},
      left_encoder_monitor_{false, 0UL, 0UL, 0UL, 0UL},
      right_encoder_monitor_{false, 0UL, 0UL, 0UL, 0UL},
      encoder_fault_mask_(0U) {}

void DriveController::begin() {
  // Preload the output latch before changing pin direction. This avoids an
  // active-low enable pulse while the pin transitions from input to output.
  digitalWrite(
      cfg::MOTOR_DRIVER_ENABLE_PIN,
      cfg::MOTOR_DRIVER_ENABLE_INACTIVE_LEVEL);
  pinMode(cfg::MOTOR_DRIVER_ENABLE_PIN, OUTPUT);
  digitalWrite(
      cfg::MOTOR_DRIVER_ENABLE_PIN,
      cfg::MOTOR_DRIVER_ENABLE_INACTIVE_LEVEL);

  pinMode(cfg::LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(cfg::LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(cfg::RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(cfg::RIGHT_MOTOR_DIR_PIN, OUTPUT);
  analogWrite(cfg::LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(cfg::RIGHT_MOTOR_PWM_PIN, 0);
  digitalWrite(cfg::LEFT_MOTOR_DIR_PIN, LOW);
  digitalWrite(cfg::RIGHT_MOTOR_DIR_PIN, LOW);
}

void DriveController::disableImmediately() {
  digitalWrite(
      cfg::MOTOR_DRIVER_ENABLE_PIN,
      cfg::MOTOR_DRIVER_ENABLE_INACTIVE_LEVEL);
  analogWrite(cfg::LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(cfg::RIGHT_MOTOR_PWM_PIN, 0);
  applied_left_mrad_s_ = 0.0F;
  applied_right_mrad_s_ = 0.0F;
  left_pid_ = {0.0F, 0.0F};
  right_pid_ = {0.0F, 0.0F};
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
    float kp,
    float ki,
    float kd,
    float feedforward,
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

  return feedforward * target_rad_s +
         kp * error +
         ki * state.integral +
         kd * derivative;
}

void DriveController::writeMotor(
    uint8_t pwm_pin,
    uint8_t direction_pin,
    int8_t motor_sign,
    float pwm) {
  float signed_pwm = pwm * static_cast<float>(motor_sign);
  signed_pwm = clampFloat(
      signed_pwm,
      -static_cast<float>(cfg::MAX_PWM),
      static_cast<float>(cfg::MAX_PWM));
  const bool forward = signed_pwm >= 0.0F;
  const uint8_t magnitude = static_cast<uint8_t>(
      lroundf(fabsf(signed_pwm)));
  digitalWrite(direction_pin, forward ? HIGH : LOW);
  analogWrite(pwm_pin, magnitude);
}

void DriveController::update(
    uint32_t elapsed_us,
    uint32_t left_encoder_count,
    uint32_t right_encoder_count,
    int32_t left_requested_mrad_s,
    int32_t right_requested_mrad_s,
    bool output_allowed) {
  if (elapsed_us == 0U) {
    return;
  }
  const float dt_seconds = static_cast<float>(elapsed_us) / 1000000.0F;

  if (!feedback_initialized_) {
    previous_left_count_ = left_encoder_count;
    previous_right_count_ = right_encoder_count;
    feedback_initialized_ = true;
  }

  const int32_t left_delta = wrappingDelta(
      left_encoder_count, previous_left_count_);
  const int32_t right_delta = wrappingDelta(
      right_encoder_count, previous_right_count_);
  previous_left_count_ = left_encoder_count;
  previous_right_count_ = right_encoder_count;

  const float mrad_per_count =
      2000.0F * PI / cfg::ENCODER_COUNTS_PER_WHEEL_REV;
  const float raw_left =
      static_cast<float>(left_delta) * mrad_per_count / dt_seconds;
  const float raw_right =
      static_cast<float>(right_delta) * mrad_per_count / dt_seconds;
  const float alpha = clampFloat(
      cfg::VELOCITY_FILTER_ALPHA, 0.0F, 1.0F);
  filtered_left_mrad_s_ += alpha * (raw_left - filtered_left_mrad_s_);
  filtered_right_mrad_s_ +=
      alpha * (raw_right - filtered_right_mrad_s_);
  if (feedback_sample_count_ < 2U) {
    ++feedback_sample_count_;
  }

  if (!output_allowed) {
    updateEncoderPlausibility(
        left_encoder_count,
        right_encoder_count,
        elapsed_us,
        false);
    disableImmediately();
    return;
  }

  const float limited_left = clampFloat(
      static_cast<float>(left_requested_mrad_s),
      -static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S));
  const float limited_right = clampFloat(
      static_cast<float>(right_requested_mrad_s),
      -static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S));
  applied_left_mrad_s_ = rampTarget(
      applied_left_mrad_s_, limited_left, dt_seconds);
  applied_right_mrad_s_ = rampTarget(
      applied_right_mrad_s_, limited_right, dt_seconds);

  updateEncoderPlausibility(
      left_encoder_count,
      right_encoder_count,
      elapsed_us,
      true);
  if (encoder_fault_mask_ != 0U) {
    disableImmediately();
    return;
  }

  const float left_pwm = calculatePid(
      applied_left_mrad_s_,
      filtered_left_mrad_s_,
      dt_seconds,
      cfg::LEFT_PID_KP,
      cfg::LEFT_PID_KI,
      cfg::LEFT_PID_KD,
      cfg::LEFT_FEEDFORWARD,
      left_pid_);
  const float right_pwm = calculatePid(
      applied_right_mrad_s_,
      filtered_right_mrad_s_,
      dt_seconds,
      cfg::RIGHT_PID_KP,
      cfg::RIGHT_PID_KI,
      cfg::RIGHT_PID_KD,
      cfg::RIGHT_FEEDFORWARD,
      right_pid_);

  writeMotor(
      cfg::LEFT_MOTOR_PWM_PIN,
      cfg::LEFT_MOTOR_DIR_PIN,
      cfg::LEFT_MOTOR_SIGN,
      left_pwm);
  writeMotor(
      cfg::RIGHT_MOTOR_PWM_PIN,
      cfg::RIGHT_MOTOR_DIR_PIN,
      cfg::RIGHT_MOTOR_SIGN,
      right_pwm);
  digitalWrite(
      cfg::MOTOR_DRIVER_ENABLE_PIN,
      cfg::MOTOR_DRIVER_ENABLE_ACTIVE_LEVEL);
}

int32_t DriveController::leftVelocityMradS() const {
  return roundedInt32(filtered_left_mrad_s_);
}

int32_t DriveController::rightVelocityMradS() const {
  return roundedInt32(filtered_right_mrad_s_);
}

int32_t DriveController::leftAppliedTargetMradS() const {
  return roundedInt32(applied_left_mrad_s_);
}

int32_t DriveController::rightAppliedTargetMradS() const {
  return roundedInt32(applied_right_mrad_s_);
}

bool DriveController::feedbackReady() const {
  return feedback_sample_count_ >= 2U;
}

uint8_t DriveController::encoderFaultMask() const {
  return encoder_fault_mask_;
}

bool DriveController::updateEncoderMonitor(
    EncoderMonitorState& state,
    uint32_t encoder_count,
    int32_t target_mrad_s,
    int32_t measured_mrad_s,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (!state.initialized) {
    state.initialized = true;
    state.previous_count = encoder_count;
  }
  const bool edge_seen = encoder_count != state.previous_count;
  state.previous_count = encoder_count;

  if (!output_allowed) {
    state.no_edge_us = 0UL;
    state.reverse_us = 0UL;
    state.overspeed_us = 0UL;
    return false;
  }

  const uint32_t target_magnitude = magnitudeInt32(target_mrad_s);
  const uint32_t measured_magnitude = magnitudeInt32(measured_mrad_s);
  const bool target_requests_motion =
      target_magnitude >= static_cast<uint32_t>(
          cfg::ENCODER_STALL_TARGET_MIN_MRAD_S);
  updateTimer(
      target_requests_motion && !edge_seen,
      elapsed_us,
      state.no_edge_us);

  const bool opposite_direction =
      measured_magnitude >= static_cast<uint32_t>(
          cfg::ENCODER_REVERSE_MIN_MRAD_S) &&
      ((target_mrad_s > 0L && measured_mrad_s < 0L) ||
       (target_mrad_s < 0L && measured_mrad_s > 0L));
  updateTimer(
      target_requests_motion && opposite_direction,
      elapsed_us,
      state.reverse_us);

  updateTimer(
      measured_magnitude > static_cast<uint32_t>(
          cfg::ENCODER_MAX_PLAUSIBLE_MRAD_S),
      elapsed_us,
      state.overspeed_us);

  return (
      state.no_edge_us >=
          static_cast<uint32_t>(cfg::ENCODER_STALL_TIMEOUT_MS) *
              1000UL ||
      state.reverse_us >=
          static_cast<uint32_t>(cfg::ENCODER_REVERSE_TIMEOUT_MS) *
              1000UL ||
      state.overspeed_us >=
          static_cast<uint32_t>(
              cfg::ENCODER_OVERSPEED_TIMEOUT_MS) *
              1000UL);
}

void DriveController::updateEncoderPlausibility(
    uint32_t left_encoder_count,
    uint32_t right_encoder_count,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (encoder_fault_mask_ != 0U) {
    return;
  }
  if (updateEncoderMonitor(
          left_encoder_monitor_,
          left_encoder_count,
          leftAppliedTargetMradS(),
          leftVelocityMradS(),
          elapsed_us,
          output_allowed)) {
    encoder_fault_mask_ |= ENCODER_FAULT_LEFT;
  }
  if (updateEncoderMonitor(
          right_encoder_monitor_,
          right_encoder_count,
          rightAppliedTargetMradS(),
          rightVelocityMradS(),
          elapsed_us,
          output_allowed)) {
    encoder_fault_mask_ |= ENCODER_FAULT_RIGHT;
  }
}
