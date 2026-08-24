#include "motor_control.h"

#include <math.h>

#include "config.h"

namespace cfg = safestride_config;

namespace {

float clampFloat(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

int32_t roundedInt32(float value) {
  if (value >= 2147483647.0F) return 2147483647L;
  if (value <= -2147483648.0F) return (-2147483647L - 1L);
  return static_cast<int32_t>(lroundf(value));
}

uint32_t magnitudeInt32(int32_t value) {
  if (value >= 0L) return static_cast<uint32_t>(value);
  if (value == (-2147483647L - 1L)) return 0x80000000UL;
  return static_cast<uint32_t>(-value);
}

void updateTimer(
    bool condition,
    uint32_t elapsed_us,
    uint32_t& accumulated_us) {
  if (!condition) {
    accumulated_us = 0UL;
  } else if (0xFFFFFFFFUL - accumulated_us < elapsed_us) {
    accumulated_us = 0xFFFFFFFFUL;
  } else {
    accumulated_us += elapsed_us;
  }
}

}  // namespace

DriveController::DriveController()
    : feedback_ready_(false),
      left_position_mrad_(0L),
      right_position_mrad_(0L),
      left_velocity_mrad_s_(0L),
      right_velocity_mrad_s_(0L),
      applied_target_mrad_s_(0.0F),
      motor_pid_{0.0F, 0.0F},
      left_encoder_monitor_{false, 0L, 0UL, 0UL},
      right_encoder_monitor_{false, 0L, 0UL, 0UL},
      encoder_fault_mask_(0U) {}

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
    float current, float requested, float dt_seconds) {
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
  state.integral = clampFloat(
      state.integral + error * dt_seconds,
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

float DriveController::compensateMotorDeadzone(
    float controller_pwm, float target_mrad_s) {
  if (fabsf(target_mrad_s) < 20.0F) return 0.0F;
  // Coasting is safer than reversing the shared motor load to correct a small
  // overspeed. Direction reversal still requires a new signed target.
  if (target_mrad_s > 0.0F) {
    if (controller_pwm <= 0.0F) return 0.0F;
    return clampFloat(
        controller_pwm,
        static_cast<float>(cfg::MOTOR_MIN_ACTIVE_PWM),
        static_cast<float>(cfg::MAX_PWM));
  }
  if (controller_pwm >= 0.0F) return 0.0F;
  return clampFloat(
      controller_pwm,
      -static_cast<float>(cfg::MAX_PWM),
      -static_cast<float>(cfg::MOTOR_MIN_ACTIVE_PWM));
}

float DriveController::openLoopPwm(float target_mrad_s) {
  if (fabsf(target_mrad_s) < 20.0F) return 0.0F;
  const float normalized = clampFloat(
      fabsf(target_mrad_s) /
          static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      0.0F,
      1.0F);
  const float pwm = static_cast<float>(cfg::MOTOR_MIN_ACTIVE_PWM) +
      normalized * static_cast<float>(
          cfg::MAX_PWM - cfg::MOTOR_MIN_ACTIVE_PWM);
  return target_mrad_s > 0.0F ? pwm : -pwm;
}

void DriveController::writeMotor(float pwm) {
  float signed_pwm = clampFloat(
      pwm * static_cast<float>(cfg::MOTOR_SIGN),
      -static_cast<float>(cfg::MAX_PWM),
      static_cast<float>(cfg::MAX_PWM));
  const uint8_t magnitude = static_cast<uint8_t>(lroundf(fabsf(signed_pwm)));
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

void DriveController::updateEncoderFeedback(
    const WheelEncoderSample& encoder) {
  feedback_ready_ = encoder.valid;
  if (!encoder.valid) {
    left_velocity_mrad_s_ = 0L;
    right_velocity_mrad_s_ = 0L;
    return;
  }
  left_position_mrad_ = encoder.left_position_mrad;
  right_position_mrad_ = encoder.right_position_mrad;
  left_velocity_mrad_s_ = encoder.left_velocity_mrad_s;
  right_velocity_mrad_s_ = encoder.right_velocity_mrad_s;
}

void DriveController::update(
    uint32_t elapsed_us,
    const WheelEncoderSample& encoder,
    int32_t requested_mrad_s,
    bool output_allowed) {
  if (elapsed_us == 0UL) return;
  updateEncoderFeedback(encoder);
  if (!output_allowed) {
    if (cfg::ENABLE_ENCODER_FEEDBACK && encoder.valid) {
      updateEncoderPlausibility(encoder, elapsed_us, false);
    }
    disableImmediately();
    return;
  }

  const float limited_target = clampFloat(
      static_cast<float>(requested_mrad_s),
      -static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S));
  if (!cfg::ENABLE_ENCODER_FEEDBACK) {
    if (!cfg::ALLOW_OPEN_LOOP_MOTOR) {
      disableImmediately();
      return;
    }
    applied_target_mrad_s_ = limited_target;
    motor_pid_ = {0.0F, 0.0F};
    writeMotor(openLoopPwm(applied_target_mrad_s_));
    return;
  }

  // Feedback selection without a calibrated, valid adapter is a hard inhibit.
  // It must never silently fall back to open-loop output.
  if (!cfg::ENCODER_CALIBRATED || !encoder.valid) {
    encoder_fault_mask_ |= ENCODER_FAULT_LEFT | ENCODER_FAULT_RIGHT;
    disableImmediately();
    return;
  }

  const float dt_seconds = static_cast<float>(elapsed_us) / 1000000.0F;
  applied_target_mrad_s_ = rampTarget(
      applied_target_mrad_s_, limited_target, dt_seconds);
  updateEncoderPlausibility(encoder, elapsed_us, true);
  if (encoder_fault_mask_ != 0U) {
    disableImmediately();
    return;
  }
  const float measured_average = 0.5F * (
      static_cast<float>(left_velocity_mrad_s_) +
      static_cast<float>(right_velocity_mrad_s_));
  const float controller_pwm = calculatePid(
      applied_target_mrad_s_, measured_average, dt_seconds, motor_pid_);
  writeMotor(compensateMotorDeadzone(controller_pwm, applied_target_mrad_s_));
}

int32_t DriveController::leftVelocityMradS() const {
  return left_velocity_mrad_s_;
}
int32_t DriveController::rightVelocityMradS() const {
  return right_velocity_mrad_s_;
}
int32_t DriveController::appliedTargetMradS() const {
  return roundedInt32(applied_target_mrad_s_);
}
int32_t DriveController::leftPositionMrad() const {
  return left_position_mrad_;
}
int32_t DriveController::rightPositionMrad() const {
  return right_position_mrad_;
}
bool DriveController::feedbackReady() const {
  return feedback_ready_;
}
uint8_t DriveController::encoderFaultMask() const {
  return encoder_fault_mask_;
}

bool DriveController::updateEncoderMonitor(
    EncoderMonitorState& state,
    int32_t position_mrad,
    int32_t target_mrad_s,
    int32_t measured_mrad_s,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (!state.initialized) {
    state.initialized = true;
    state.previous_position_mrad = position_mrad;
  }
  const bool motion_seen = position_mrad != state.previous_position_mrad;
  state.previous_position_mrad = position_mrad;
  if (!output_allowed) {
    state.no_motion_us = 0UL;
    state.overspeed_us = 0UL;
    return false;
  }
  const bool target_requests_motion =
      magnitudeInt32(target_mrad_s) >= static_cast<uint32_t>(
          cfg::ENCODER_STALL_TARGET_MIN_MRAD_S);
  updateTimer(
      target_requests_motion && !motion_seen,
      elapsed_us,
      state.no_motion_us);
  updateTimer(
      magnitudeInt32(measured_mrad_s) > static_cast<uint32_t>(
          cfg::ENCODER_MAX_PLAUSIBLE_MRAD_S),
      elapsed_us,
      state.overspeed_us);
  return (
      state.no_motion_us >=
          static_cast<uint32_t>(cfg::ENCODER_STALL_TIMEOUT_MS) * 1000UL ||
      state.overspeed_us >=
          static_cast<uint32_t>(cfg::ENCODER_OVERSPEED_TIMEOUT_MS) * 1000UL);
}

void DriveController::updateEncoderPlausibility(
    const WheelEncoderSample& encoder,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (encoder_fault_mask_ != 0U || !encoder.valid) return;
  if (updateEncoderMonitor(
          left_encoder_monitor_, encoder.left_position_mrad,
          appliedTargetMradS(), encoder.left_velocity_mrad_s,
          elapsed_us, output_allowed)) {
    encoder_fault_mask_ |= ENCODER_FAULT_LEFT;
  }
  if (updateEncoderMonitor(
          right_encoder_monitor_, encoder.right_position_mrad,
          appliedTargetMradS(), encoder.right_velocity_mrad_s,
          elapsed_us, output_allowed)) {
    encoder_fault_mask_ |= ENCODER_FAULT_RIGHT;
  }
}
