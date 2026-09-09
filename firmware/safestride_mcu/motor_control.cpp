// 모터 제어 구현: 목표 ramp → 기본 FF + 경사 FF + Hall P 보정 → 상한/방향 제한 → PWM slew.
// 각속도 입력은 mrad/s, P 계산은 rad/s, 출력은 PWM count(0~255 척도)이다.
// 두 모터를 하나의 출력으로 구동하며, 좌우 독립 속도 제어는 하지 않는다.

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

uint32_t hallZeroTimeoutUs() {
  return cfg::MAGNET_BENCH_MODE
      ? cfg::MAGNET_BENCH_VELOCITY_HOLD_US
      : cfg::HALL_ZERO_TIMEOUT_US;
}

// 저속에서는 정상 펄스 간격도 길다. 예상 간격의 2.5배를 쓰되 5~10초로 제한한다.
uint32_t hallStallTimeoutUs(uint32_t target_mrad_s) {
  const uint32_t minimum_us =
      static_cast<uint32_t>(cfg::HALL_STALL_TIMEOUT_MS) * 1000UL;
  if (target_mrad_s == 0UL) {
    return minimum_us;
  }
  const float mrad_per_pulse =
      2000.0F * static_cast<float>(PI) /
      static_cast<float>(cfg::HALL_PULSES_PER_WHEEL_REV);
  const float adaptive_us =
      mrad_per_pulse * 1000000.0F /
      static_cast<float>(target_mrad_s) *
      cfg::HALL_STALL_EXPECTED_PULSE_PERIODS;
  if (adaptive_us >= cfg::HALL_STALL_MAX_TIMEOUT_US) {
    return cfg::HALL_STALL_MAX_TIMEOUT_US;
  }
  return adaptive_us > static_cast<float>(minimum_us)
      ? static_cast<uint32_t>(adaptive_us)
      : minimum_us;
}

float velocityFilterAlpha() {
  return cfg::MAGNET_BENCH_MODE
      ? cfg::MAGNET_BENCH_VELOCITY_FILTER_ALPHA
      : cfg::VELOCITY_FILTER_ALPHA;
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
      last_commanded_pwm_(0.0F),
      release_start_pwm_(0.0F),
      release_pwm_fade_active_(false),
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
  last_commanded_pwm_ = 0.0F;
  release_start_pwm_ = 0.0F;
  release_pwm_fade_active_ = false;
}

// 00 핀 상태와 PWM 0으로 BRAKE를 유지하고 제어 출력 이력을 초기화한다. 기계식 위치 고정은 아니다.
void DriveController::disableImmediately() {
  analogWrite(cfg::MOTOR_PWM_PIN, 0);
  digitalWrite(cfg::MOTOR_IN1_PIN, LOW);
  digitalWrite(cfg::MOTOR_IN2_PIN, LOW);
  braking_ = true;
  applied_pwm_counts_ = 0;
  ff_pwm_ = feedback_pwm_ = 0.0F;
  applied_target_mrad_s_ = 0.0F;
  last_commanded_pwm_ = 0.0F;
  release_start_pwm_ = 0.0F;
  release_pwm_fade_active_ = false;
  motor_pid_ = {0.0F, 0.0F};
}

void DriveController::clearRecoverableFaults() {
  hall_fault_mask_ = 0U;
  left_hall_monitor_ = {false, 0UL, 0UL, 0UL};
  right_hall_monitor_ = {false, 0UL, 0UL, 0UL};
  motor_pid_ = {0.0F, 0.0F};
}

// 목표 각속도의 변화량을 가속/감속률 × dt로 제한한다. 최종 PWM slew와 별도의 제한이다.
float DriveController::rampTarget(
    float current,
    float requested,
    float dt_seconds,
    uint32_t deceleration_mrad_s2) {
  const bool increasing_magnitude =
      current == 0.0F ||
      (current * requested > 0.0F && fabsf(requested) > fabsf(current));
  const float rate = increasing_magnitude
      ? static_cast<float>(cfg::MAX_ACCEL_MRAD_S2)
      : static_cast<float>(
            deceleration_mrad_s2 == 0UL
                ? cfg::MAX_DECEL_MRAD_S2
                : deceleration_mrad_s2);
  const float maximum_step = rate * dt_seconds;
  return current + clampFloat(
      requested - current, -maximum_step, maximum_step);
}

// mrad/s를 rad/s로 변환한 뒤 오차 = 목표 - 측정을 계산한다.
// I/D 상태 계산은 남아 있지만 현재 Ki=Kd=0이므로 출력은 Kp × 오차뿐이다.
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

  return cfg::MOTOR_PID_KP * error +
         cfg::MOTOR_PID_KI * state.integral +
         cfg::MOTOR_PID_KD * derivative;
}

// 이름과 달리 최소 PWM을 더하지 않는다. 작은 목표를 0으로 만들고 목표 반대 방향 출력을 금지한다.
float DriveController::compensateMotorDeadzone(
    float controller_pwm,
    float target_mrad_s) {
  if (!isfinite(controller_pwm) || !isfinite(target_mrad_s) ||
      fabsf(target_mrad_s) < 20.0F) {
    return 0.0F;
  }
  return target_mrad_s > 0.0F
      ? clampFloat(controller_pwm, 0.0F, cfg::MAX_PWM)
      : clampFloat(controller_pwm, -cfg::MAX_PWM, 0.0F);
}

// Hall 고장 감시를 끈 별도 개방루프 경로용이다. 정상 FF+P 계산식은 update() 아래에 있다.
float DriveController::openLoopPwm(float target_mrad_s) {
  if (fabsf(target_mrad_s) < 20.0F) {
    return 0.0F;
  }
  const float normalized = clampFloat(
      fabsf(target_mrad_s) /
          static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      0.0F,
      1.0F);
  const float pwm = static_cast<float>(cfg::MOTOR_FF_BIAS_PWM) +
      normalized * static_cast<float>(
          cfg::MAX_PWM - cfg::MOTOR_FF_BIAS_PWM);
  return target_mrad_s > 0.0F ? pwm : -pwm;
}

// 논리 출력 부호를 실제 핀 방향으로 바꾼다. 방향 반전 시 최소 150ms BRAKE를 끼우며 delay로 멈추지 않는다.
void DriveController::writeMotor(float pwm) {
  const float logical_pwm = clampFloat(
      pwm,
      -static_cast<float>(cfg::MAX_PWM),
      static_cast<float>(cfg::MAX_PWM));
  const int8_t direction = logical_pwm > 0.0F ? 1 : (logical_pwm < 0.0F ? -1 : 0);
  // DRI0042: BRAKE (00) for >0.1 s before reversing. Non-blocking.
  if (direction != 0 && last_drive_direction_ != 0 &&
      direction != last_drive_direction_) {
    reversal_remaining_us_ = cfg::MOTOR_REVERSAL_BRAKE_US;
    last_drive_direction_ = direction;
  }
  const float applied = reversal_remaining_us_ > 0UL ? 0.0F : logical_pwm;
  const float signed_pwm = applied * static_cast<float>(cfg::MOTOR_SIGN);
  const uint8_t magnitude = static_cast<uint8_t>(lroundf(fabsf(signed_pwm)));
  last_commanded_pwm_ = applied;
  braking_ = magnitude == 0U;
  applied_pwm_counts_ = direction * static_cast<int16_t>(magnitude);
  // Remove PWM before changing pins; 00 holds BRAKE, never 11 (vacant).
  analogWrite(cfg::MOTOR_PWM_PIN, 0);
  digitalWrite(cfg::MOTOR_IN1_PIN, magnitude && signed_pwm > 0.0F ? HIGH : LOW);
  digitalWrite(cfg::MOTOR_IN2_PIN, magnitude && signed_pwm < 0.0F ? HIGH : LOW);
  analogWrite(cfg::MOTOR_PWM_PIN, magnitude);
  if (magnitude) last_drive_direction_ = direction;
}

// 속도 크기 = (2π / 회전당 펄스 수) / 펄스 간격. 반환 단위는 mrad/s이다.
// 첫 펄스만으로는 간격을 알 수 없고, 5초 이상 오래된 측정도 0을 반환한다.
float DriveController::hallSpeedMagnitude(
    const HallSample& sample,
    uint32_t pulse_delta,
    uint32_t elapsed_us) {
  (void)pulse_delta;
  (void)elapsed_us;
  if (sample.age_us >= hallZeroTimeoutUs()) {
    return 0.0F;
  }
  const float mrad_per_pulse =
      2000.0F * static_cast<float>(PI) /
      static_cast<float>(cfg::HALL_PULSES_PER_WHEEL_REV);
  if (sample.period_us >= cfg::HALL_MIN_PULSE_INTERVAL_US) {
    return mrad_per_pulse * 1000000.0F /
        static_cast<float>(sample.period_us);
  }
  // One pulse establishes position but not speed. A period is available only
  // after the next magnet arrives; using the 5 ms control interval here would
  // turn the first pulse into a false extreme-speed sample and brake the motor.
  return 0.0F;
}

// 새 펄스일 때만 필터를 갱신한다: filtered += 0.35 × (raw - filtered).
// 부호는 실제 회전 방향이 아니라 목표 방향에서 추정한다. timeout의 0은 정지 확인과 다르다.
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

  new_pulse_ = left_delta != 0UL;
  speed_age_us_ = left_hall.age_us;
  speed_valid_ = left_hall.age_us < hallZeroTimeoutUs() &&
      left_hall.period_us >= cfg::HALL_MIN_PULSE_INTERVAL_US &&
      left_hall.period_us < hallZeroTimeoutUs();
  const float direction = static_cast<float>(feedback_direction_);
  const float raw_left = direction * hallSpeedMagnitude(
      left_hall, left_delta, elapsed_us);
  const float raw_right = direction * hallSpeedMagnitude(
      right_hall, right_delta, elapsed_us);
  const float alpha = clampFloat(
      velocityFilterAlpha(), 0.0F, 1.0F);

  if (left_hall.age_us >= hallZeroTimeoutUs()) {
    filtered_left_mrad_s_ = 0.0F;
  } else if (new_pulse_ && speed_valid_) {
    filtered_left_mrad_s_ +=
        alpha * (raw_left - filtered_left_mrad_s_);
  }
  if (right_hall.age_us >= hallZeroTimeoutUs()) {
    filtered_right_mrad_s_ = 0.0F;
  } else if (right_delta != 0UL && speed_valid_) {
    filtered_right_mrad_s_ +=
        alpha * (raw_right - filtered_right_mrad_s_);
  }
  if (feedback_sample_count_ < 2U) {
    ++feedback_sample_count_;
  }
}

// 매 제어 주기의 중심 함수. 출력 금지/BRAKE가 정상 FF+P 계산보다 우선한다.
void DriveController::update(
    uint32_t elapsed_us,
    const HallSample& left_hall,
    const HallSample& right_hall,
    int32_t requested_mrad_s,
    bool output_allowed,
    bool enforce_hall_faults,
    uint32_t deceleration_mrad_s2,
    bool fade_pwm_during_deceleration,
    int16_t slope_ff_pwm,
    uint8_t pwm_cap,
    bool brake_requested) {
  if (elapsed_us == 0UL) {
    return;
  }
  const float dt_seconds = static_cast<float>(elapsed_us) / 1000000.0F;
  reversal_remaining_us_ = elapsed_us >= reversal_remaining_us_
      ? 0UL : reversal_remaining_us_ - elapsed_us;
  updateHallFeedback(elapsed_us, left_hall, right_hall);

  if (!output_allowed) {
    updateHallPlausibility(
        left_hall, right_hall, elapsed_us, false);
    speed_brake_ = false;
    absolute_overspeed_pulses_ = 0U;
    disableImmediately();
    return;
  }

  // The P term handles ordinary target-speed error. Reserve a hard BRAKE for
  // consecutive, newly observed absolute overspeed periods. Re-evaluating one
  // stale period at 200 Hz made a single Hall sample look like sustained
  // overspeed and stopped the motor shortly after its second startup pulse.
  const float raw_speed = hallSpeedMagnitude(left_hall, 0UL, elapsed_us);
  if (new_pulse_ && speed_valid_) {
    if (raw_speed > cfg::SPEED_BRAKE_ABSOLUTE_MRAD_S) {
      if (absolute_overspeed_pulses_ < 0xFFU) {
        ++absolute_overspeed_pulses_;
      }
    } else {
      absolute_overspeed_pulses_ = 0U;
    }
  } else if (!speed_valid_ && !speed_brake_) {
    absolute_overspeed_pulses_ = 0U;
  }
  if (absolute_overspeed_pulses_ >= cfg::SPEED_BRAKE_CONFIRM_PULSES) {
    speed_brake_ = true;
  }
  if (speed_brake_ &&
      ((new_pulse_ && speed_valid_ &&
        raw_speed < cfg::SPEED_BRAKE_RELEASE_MRAD_S) ||
       left_hall.age_us >= cfg::HALL_ZERO_TIMEOUT_US)) {
    speed_brake_ = false;
    absolute_overspeed_pulses_ = 0U;
  }
  if (brake_requested ||
      (requested_mrad_s == 0L && !fade_pwm_during_deceleration)) {
    speed_brake_ = false;
    absolute_overspeed_pulses_ = 0U;
    updateHallPlausibility(left_hall, right_hall, elapsed_us, false);
    disableImmediately();
    return;
  }
  if (speed_brake_) {
    updateHallPlausibility(left_hall, right_hall, elapsed_us, false);
    disableImmediately();
    return;
  }

  const float limited_target = clampFloat(
      static_cast<float>(requested_mrad_s),
      -static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S),
      static_cast<float>(cfg::MAX_WHEEL_TARGET_MRAD_S));
  applied_target_mrad_s_ = rampTarget(
      applied_target_mrad_s_,
      limited_target,
      dt_seconds,
      deceleration_mrad_s2);
  if (fade_pwm_during_deceleration) {
    if (!release_pwm_fade_active_) {
      release_start_pwm_ = last_commanded_pwm_;
      release_pwm_fade_active_ = true;
    }
  } else {
    release_start_pwm_ = 0.0F;
    release_pwm_fade_active_ = false;
  }

  if (enforce_hall_faults) {
    updateHallPlausibility(
        left_hall, right_hall, elapsed_us, true);
    if (hall_fault_mask_ != 0U) {
      disableImmediately();
      return;
    }
  } else {
    hall_fault_mask_ = 0U;
    left_hall_monitor_ = {false, 0UL, 0UL, 0UL};
    right_hall_monitor_ = {false, 0UL, 0UL, 0UL};
    motor_pid_ = {0.0F, 0.0F};
    if (!fade_pwm_during_deceleration) {
      writeMotor(openLoopPwm(applied_target_mrad_s_));
      return;
    }
  }

  if (fade_pwm_during_deceleration) {
    // Fade from the actual preceding drive command so a
    // release cannot add torque before the final dynamic brake.
    motor_pid_ = {0.0F, 0.0F};
    const float starting_magnitude =
        static_cast<float>(deceleration_mrad_s2) *
        static_cast<float>(cfg::DEADMAN_RELEASE_RAMP_MS) / 1000.0F;
    const float remaining_ratio = starting_magnitude > 0.0F
        ? clampFloat(
              fabsf(applied_target_mrad_s_) / starting_magnitude,
              0.0F,
              1.0F)
        : 0.0F;
    writeMotor(release_start_pwm_ * remaining_ratio);
    return;
  }

  // 정상 제어식: u = sign(ω) × [30 + (60-30)|ω|/ω_nom] + 경사 FF + Kp(ω-측정).
  // ω_nom은 0.08m/s에 해당한다. 30은 계산 bias이며 최종 PWM의 하한이 아니다.
  const float target = applied_target_mrad_s_;
  const float direction = target >= 0.0F ? 1.0F : -1.0F;
  ff_pwm_ = fabsf(target) < 20.0F ? 0.0F : direction *
      (cfg::MOTOR_FF_BIAS_PWM +
       (cfg::MOTOR_FF_NOMINAL_PWM - cfg::MOTOR_FF_BIAS_PWM) *
       fabsf(target) / cfg::MOTOR_NOMINAL_MRAD_S);
  // Slope assistance is forward only. No positive correction from invalid
  // Hall data: bounded FF starts the wheel while the pulse monitor runs.
  if (target > 20.0F) ff_pwm_ += clampFloat(slope_ff_pwm, -60.0F, 30.0F);
  feedback_pwm_ = speed_valid_ ? calculatePid(
      target, filtered_left_mrad_s_, dt_seconds, motor_pid_) : 0.0F;
  float output = compensateMotorDeadzone(ff_pwm_ + feedback_pwm_, target);
  const float cap = fminf(pwm_cap, cfg::MAX_PWM);
  output = clampFloat(output, -cap, cap);
  if (!speed_valid_) output = clampFloat(output,
      -cfg::MOTOR_FF_NOMINAL_PWM, cfg::MOTOR_FF_NOMINAL_PWM);
  // 출력 크기를 늘릴 때 20count/s, 줄일 때 60count/s로 제한한다.
  // 0→60은 약 3초 이상 걸리며 실제 지면 속도 도달 시간과 같지 않다.
  const float rate = fabsf(output) > fabsf(last_commanded_pwm_)
      ? cfg::MOTOR_PWM_RISE_PER_S : cfg::MOTOR_PWM_FALL_PER_S;
  output = last_commanded_pwm_ + clampFloat(output - last_commanded_pwm_,
      -rate * dt_seconds, rate * dt_seconds);
  // A new cap and the requested direction apply even while slewing.
  output = clampFloat(compensateMotorDeadzone(output, target), -cap, cap);
  writeMotor(output);
}

// 바퀴 대신 손으로 자석을 움직이는 시험 전용 경로. 정상 속도 제어/고장 감시와 혼동하지 않는다.
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

// 제어 갱신을 두 번 거쳤는지만 나타낸다. 유효한 Hall 펄스 두 개를 받았다는 의미가 아니다.
bool DriveController::feedbackReady() const {
  return feedback_sample_count_ >= 2U;
}

uint8_t DriveController::hallFaultMask() const {
  return hall_fault_mask_;
}

// 출력이 있는 동안 펄스가 오래 없거나 속도 추정이 물리 상한을 넘으면 Hall 고장으로 본다.
bool DriveController::updateHallMonitor(
    HallMonitorState& state,
    uint32_t pulse_count,
    int32_t target_mrad_s,
    int32_t measured_mrad_s,
    uint32_t elapsed_us,
    bool output_allowed,
    bool motor_output_active) {
  if (!state.initialized) {
    state.initialized = true;
    state.previous_count = pulse_count;
  }
  const bool pulse_seen = pulse_count != state.previous_count;
  state.previous_count = pulse_count;

  if (!output_allowed) {
    state.no_pulse_us = 0UL;
    state.overspeed_pulses = 0U;
    return false;
  }

  const uint32_t target_magnitude = magnitudeInt32(target_mrad_s);
  const uint32_t measured_magnitude = magnitudeInt32(measured_mrad_s);
  const bool target_requests_motion =
      target_magnitude >= static_cast<uint32_t>(
          cfg::HALL_STALL_TARGET_MIN_MRAD_S);
  updateTimer(
      target_requests_motion && motor_output_active && !pulse_seen,
      elapsed_us,
      state.no_pulse_us);
  // Velocity changes only when a new pulse period is observed. Counting the
  // same cached value every 5 ms turns one noisy period into a latched fault.
  if (pulse_seen) {
    if (measured_magnitude > static_cast<uint32_t>(
            cfg::HALL_MAX_PLAUSIBLE_MRAD_S)) {
      if (state.overspeed_pulses < 0xFFU) {
        ++state.overspeed_pulses;
      }
    } else {
      state.overspeed_pulses = 0U;
    }
  }

  return (
      state.no_pulse_us >= hallStallTimeoutUs(target_magnitude) ||
      state.overspeed_pulses >= cfg::HALL_OVERSPEED_CONFIRM_PULSES);
}

void DriveController::updateHallPlausibility(
    const HallSample& left_hall,
    const HallSample& right_hall,
    uint32_t elapsed_us,
    bool output_allowed) {
  if (hall_fault_mask_ != 0U) {
    return;
  }
  const bool motor_output_active =
      fabsf(last_commanded_pwm_) >= 0.5F;
  if (updateHallMonitor(
          left_hall_monitor_,
          left_hall.pulse_count,
          appliedTargetMradS(),
          leftVelocityMradS(),
          elapsed_us,
          output_allowed,
          motor_output_active)) {
    hall_fault_mask_ |= HALL_FAULT_LEFT;
  }
  (void)right_hall;
}
