// 모터 제어 인터페이스와 내부 상태. update()는 실제 경과시간(us)을 받아 한 제어 단계를 수행한다.
// braking은 전기적 BRAKE 출력 상태이며, 물리적으로 정지했거나 경사에서 고정됐다는 뜻은 아니다.

#pragma once

#include <Arduino.h>

// 한 시점의 Hall 스냅샷. 누적 count와 직전 period/현재 age를 함께 전달해 오래된 측정을 구분한다.
struct HallSample {
  uint32_t pulse_count;
  uint32_t period_us;
  uint32_t age_us;
};

class DriveController {
 public:
  static const uint8_t HALL_FAULT_LEFT = 1U << 0U;

  DriveController();

  void begin();
  // output_allowed는 최종 출력 허용, enforce_hall_faults는 Hall 감시/피드백 경로 선택이다.
  // deceleration=0이면 기본 감속률, fade는 손 해제용, brake_requested는 Pi의 명시적 BRAKE다.
  void update(
      uint32_t elapsed_us,
      const HallSample& left_hall,
      const HallSample& right_hall,
      int32_t requested_mrad_s,
      bool output_allowed,
      bool enforce_hall_faults = true,
      uint32_t deceleration_mrad_s2 = 0UL,
      bool fade_pwm_during_deceleration = false,
      int16_t slope_ff_pwm = 0,
      uint8_t pwm_cap = 100U,
      bool brake_requested = false);
  void updateMagnetBench(
      uint32_t elapsed_us,
      const HallSample& left_hall,
      const HallSample& right_hall,
      int32_t requested_mrad_s,
      bool output_allowed);
  void disableImmediately();
  void clearRecoverableFaults();

  int32_t leftVelocityMradS() const;
  int32_t rightVelocityMradS() const;
  int32_t appliedTargetMradS() const;
  int32_t leftHallPulsePosition() const;
  int32_t rightHallPulsePosition() const;
  bool feedbackReady() const;
  bool speedValid() const { return speed_valid_; }
  bool newPulse() const { return new_pulse_; }
  uint32_t speedAgeUs() const { return speed_age_us_; }
  int16_t feedforwardPwm() const { return static_cast<int16_t>(ff_pwm_); }
  int16_t feedbackPwm() const { return static_cast<int16_t>(feedback_pwm_); }
  int16_t appliedPwm() const { return applied_pwm_counts_; }
  bool braking() const { return braking_; }
  uint8_t hallFaultMask() const;

 private:
  struct PidState {
    float integral;
    float previous_error;
  };

  struct HallMonitorState {
    bool initialized;
    uint32_t previous_count;
    uint32_t no_pulse_us;
    uint8_t overspeed_pulses;
  };

  int16_t applied_pwm_counts_ = 0;
  bool speed_valid_ = false;
  bool new_pulse_ = false;
  uint32_t speed_age_us_ = 0xFFFFFFFFUL;
  float ff_pwm_ = 0.0F;
  float feedback_pwm_ = 0.0F;
  bool braking_ = true;
  bool speed_brake_ = false;
  uint8_t absolute_overspeed_pulses_ = 0U;
  int8_t last_drive_direction_ = 0;
  uint32_t reversal_remaining_us_ = 0UL;
  bool feedback_initialized_;
  uint8_t feedback_sample_count_;
  uint32_t previous_left_pulse_count_;
  uint32_t previous_right_pulse_count_;
  uint32_t left_position_bits_;
  uint32_t right_position_bits_;
  int8_t feedback_direction_;
  float filtered_left_mrad_s_;
  float filtered_right_mrad_s_;
  float applied_target_mrad_s_;
  float last_commanded_pwm_;
  float release_start_pwm_;
  bool release_pwm_fade_active_;
  PidState motor_pid_;
  HallMonitorState left_hall_monitor_;
  HallMonitorState right_hall_monitor_;
  uint8_t hall_fault_mask_;

  static float rampTarget(
      float current,
      float requested,
      float dt_seconds,
      uint32_t deceleration_mrad_s2);
  static float calculatePid(
      float target_mrad_s,
      float measured_mrad_s,
      float dt_seconds,
      PidState& state);
  static float compensateMotorDeadzone(
      float controller_pwm,
      float target_mrad_s);
  static float openLoopPwm(float target_mrad_s);
  void writeMotor(float pwm);
  static float hallSpeedMagnitude(
      const HallSample& sample,
      uint32_t pulse_delta,
      uint32_t elapsed_us);
  static bool updateHallMonitor(
      HallMonitorState& state,
      uint32_t pulse_count,
      int32_t target_mrad_s,
      int32_t measured_mrad_s,
      uint32_t elapsed_us,
      bool output_allowed,
      bool motor_output_active);
  void updateHallFeedback(
      uint32_t elapsed_us,
      const HallSample& left_hall,
      const HallSample& right_hall);
  void updateHallPlausibility(
      const HallSample& left_hall,
      const HallSample& right_hall,
      uint32_t elapsed_us,
      bool output_allowed);
};
