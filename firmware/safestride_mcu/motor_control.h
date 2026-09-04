#pragma once

#include <Arduino.h>

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
  void update(
      uint32_t elapsed_us,
      const HallSample& left_hall,
      const HallSample& right_hall,
      int32_t requested_mrad_s,
      bool output_allowed,
      bool enforce_hall_faults = true,
      uint32_t deceleration_mrad_s2 = 0UL,
      bool fade_pwm_during_deceleration = false);
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
    uint32_t overspeed_us;
  };

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
