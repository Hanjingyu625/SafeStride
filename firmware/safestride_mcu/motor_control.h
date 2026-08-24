#pragma once

#include <Arduino.h>

#include "encoder_feedback.h"

class DriveController {
 public:
  static const uint8_t ENCODER_FAULT_LEFT = 1U << 0U;
  static const uint8_t ENCODER_FAULT_RIGHT = 1U << 1U;

  DriveController();
  void begin();
  void update(
      uint32_t elapsed_us,
      const WheelEncoderSample& encoder,
      int32_t requested_mrad_s,
      bool output_allowed);
  void disableImmediately();

  int32_t leftVelocityMradS() const;
  int32_t rightVelocityMradS() const;
  int32_t appliedTargetMradS() const;
  int32_t leftPositionMrad() const;
  int32_t rightPositionMrad() const;
  bool feedbackReady() const;
  uint8_t encoderFaultMask() const;

 private:
  struct PidState {
    float integral;
    float previous_error;
  };
  struct EncoderMonitorState {
    bool initialized;
    int32_t previous_position_mrad;
    uint32_t no_motion_us;
    uint32_t overspeed_us;
  };

  bool feedback_ready_;
  int32_t left_position_mrad_;
  int32_t right_position_mrad_;
  int32_t left_velocity_mrad_s_;
  int32_t right_velocity_mrad_s_;
  float applied_target_mrad_s_;
  PidState motor_pid_;
  EncoderMonitorState left_encoder_monitor_;
  EncoderMonitorState right_encoder_monitor_;
  uint8_t encoder_fault_mask_;

  static float rampTarget(float current, float requested, float dt_seconds);
  static float calculatePid(
      float target_mrad_s,
      float measured_mrad_s,
      float dt_seconds,
      PidState& state);
  static float compensateMotorDeadzone(
      float controller_pwm,
      float target_mrad_s);
  static float openLoopPwm(float target_mrad_s);
  static void writeMotor(float pwm);
  static bool updateEncoderMonitor(
      EncoderMonitorState& state,
      int32_t position_mrad,
      int32_t target_mrad_s,
      int32_t measured_mrad_s,
      uint32_t elapsed_us,
      bool output_allowed);
  void updateEncoderFeedback(const WheelEncoderSample& encoder);
  void updateEncoderPlausibility(
      const WheelEncoderSample& encoder,
      uint32_t elapsed_us,
      bool output_allowed);
};
