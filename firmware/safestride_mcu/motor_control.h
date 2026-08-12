#pragma once

#include <Arduino.h>

class DriveController {
 public:
  static const uint8_t ENCODER_FAULT_LEFT = 1U << 0U;
  static const uint8_t ENCODER_FAULT_RIGHT = 1U << 1U;

  DriveController();

  void begin();
  void update(
      uint32_t elapsed_us,
      uint32_t left_encoder_count,
      uint32_t right_encoder_count,
      int32_t left_requested_mrad_s,
      int32_t right_requested_mrad_s,
      bool output_allowed);
  void disableImmediately();

  int32_t leftVelocityMradS() const;
  int32_t rightVelocityMradS() const;
  int32_t leftAppliedTargetMradS() const;
  int32_t rightAppliedTargetMradS() const;
  bool feedbackReady() const;
  uint8_t encoderFaultMask() const;

 private:
  struct PidState {
    float integral;
    float previous_error;
  };

  struct EncoderMonitorState {
    bool initialized;
    uint32_t previous_count;
    uint32_t no_edge_us;
    uint32_t reverse_us;
    uint32_t overspeed_us;
  };

  bool feedback_initialized_;
  uint8_t feedback_sample_count_;
  uint32_t previous_left_count_;
  uint32_t previous_right_count_;
  float filtered_left_mrad_s_;
  float filtered_right_mrad_s_;
  float applied_left_mrad_s_;
  float applied_right_mrad_s_;
  PidState left_pid_;
  PidState right_pid_;
  EncoderMonitorState left_encoder_monitor_;
  EncoderMonitorState right_encoder_monitor_;
  uint8_t encoder_fault_mask_;

  static float rampTarget(
      float current,
      float requested,
      float dt_seconds);
  static float calculatePid(
      float target_mrad_s,
      float measured_mrad_s,
      float dt_seconds,
      float kp,
      float ki,
      float kd,
      float feedforward,
      PidState& state);
  static void writeMotor(
      uint8_t pwm_pin,
      uint8_t in1_pin,
      uint8_t in2_pin,
      int8_t motor_sign,
      float pwm);
  static bool updateEncoderMonitor(
      EncoderMonitorState& state,
      uint32_t encoder_count,
      int32_t target_mrad_s,
      int32_t measured_mrad_s,
      uint32_t elapsed_us,
      bool output_allowed);
  void updateEncoderPlausibility(
      uint32_t left_encoder_count,
      uint32_t right_encoder_count,
      uint32_t elapsed_us,
      bool output_allowed);
};
