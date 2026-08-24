#pragma once

#include <Arduino.h>

// Hardware-neutral contract between the future encoder driver and control
// loop. Values describe the wheel output shaft, after encoder/gear conversion.
struct WheelEncoderSample {
  int32_t left_position_mrad;
  int32_t right_position_mrad;
  int32_t left_velocity_mrad_s;
  int32_t right_velocity_mrad_s;
  bool valid;
};

class EncoderFeedback {
 public:
  EncoderFeedback();

  // Returns true only when a hardware-specific driver is ready to sample.
  bool begin();
  WheelEncoderSample sample(uint32_t now_us);
  bool available() const;

 private:
  bool available_;
};
