#pragma once

#include <Arduino.h>

class Bno055Sensor {
 public:
  Bno055Sensor();

  void begin(uint32_t now_ms);
  void update(uint32_t now_ms);

  bool valid() const;
  uint8_t address() const;
  int16_t headingMrad() const;
  int16_t rollMrad() const;
  int16_t pitchMrad() const;
  uint8_t calibration() const;

 private:
  bool configured_;
  bool valid_;
  uint8_t address_;
  uint8_t consecutive_errors_;
  uint32_t last_sample_ms_;
  uint32_t last_configure_attempt_ms_;
  int16_t heading_mrad_;
  int16_t roll_mrad_;
  int16_t pitch_mrad_;
  uint8_t calibration_;

  bool configure(uint32_t now_ms);
  bool probe(uint8_t address);
  bool writeRegister(uint8_t register_address, uint8_t value);
  bool readRegisters(
      uint8_t register_address,
      uint8_t* destination,
      uint8_t length);
  void noteReadFailure();
};
