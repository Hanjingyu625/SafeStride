#pragma once

#include <Arduino.h>

class Mpu6050Sensor {
 public:
  Mpu6050Sensor();

  void begin(uint32_t now_ms);
  void update(uint32_t now_ms);

  bool valid() const;
  uint8_t address() const;
  int16_t accelXMg() const;
  int16_t accelYMg() const;
  int16_t accelZMg() const;
  int16_t gyroXMradS() const;
  int16_t gyroYMradS() const;
  int16_t gyroZMradS() const;
  int16_t rollMrad() const;
  int16_t pitchMrad() const;

 private:
  bool configured_;
  bool valid_;
  bool attitude_initialized_;
  uint8_t address_;
  uint8_t consecutive_errors_;
  uint32_t last_sample_ms_;
  uint32_t last_reconnect_ms_;
  int16_t accel_x_mg_;
  int16_t accel_y_mg_;
  int16_t accel_z_mg_;
  int16_t gyro_x_mrad_s_;
  int16_t gyro_y_mrad_s_;
  int16_t gyro_z_mrad_s_;
  float roll_mrad_;
  float pitch_mrad_;

  bool configure();
  bool probe(uint8_t address);
  bool writeRegister(uint8_t register_address, uint8_t value);
  bool readRegisters(
      uint8_t register_address,
      uint8_t* destination,
      uint8_t length);
  void noteReadFailure();
};
