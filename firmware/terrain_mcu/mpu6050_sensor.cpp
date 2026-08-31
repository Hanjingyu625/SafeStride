#include "mpu6050_sensor.h"

#include <math.h>
#include <Wire.h>

#include "config.h"

namespace cfg = safestride_terrain_config;

namespace {

constexpr uint8_t REG_SMPLRT_DIV = 0x19U;
constexpr uint8_t REG_CONFIG = 0x1AU;
constexpr uint8_t REG_GYRO_CONFIG = 0x1BU;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1CU;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3BU;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6BU;
constexpr uint8_t REG_WHO_AM_I = 0x75U;
constexpr uint8_t WHO_AM_I_MPU6050 = 0x68U;

int16_t readBigEndianI16(const uint8_t* bytes) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(bytes[0]) << 8U) |
      static_cast<uint16_t>(bytes[1]));
}

int16_t roundedI16(float value) {
  if (value >= 32767.0F) {
    return 32767;
  }
  if (value <= -32768.0F) {
    return static_cast<int16_t>(-32768);
  }
  return static_cast<int16_t>(value >= 0.0F ? value + 0.5F : value - 0.5F);
}

}  // namespace

Mpu6050Sensor::Mpu6050Sensor()
    : configured_(false),
      valid_(false),
      attitude_initialized_(false),
      address_(0U),
      consecutive_errors_(0U),
      last_sample_ms_(0UL),
      last_reconnect_ms_(0UL),
      accel_x_mg_(0),
      accel_y_mg_(0),
      accel_z_mg_(0),
      gyro_x_mrad_s_(0),
      gyro_y_mrad_s_(0),
      gyro_z_mrad_s_(0),
      roll_mrad_(0.0F),
      pitch_mrad_(0.0F) {}

void Mpu6050Sensor::begin(uint32_t now_ms) {
  configured_ = false;
  valid_ = false;
  attitude_initialized_ = false;
  consecutive_errors_ = 0U;
  last_sample_ms_ = now_ms - cfg::MPU6050_SAMPLE_PERIOD_MS;
  last_reconnect_ms_ = now_ms - cfg::MPU6050_RECONNECT_PERIOD_MS;
  if (cfg::ENABLE_MPU6050) {
    configured_ = configure();
    last_reconnect_ms_ = now_ms;
  }
}

void Mpu6050Sensor::update(uint32_t now_ms) {
  if (!cfg::ENABLE_MPU6050) {
    valid_ = false;
    return;
  }
  if (!configured_) {
    valid_ = false;
    if (now_ms - last_reconnect_ms_ >=
        cfg::MPU6050_RECONNECT_PERIOD_MS) {
      last_reconnect_ms_ = now_ms;
      configured_ = configure();
    }
    return;
  }
  if (now_ms - last_sample_ms_ < cfg::MPU6050_SAMPLE_PERIOD_MS) {
    return;
  }
  last_sample_ms_ = now_ms;

  uint8_t sample[14U];
  if (!readRegisters(REG_ACCEL_XOUT_H, sample, sizeof(sample))) {
    noteReadFailure();
    return;
  }

  const int16_t raw_ax = readBigEndianI16(sample + 0U);
  const int16_t raw_ay = readBigEndianI16(sample + 2U);
  const int16_t raw_az = readBigEndianI16(sample + 4U);
  const int16_t raw_gx = readBigEndianI16(sample + 8U);
  const int16_t raw_gy = readBigEndianI16(sample + 10U);
  const int16_t raw_gz = readBigEndianI16(sample + 12U);

  // MPU6050 default ranges selected below: +/-2 g and +/-250 deg/s.
  accel_x_mg_ = roundedI16(static_cast<float>(raw_ax) / 16.384F);
  accel_y_mg_ = roundedI16(static_cast<float>(raw_ay) / 16.384F);
  accel_z_mg_ = roundedI16(static_cast<float>(raw_az) / 16.384F);
  constexpr float GYRO_RAW_TO_MRAD_S = 0.13323124F;
  gyro_x_mrad_s_ = roundedI16(raw_gx * GYRO_RAW_TO_MRAD_S);
  gyro_y_mrad_s_ = roundedI16(raw_gy * GYRO_RAW_TO_MRAD_S);
  gyro_z_mrad_s_ = roundedI16(raw_gz * GYRO_RAW_TO_MRAD_S);

  const float ax = static_cast<float>(raw_ax);
  const float ay = static_cast<float>(raw_ay);
  const float az = static_cast<float>(raw_az);
  const float roll = atan2f(ay, az) * 1000.0F;
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 1000.0F;
  if (!attitude_initialized_) {
    roll_mrad_ = roll;
    pitch_mrad_ = pitch;
    attitude_initialized_ = true;
  } else {
    roll_mrad_ += cfg::MPU6050_ATTITUDE_ALPHA * (roll - roll_mrad_);
    pitch_mrad_ += cfg::MPU6050_ATTITUDE_ALPHA * (pitch - pitch_mrad_);
  }

  valid_ = true;
  consecutive_errors_ = 0U;
}

bool Mpu6050Sensor::configure() {
  valid_ = false;
  attitude_initialized_ = false;
  address_ = 0U;
  if (probe(cfg::MPU6050_ADDRESS_LOW)) {
    address_ = cfg::MPU6050_ADDRESS_LOW;
  } else if (probe(cfg::MPU6050_ADDRESS_HIGH)) {
    address_ = cfg::MPU6050_ADDRESS_HIGH;
  } else {
    return false;
  }
  if (!writeRegister(REG_PWR_MGMT_1, 0x01U) ||
      !writeRegister(
          REG_SMPLRT_DIV, cfg::MPU6050_SAMPLE_RATE_DIVIDER) ||
      !writeRegister(REG_CONFIG, 0x03U) ||
      !writeRegister(REG_GYRO_CONFIG, 0x00U) ||
      !writeRegister(REG_ACCEL_CONFIG, 0x00U)) {
    address_ = 0U;
    return false;
  }
  consecutive_errors_ = 0U;
  return true;
}

bool Mpu6050Sensor::probe(uint8_t address) {
  address_ = address;
  uint8_t identity = 0U;
  return readRegisters(REG_WHO_AM_I, &identity, 1U) &&
         (identity & 0x7EU) == WHO_AM_I_MPU6050;
}

bool Mpu6050Sensor::writeRegister(uint8_t register_address, uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(register_address);
  Wire.write(value);
  return Wire.endTransmission() == 0U;
}

bool Mpu6050Sensor::readRegisters(
    uint8_t register_address,
    uint8_t* destination,
    uint8_t length) {
  Wire.beginTransmission(address_);
  Wire.write(register_address);
  if (Wire.endTransmission() != 0U) {
    return false;
  }
  delayMicroseconds(50U);
  if (Wire.requestFrom(address_, length) != length ||
      Wire.available() < length) {
    return false;
  }
  for (uint8_t index = 0U; index < length; ++index) {
    const int value = Wire.read();
    if (value < 0) {
      return false;
    }
    destination[index] = static_cast<uint8_t>(value);
  }
  return true;
}

void Mpu6050Sensor::noteReadFailure() {
  valid_ = false;
  if (consecutive_errors_ < 0xFFU) {
    ++consecutive_errors_;
  }
  if (consecutive_errors_ >= cfg::MPU6050_MAX_CONSECUTIVE_ERRORS) {
    configured_ = false;
  }
}

bool Mpu6050Sensor::valid() const { return valid_; }
uint8_t Mpu6050Sensor::address() const { return address_; }
int16_t Mpu6050Sensor::accelXMg() const { return accel_x_mg_; }
int16_t Mpu6050Sensor::accelYMg() const { return accel_y_mg_; }
int16_t Mpu6050Sensor::accelZMg() const { return accel_z_mg_; }
int16_t Mpu6050Sensor::gyroXMradS() const { return gyro_x_mrad_s_; }
int16_t Mpu6050Sensor::gyroYMradS() const { return gyro_y_mrad_s_; }
int16_t Mpu6050Sensor::gyroZMradS() const { return gyro_z_mrad_s_; }
int16_t Mpu6050Sensor::rollMrad() const { return roundedI16(roll_mrad_); }
int16_t Mpu6050Sensor::pitchMrad() const { return roundedI16(pitch_mrad_); }
