#include "bno055_sensor.h"

#include <Wire.h>

#include "config.h"

namespace {

namespace cfg = safestride_terrain_config;

constexpr uint8_t REG_CHIP_ID = 0x00U;
constexpr uint8_t REG_PAGE_ID = 0x07U;
constexpr uint8_t REG_EULER_H_LSB = 0x1AU;
constexpr uint8_t REG_UNIT_SEL = 0x3BU;
constexpr uint8_t REG_OPR_MODE = 0x3DU;
constexpr uint8_t REG_PWR_MODE = 0x3EU;
constexpr uint8_t REG_CALIB_STAT = 0x35U;
constexpr uint8_t CHIP_ID = 0xA0U;
constexpr uint8_t MODE_CONFIG = 0x00U;
constexpr uint8_t MODE_NDOF = 0x0CU;
constexpr uint8_t POWER_NORMAL = 0x00U;
constexpr float MRAD_PER_EULER_LSB = 1.0908308F;

int16_t signedLittleEndian(const uint8_t* bytes) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(bytes[0U]) |
      (static_cast<uint16_t>(bytes[1U]) << 8U));
}

int16_t toMrad(int16_t raw) {
  const float value = static_cast<float>(raw) * MRAD_PER_EULER_LSB;
  if (value >= 32767.0F) {
    return 32767;
  }
  if (value <= -32768.0F) {
    return static_cast<int16_t>(-32768);
  }
  return static_cast<int16_t>(
      value >= 0.0F ? value + 0.5F : value - 0.5F);
}

}  // namespace

Bno055Sensor::Bno055Sensor()
    : configured_(false),
      valid_(false),
      address_(0U),
      consecutive_errors_(0U),
      last_sample_ms_(0UL),
      last_configure_attempt_ms_(0UL),
      heading_mrad_(0),
      roll_mrad_(0),
      pitch_mrad_(0),
      calibration_(0U) {}

void Bno055Sensor::begin(uint32_t now_ms) {
  last_sample_ms_ = now_ms - cfg::BNO055_SAMPLE_PERIOD_MS;
  last_configure_attempt_ms_ =
      now_ms - cfg::BNO055_RECONNECT_PERIOD_MS;
  if (cfg::ENABLE_BNO055) {
    configure(now_ms);
  }
}

void Bno055Sensor::update(uint32_t now_ms) {
  if (!cfg::ENABLE_BNO055) {
    valid_ = false;
    return;
  }
  if (!configured_) {
    if (now_ms - last_configure_attempt_ms_ >=
        cfg::BNO055_RECONNECT_PERIOD_MS) {
      configure(now_ms);
    }
    return;
  }
  if (now_ms - last_sample_ms_ < cfg::BNO055_SAMPLE_PERIOD_MS) {
    return;
  }
  last_sample_ms_ = now_ms;

  uint8_t euler[6U];
  uint8_t calibration = 0U;
  if (!readRegisters(REG_EULER_H_LSB, euler, sizeof(euler)) ||
      !readRegisters(REG_CALIB_STAT, &calibration, 1U)) {
    noteReadFailure();
    return;
  }

  heading_mrad_ = toMrad(signedLittleEndian(euler + 0U));
  roll_mrad_ = toMrad(signedLittleEndian(euler + 2U));
  pitch_mrad_ = toMrad(signedLittleEndian(euler + 4U));
  calibration_ = calibration;
  consecutive_errors_ = 0U;
  valid_ = true;
}

bool Bno055Sensor::configure(uint32_t now_ms) {
  last_configure_attempt_ms_ = now_ms;
  valid_ = false;
  configured_ = false;
  address_ = 0U;
  calibration_ = 0U;

  if (probe(cfg::BNO055_ADDRESS_LOW)) {
    address_ = cfg::BNO055_ADDRESS_LOW;
  } else if (probe(cfg::BNO055_ADDRESS_HIGH)) {
    address_ = cfg::BNO055_ADDRESS_HIGH;
  } else {
    return false;
  }

  if (!writeRegister(REG_OPR_MODE, MODE_CONFIG)) {
    address_ = 0U;
    return false;
  }
  delayMicroseconds(20000U);
  if (!writeRegister(REG_PAGE_ID, 0U) ||
      !writeRegister(REG_PWR_MODE, POWER_NORMAL) ||
      !writeRegister(REG_UNIT_SEL, 0U)) {
    address_ = 0U;
    return false;
  }
  delayMicroseconds(10000U);
  if (!writeRegister(REG_OPR_MODE, MODE_NDOF)) {
    address_ = 0U;
    return false;
  }
  delayMicroseconds(30000U);
  configured_ = true;
  consecutive_errors_ = 0U;
  return true;
}

bool Bno055Sensor::probe(uint8_t address) {
  address_ = address;
  uint8_t chip_id = 0U;
  return readRegisters(REG_CHIP_ID, &chip_id, 1U) && chip_id == CHIP_ID;
}

bool Bno055Sensor::writeRegister(
    uint8_t register_address,
    uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(register_address);
  Wire.write(value);
  return Wire.endTransmission() == 0U;
}

bool Bno055Sensor::readRegisters(
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

void Bno055Sensor::noteReadFailure() {
  valid_ = false;
  calibration_ = 0U;
  if (consecutive_errors_ < 0xFFU) {
    ++consecutive_errors_;
  }
  if (consecutive_errors_ >= cfg::BNO055_MAX_CONSECUTIVE_ERRORS) {
    configured_ = false;
  }
}

bool Bno055Sensor::valid() const { return valid_; }
uint8_t Bno055Sensor::address() const { return address_; }
int16_t Bno055Sensor::headingMrad() const { return heading_mrad_; }
int16_t Bno055Sensor::rollMrad() const { return roll_mrad_; }
int16_t Bno055Sensor::pitchMrad() const { return pitch_mrad_; }
uint8_t Bno055Sensor::calibration() const { return calibration_; }
