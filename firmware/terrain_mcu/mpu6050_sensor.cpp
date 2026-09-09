// MPU6050 I2C 측정과 자세 추정. 가속도는 mg(중력가속도의 1/1000),
// 자이로는 mrad/s, roll/pitch는 mrad로 보낸다. 1000 mrad = 1 rad이다.
// 현재 자세는 가속도 기반이며 자이로 적분/센서 융합은 구현하지 않았다.

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
constexpr float PI_MRAD = static_cast<float>(PI) * 1000.0F;

// MPU 레지스터는 상위 바이트 먼저(big-endian) 온다. Pi 통신의 little-endian과 구분한다.
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

// +π와 -π 경계에서도 짧은 회전 방향의 차이를 구해 필터가 0도를 지나 크게 튀지 않게 한다.
float shortestAngleDeltaMrad(float target, float current) {
  float delta = target - current;
  const float full_turn = 2.0F * PI_MRAD;
  while (delta > PI_MRAD) {
    delta -= full_turn;
  }
  while (delta < -PI_MRAD) {
    delta += full_turn;
  }
  return delta;
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

// 설정 실패 시 정해진 간격으로 재연결하고, 설정 성공 후 50ms마다 데이터를 읽는다.
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

  // 가속도 6바이트 + 온도 2바이트 + 자이로 6바이트를 연속으로 읽는다. 온도는 사용하지 않는다.
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
  // Board/body convention: +X forward, +Y left, +Z up. Acceleration due to
  // gravity therefore makes nose-up pitch positive through the -X component.
  // 가속도 방향을 중력 방향으로 가정해 각도를 구한다.
  // roll=atan2(ay,az), pitch=atan2(-ax,sqrt(ay²+az²)); rad에 1000을 곱해 mrad로 저장한다.
  // 가감속/충격도 가속도에 섞이므로 실제 경사와 일시적으로 다를 수 있다.
  const float roll = atan2f(ay, az) * 1000.0F;
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 1000.0F;
  if (!attitude_initialized_) {
    roll_mrad_ = roll;
    pitch_mrad_ = pitch;
    attitude_initialized_ = true;
  } else {
    // Roll crosses from +pi to -pi when the board is close to upside down.
    // Filter the shortest angular difference instead of jumping through zero.
    roll_mrad_ += cfg::MPU6050_ATTITUDE_ALPHA *
        shortestAngleDeltaMrad(roll, roll_mrad_);
    if (roll_mrad_ > PI_MRAD) {
      roll_mrad_ -= 2.0F * PI_MRAD;
    } else if (roll_mrad_ < -PI_MRAD) {
      roll_mrad_ += 2.0F * PI_MRAD;
    }
    // pitch 필터: 추정값 += 0.15 × (새 각도 - 추정값). 자이로 데이터는 이 계산에 넣지 않는다.
    pitch_mrad_ += cfg::MPU6050_ATTITUDE_ALPHA * (pitch - pitch_mrad_);
  }

  valid_ = true;
  consecutive_errors_ = 0U;
}

// 0x68/0x69를 탐색하고 측정 범위와 출력 주기를 설정한다. 재설정하면 자세 필터도 새로 시작한다.
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

// 한 번만 읽기에 실패해도 valid=false. 연속 실패가 기준에 도달하면 재설정 경로로 돌아간다.
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
