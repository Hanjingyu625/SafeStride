#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t TOF_ADDRESS = 0x52U;
constexpr uint8_t MPU_ADDRESS_LOW = 0x68U;
constexpr uint8_t MPU_ADDRESS_HIGH = 0x69U;
constexpr uint8_t MPU_WHO_AM_I = 0x75U;
constexpr uint8_t MPU_PWR_MGMT_1 = 0x6BU;
constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3BU;
constexpr uint16_t SAMPLE_PERIOD_MS = 100U;

uint8_t g_mpu_address = 0U;
uint32_t g_last_sample_ms = 0UL;

bool readRegisters(
    uint8_t address,
    uint8_t first_register,
    uint8_t* destination,
    uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(first_register);
  if (Wire.endTransmission() != 0U) {
    return false;
  }
  if (Wire.requestFrom(address, length) != length) {
    return false;
  }
  for (uint8_t index = 0U; index < length; ++index) {
    destination[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0U;
}

uint16_t readTof() {
  uint8_t bytes[2U];
  return readRegisters(TOF_ADDRESS, 0x00U, bytes, 2U)
      ? static_cast<uint16_t>(
            (static_cast<uint16_t>(bytes[0]) << 8U) | bytes[1])
      : 0xFFFFU;
}

int16_t readI16(const uint8_t* data) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(data[0]) << 8U) | data[1]);
}

void setup() {
  Serial.begin(115200UL);
  Wire.begin();
  delay(100U);
  for (uint8_t address = MPU_ADDRESS_LOW;
       address <= MPU_ADDRESS_HIGH;
       ++address) {
    uint8_t identity = 0U;
    if (readRegisters(address, MPU_WHO_AM_I, &identity, 1U) &&
        (identity & 0x7EU) == 0x68U) {
      g_mpu_address = address;
      writeRegister(address, MPU_PWR_MGMT_1, 0x01U);
      break;
    }
  }
  Serial.println(F("tof_mm,accel_x_raw,accel_y_raw,accel_z_raw,gyro_x_raw,gyro_y_raw,gyro_z_raw"));
}

void loop() {
  const uint32_t now_ms = millis();
  if (now_ms - g_last_sample_ms < SAMPLE_PERIOD_MS) {
    return;
  }
  g_last_sample_ms = now_ms;
  uint8_t sample[14U] = {0U};
  const bool mpu_ok = g_mpu_address != 0U &&
      readRegisters(g_mpu_address, MPU_ACCEL_XOUT_H, sample, sizeof(sample));
  Serial.print(readTof());
  for (uint8_t offset = 0U; offset <= 12U; offset += 2U) {
    if (offset == 6U) {
      continue;  // Temperature is not used by SafeStride.
    }
    Serial.print(',');
    if (mpu_ok) {
      Serial.print(readI16(sample + offset));
    } else {
      Serial.print(F("invalid"));
    }
  }
  Serial.println();
}
