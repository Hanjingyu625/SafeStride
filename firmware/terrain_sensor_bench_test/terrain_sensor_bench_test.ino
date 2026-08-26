#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>

#if !defined(ARDUINO_ARCH_AVR)
#error "This sensor bench is intended for an AVR Arduino Uno/Nano."
#endif

namespace {

// Keep these values synchronized with firmware/terrain_mcu/config.h. The
// repository sync test compares every TOF and LED constant.
namespace terrain_cfg {
constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint8_t TOF_I2C_ADDRESS = 0x52U;
constexpr uint8_t TOF_DISTANCE_REGISTER = 0x00U;
constexpr uint16_t TOF_SAMPLE_PERIOD_MS = 50U;
constexpr uint16_t TOF_MIN_VALID_DISTANCE_MM = 100U;
constexpr uint16_t TOF_MAX_VALID_DISTANCE_MM = 2000U;
constexpr float TOF_FILTER_ALPHA = 0.3F;
constexpr float TOF_REFERENCE_ALPHA = 0.02F;
constexpr float TOF_ERROR_THRESHOLD_MM = 60.0F;
constexpr float TOF_CHANGE_THRESHOLD_MM = 10.0F;
constexpr uint8_t TOF_REQUIRED_FRAMES = 4U;
constexpr uint16_t TOF_RED_HOLD_MS = 1000U;
}  // namespace terrain_cfg

enum class TofAlert : uint8_t {
  NORMAL = 0U,
  CANDIDATE = 1U,
  STEP = 2U,
  INVALID = 3U,
};

class BenchTof10120Sensor {
 public:
  BenchTof10120Sensor()
      : initialized_(false),
        valid_(false),
        red_hold_active_(false),
        consecutive_count_(0U),
        last_sample_ms_(0UL),
        last_red_ms_(0UL),
        distance_mm_(0xFFFFU),
        filtered_mm_(0.0F),
        reference_mm_(0.0F),
        error_mm_(0.0F),
        change_mm_(0.0F),
        alert_(TofAlert::INVALID) {}

  void begin(uint32_t now_ms) {
    last_sample_ms_ = now_ms - terrain_cfg::TOF_SAMPLE_PERIOD_MS;
  }

  void update(uint32_t now_ms) {
    if (now_ms - last_sample_ms_ < terrain_cfg::TOF_SAMPLE_PERIOD_MS) {
      return;
    }
    last_sample_ms_ = now_ms;
    const uint16_t distance = readDistanceI2c();
    if (distance < terrain_cfg::TOF_MIN_VALID_DISTANCE_MM ||
        distance > terrain_cfg::TOF_MAX_VALID_DISTANCE_MM) {
      valid_ = false;
      consecutive_count_ = 0U;
      alert_ = TofAlert::INVALID;
      return;
    }

    distance_mm_ = distance;
    valid_ = true;
    if (!initialized_) {
      filtered_mm_ = static_cast<float>(distance);
      reference_mm_ = filtered_mm_;
      error_mm_ = 0.0F;
      change_mm_ = 0.0F;
      initialized_ = true;
      alert_ = TofAlert::NORMAL;
      return;
    }

    const float previous_filtered = filtered_mm_;
    filtered_mm_ =
        terrain_cfg::TOF_FILTER_ALPHA * static_cast<float>(distance) +
        (1.0F - terrain_cfg::TOF_FILTER_ALPHA) * filtered_mm_;
    reference_mm_ = terrain_cfg::TOF_REFERENCE_ALPHA * filtered_mm_ +
        (1.0F - terrain_cfg::TOF_REFERENCE_ALPHA) * reference_mm_;
    error_mm_ = filtered_mm_ - reference_mm_;
    change_mm_ = filtered_mm_ - previous_filtered;

    if (error_mm_ > terrain_cfg::TOF_ERROR_THRESHOLD_MM &&
        change_mm_ > terrain_cfg::TOF_CHANGE_THRESHOLD_MM) {
      if (consecutive_count_ < terrain_cfg::TOF_REQUIRED_FRAMES) {
        ++consecutive_count_;
      }
    } else {
      consecutive_count_ = 0U;
    }
    if (consecutive_count_ >= terrain_cfg::TOF_REQUIRED_FRAMES) {
      red_hold_active_ = true;
      last_red_ms_ = now_ms;
    }
    if (red_hold_active_ &&
        now_ms - last_red_ms_ >= terrain_cfg::TOF_RED_HOLD_MS) {
      red_hold_active_ = false;
    }
    if (red_hold_active_) {
      alert_ = TofAlert::STEP;
    } else if (error_mm_ > terrain_cfg::TOF_ERROR_THRESHOLD_MM) {
      alert_ = TofAlert::CANDIDATE;
    } else {
      alert_ = TofAlert::NORMAL;
    }
  }

  bool valid() const { return valid_; }
  uint16_t distanceMm() const {
    return valid_ ? distance_mm_ : 0xFFFFU;
  }
  float filteredDistanceMm() const { return filtered_mm_; }
  float referenceDistanceMm() const { return reference_mm_; }
  float errorMm() const { return error_mm_; }
  float changeMm() const { return change_mm_; }
  TofAlert alert() const { return alert_; }

 private:
  bool initialized_;
  bool valid_;
  bool red_hold_active_;
  uint8_t consecutive_count_;
  uint32_t last_sample_ms_;
  uint32_t last_red_ms_;
  uint16_t distance_mm_;
  float filtered_mm_;
  float reference_mm_;
  float error_mm_;
  float change_mm_;
  TofAlert alert_;

  uint16_t readDistanceI2c() {
    Wire.beginTransmission(terrain_cfg::TOF_I2C_ADDRESS);
    Wire.write(terrain_cfg::TOF_DISTANCE_REGISTER);
    if (Wire.endTransmission() != 0U) {
      return 0xFFFFU;
    }
    delayMicroseconds(50U);
    if (Wire.requestFrom(
            terrain_cfg::TOF_I2C_ADDRESS,
            static_cast<uint8_t>(2U)) != 2U ||
        Wire.available() < 2) {
      return 0xFFFFU;
    }
    const uint8_t high_byte = static_cast<uint8_t>(Wire.read());
    const uint8_t low_byte = static_cast<uint8_t>(Wire.read());
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(high_byte) << 8U) | low_byte);
  }

};

constexpr uint16_t STREAM_PERIOD_MS = 200U;
constexpr size_t COMMAND_BUFFER_SIZE = 40U;

constexpr uint8_t MPU_ADDRESS_LOW = 0x68U;
constexpr uint8_t MPU_ADDRESS_HIGH = 0x69U;
constexpr uint8_t MPU_WHO_AM_I = 0x75U;
constexpr uint8_t MPU_PWR_MGMT_1 = 0x6BU;
constexpr uint8_t MPU_GYRO_CONFIG = 0x1BU;
constexpr uint8_t MPU_ACCEL_CONFIG = 0x1CU;
constexpr uint8_t MPU_INT_PIN_CFG = 0x37U;
constexpr uint8_t MPU_USER_CTRL = 0x6AU;
constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3BU;

constexpr uint8_t AK8963_ADDRESS = 0x0CU;
constexpr uint8_t AK8963_WIA = 0x00U;
constexpr uint8_t AK8963_ST1 = 0x02U;
constexpr uint8_t AK8963_CNTL1 = 0x0AU;
constexpr uint8_t AK8963_ASAX = 0x10U;

constexpr uint8_t BNO_ADDRESS_LOW = 0x28U;
constexpr uint8_t BNO_ADDRESS_HIGH = 0x29U;
constexpr uint8_t BNO_CHIP_ID = 0x00U;
constexpr uint8_t BNO_PAGE_ID = 0x07U;
constexpr uint8_t BNO_EULER_H_LSB = 0x1AU;
constexpr uint8_t BNO_CALIB_STAT = 0x35U;
constexpr uint8_t BNO_OPR_MODE = 0x3DU;
constexpr uint8_t BNO_PWR_MODE = 0x3EU;
constexpr uint8_t BNO_SYS_STATUS = 0x39U;
constexpr uint8_t BNO_SYS_ERR = 0x3AU;
constexpr uint8_t BNO_NDOF_MODE = 0x0CU;

BenchTof10120Sensor g_tof;
char g_command_buffer[COMMAND_BUFFER_SIZE];
size_t g_command_length = 0U;
bool g_stream_enabled = true;
uint32_t g_last_stream_ms = 0UL;

uint8_t g_mpu_address = 0U;
uint8_t g_mpu_id = 0xFFU;
bool g_mpu_configured = false;
bool g_ak8963_present = false;
float g_mag_adjustment[3] = {1.0F, 1.0F, 1.0F};

uint8_t g_bno_address = 0U;
uint8_t g_bno_id = 0xFFU;
bool g_bno_configured = false;

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0U;
}

bool readRegisters(
    uint8_t address,
    uint8_t start_register,
    uint8_t* data,
    uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(start_register);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }
  if (Wire.requestFrom(address, length) != length) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }
  for (uint8_t index = 0U; index < length; ++index) {
    if (Wire.available() <= 0) {
      return false;
    }
    data[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool readRegister(uint8_t address, uint8_t reg, uint8_t& value) {
  return readRegisters(address, reg, &value, 1U);
}

bool probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0U;
}

int16_t signedBigEndian(const uint8_t* bytes) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(bytes[0]) << 8U) | bytes[1]);
}

int16_t signedLittleEndian(const uint8_t* bytes) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(bytes[1]) << 8U) | bytes[0]);
}

void printHex8(uint8_t value) {
  if (value < 0x10U) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

const __FlashStringHelper* tofAlertText(TofAlert alert) {
  switch (alert) {
    case TofAlert::NORMAL:
      return F("NORMAL");
    case TofAlert::CANDIDATE:
      return F("CANDIDATE");
    case TofAlert::STEP:
      return F("STEP");
    case TofAlert::INVALID:
    default:
      return F("INVALID");
  }
}

void scanBus() {
  Serial.print(F("I2C_SCAN"));
  uint8_t found = 0U;
  for (uint8_t address = 1U; address < 0x7FU; ++address) {
    wdt_reset();
    if (probeAddress(address)) {
      Serial.print(F(" 0x"));
      printHex8(address);
      ++found;
    }
  }
  Serial.print(F(" count="));
  Serial.println(found);
}

bool configureMagnetometer() {
  uint8_t identity = 0xFFU;
  if (!readRegister(AK8963_ADDRESS, AK8963_WIA, identity) ||
      identity != 0x48U) {
    return false;
  }

  if (!writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x00U)) {
    return false;
  }
  delay(10);
  if (!writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x0FU)) {
    return false;
  }
  delay(10);

  uint8_t adjustment[3] = {128U, 128U, 128U};
  if (readRegisters(AK8963_ADDRESS, AK8963_ASAX, adjustment, 3U)) {
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
      g_mag_adjustment[axis] =
          (static_cast<float>(adjustment[axis]) - 128.0F) /
              256.0F +
          1.0F;
    }
  }

  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x00U);
  delay(10);
  // 16-bit output, continuous measurement mode 2 (100 Hz).
  return writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x16U);
}

void detectAndConfigureMpu() {
  g_mpu_address = 0U;
  g_mpu_id = 0xFFU;
  g_mpu_configured = false;
  g_ak8963_present = false;
  g_mag_adjustment[0] = 1.0F;
  g_mag_adjustment[1] = 1.0F;
  g_mag_adjustment[2] = 1.0F;

  for (uint8_t address = MPU_ADDRESS_LOW;
       address <= MPU_ADDRESS_HIGH;
       ++address) {
    uint8_t identity = 0xFFU;
    if (readRegister(address, MPU_WHO_AM_I, identity)) {
      g_mpu_address = address;
      g_mpu_id = identity;
      break;
    }
  }
  if (g_mpu_address == 0U ||
      (g_mpu_id != 0x71U && g_mpu_id != 0x73U)) {
    return;
  }

  // Wake the device and force the production-intended default scales:
  // accelerometer +/-2 g and gyroscope +/-250 dps.
  const bool configured =
      writeRegister(g_mpu_address, MPU_PWR_MGMT_1, 0x01U) &&
      writeRegister(g_mpu_address, MPU_GYRO_CONFIG, 0x00U) &&
      writeRegister(g_mpu_address, MPU_ACCEL_CONFIG, 0x00U) &&
      writeRegister(g_mpu_address, MPU_USER_CTRL, 0x00U) &&
      writeRegister(g_mpu_address, MPU_INT_PIN_CFG, 0x02U);
  if (!configured) {
    return;
  }
  delay(100);
  g_mpu_configured = true;
  g_ak8963_present = configureMagnetometer();
}

void detectAndConfigureBno() {
  g_bno_address = 0U;
  g_bno_id = 0xFFU;
  g_bno_configured = false;

  for (uint8_t address = BNO_ADDRESS_LOW;
       address <= BNO_ADDRESS_HIGH;
       ++address) {
    uint8_t identity = 0xFFU;
    if (readRegister(address, BNO_CHIP_ID, identity)) {
      g_bno_address = address;
      g_bno_id = identity;
      break;
    }
  }
  if (g_bno_address == 0U || g_bno_id != 0xA0U) {
    return;
  }

  if (!writeRegister(g_bno_address, BNO_OPR_MODE, 0x00U)) {
    return;
  }
  delay(25);
  if (!writeRegister(g_bno_address, BNO_PAGE_ID, 0x00U) ||
      !writeRegister(g_bno_address, BNO_PWR_MODE, 0x00U)) {
    return;
  }
  delay(10);
  if (!writeRegister(g_bno_address, BNO_OPR_MODE, BNO_NDOF_MODE)) {
    return;
  }
  delay(30);
  g_bno_configured = true;
}

void detectAndConfigureSensors() {
  detectAndConfigureMpu();
  detectAndConfigureBno();
}

void printMpuStatus() {
  Serial.print(F(" mpu_addr="));
  if (g_mpu_address == 0U) {
    Serial.print(F("missing"));
    return;
  }
  Serial.print(F("0x"));
  printHex8(g_mpu_address);
  Serial.print(F(" mpu_id=0x"));
  printHex8(g_mpu_id);
  Serial.print(F(" mpu_ok="));
  Serial.print(g_mpu_configured ? 1 : 0);
  if (!g_mpu_configured) {
    return;
  }

  uint8_t sample[14];
  if (!readRegisters(
          g_mpu_address, MPU_ACCEL_XOUT_H, sample, sizeof(sample))) {
    Serial.print(F(" mpu_read=ERROR"));
    return;
  }
  const int16_t ax = signedBigEndian(sample + 0U);
  const int16_t ay = signedBigEndian(sample + 2U);
  const int16_t az = signedBigEndian(sample + 4U);
  const int16_t gx = signedBigEndian(sample + 8U);
  const int16_t gy = signedBigEndian(sample + 10U);
  const int16_t gz = signedBigEndian(sample + 12U);

  Serial.print(F(" ax_g="));
  Serial.print(static_cast<float>(ax) / 16384.0F, 3);
  Serial.print(F(" ay_g="));
  Serial.print(static_cast<float>(ay) / 16384.0F, 3);
  Serial.print(F(" az_g="));
  Serial.print(static_cast<float>(az) / 16384.0F, 3);
  Serial.print(F(" gx_dps="));
  Serial.print(static_cast<float>(gx) / 131.0F, 2);
  Serial.print(F(" gy_dps="));
  Serial.print(static_cast<float>(gy) / 131.0F, 2);
  Serial.print(F(" gz_dps="));
  Serial.print(static_cast<float>(gz) / 131.0F, 2);

  Serial.print(F(" ak8963="));
  Serial.print(g_ak8963_present ? 1 : 0);
  if (!g_ak8963_present) {
    return;
  }
  uint8_t magnetometer[8];
  if (!readRegisters(
          AK8963_ADDRESS, AK8963_ST1, magnetometer,
          sizeof(magnetometer)) ||
      (magnetometer[0] & 0x01U) == 0U ||
      (magnetometer[7] & 0x08U) != 0U) {
    Serial.print(F(" mag_ready=0"));
    return;
  }
  const int16_t mx = signedLittleEndian(magnetometer + 1U);
  const int16_t my = signedLittleEndian(magnetometer + 3U);
  const int16_t mz = signedLittleEndian(magnetometer + 5U);
  Serial.print(F(" mx_uT="));
  Serial.print(static_cast<float>(mx) * 0.15F * g_mag_adjustment[0], 1);
  Serial.print(F(" my_uT="));
  Serial.print(static_cast<float>(my) * 0.15F * g_mag_adjustment[1], 1);
  Serial.print(F(" mz_uT="));
  Serial.print(static_cast<float>(mz) * 0.15F * g_mag_adjustment[2], 1);
}

void printBnoStatus() {
  Serial.print(F(" bno_addr="));
  if (g_bno_address == 0U) {
    Serial.print(F("missing"));
    return;
  }
  Serial.print(F("0x"));
  printHex8(g_bno_address);
  Serial.print(F(" bno_id=0x"));
  printHex8(g_bno_id);
  Serial.print(F(" bno_ok="));
  Serial.print(g_bno_configured ? 1 : 0);
  if (!g_bno_configured) {
    return;
  }

  uint8_t euler[6];
  uint8_t calibration = 0U;
  uint8_t system_status = 0U;
  uint8_t system_error = 0U;
  if (!readRegisters(g_bno_address, BNO_EULER_H_LSB, euler, 6U) ||
      !readRegister(g_bno_address, BNO_CALIB_STAT, calibration) ||
      !readRegister(g_bno_address, BNO_SYS_STATUS, system_status) ||
      !readRegister(g_bno_address, BNO_SYS_ERR, system_error)) {
    Serial.print(F(" bno_read=ERROR"));
    return;
  }

  Serial.print(F(" heading_deg="));
  Serial.print(static_cast<float>(signedLittleEndian(euler + 0U)) / 16.0F, 2);
  Serial.print(F(" roll_deg="));
  Serial.print(static_cast<float>(signedLittleEndian(euler + 2U)) / 16.0F, 2);
  Serial.print(F(" pitch_deg="));
  Serial.print(static_cast<float>(signedLittleEndian(euler + 4U)) / 16.0F, 2);
  Serial.print(F(" cal_sys="));
  Serial.print((calibration >> 6U) & 0x03U);
  Serial.print(F(" cal_gyr="));
  Serial.print((calibration >> 4U) & 0x03U);
  Serial.print(F(" cal_acc="));
  Serial.print((calibration >> 2U) & 0x03U);
  Serial.print(F(" cal_mag="));
  Serial.print(calibration & 0x03U);
  Serial.print(F(" sys_status="));
  Serial.print(system_status);
  Serial.print(F(" sys_error="));
  Serial.print(system_error);
}

void printStatus() {
  Serial.print(F("TERRAIN_SENSOR ms="));
  Serial.print(millis());
  Serial.print(F(" tof_addr=0x"));
  printHex8(terrain_cfg::TOF_I2C_ADDRESS);
  Serial.print(F(" tof_valid="));
  Serial.print(g_tof.valid() ? 1 : 0);
  Serial.print(F(" tof_raw_mm="));
  Serial.print(g_tof.distanceMm());
  Serial.print(F(" tof_filtered_mm="));
  Serial.print(g_tof.filteredDistanceMm(), 1);
  Serial.print(F(" tof_reference_mm="));
  Serial.print(g_tof.referenceDistanceMm(), 1);
  Serial.print(F(" tof_error_mm="));
  Serial.print(g_tof.errorMm(), 1);
  Serial.print(F(" tof_change_mm="));
  Serial.print(g_tof.changeMm(), 1);
  Serial.print(F(" tof_alert="));
  Serial.print(tofAlertText(g_tof.alert()));
  printMpuStatus();
  printBnoStatus();
  Serial.println();
}

void printHelp() {
  Serial.println(F("SafeStride Terrain sensor bench (no actuator commands)"));
  Serial.println(F("I2C A4=SDA A5=SCL: TOF=0x52, MPU=0x68/69, BNO=0x28/29"));
  Serial.println(F("Commands: STATUS, SCAN, REINIT, STREAM ON, STREAM OFF, HELP"));
  Serial.println(F("TOF filters/thresholds/LEDs reuse production code."));
  Serial.println(F("Leg output and unassigned limit-switch pins remain unused."));
}

void handleCommand(char* line) {
  char* command = strtok(line, " \t");
  if (command == nullptr) {
    return;
  }
  if (strcmp(command, "STATUS") == 0 || strcmp(command, "status") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "SCAN") == 0 || strcmp(command, "scan") == 0) {
    scanBus();
    return;
  }
  if (strcmp(command, "REINIT") == 0 || strcmp(command, "reinit") == 0) {
    detectAndConfigureSensors();
    Serial.println(F("SENSORS REINITIALIZED"));
    printStatus();
    return;
  }
  if (strcmp(command, "HELP") == 0 || strcmp(command, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(command, "STREAM") == 0 || strcmp(command, "stream") == 0) {
    const char* state = strtok(nullptr, " \t");
    const char* extra = strtok(nullptr, " \t");
    if (state != nullptr && extra == nullptr &&
        (strcmp(state, "ON") == 0 || strcmp(state, "on") == 0)) {
      g_stream_enabled = true;
      Serial.println(F("STREAM ON"));
      return;
    }
    if (state != nullptr && extra == nullptr &&
        (strcmp(state, "OFF") == 0 || strcmp(state, "off") == 0)) {
      g_stream_enabled = false;
      Serial.println(F("STREAM OFF"));
      return;
    }
  }
  Serial.println(F("ERROR unknown command"));
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      return;
    }
    const char value = static_cast<char>(incoming);
    if (value == '\r') {
      continue;
    }
    if (value == '\n') {
      g_command_buffer[g_command_length] = '\0';
      handleCommand(g_command_buffer);
      g_command_length = 0U;
      continue;
    }
    if (value < 0x20 || value > 0x7E) {
      g_command_length = 0U;
      continue;
    }
    if (g_command_length + 1U >= COMMAND_BUFFER_SIZE) {
      g_command_length = 0U;
      Serial.println(F("ERROR command too long"));
      continue;
    }
    g_command_buffer[g_command_length++] = value;
  }
}

}  // namespace

void setup() {
  MCUSR = 0U;
  wdt_disable();

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000UL, true);
#endif
  Serial.begin(terrain_cfg::SERIAL_BAUD);

  // BNO055 can require about 650 ms after power-on before CHIP_ID is valid.
  delay(700);
  g_tof.begin(millis());
  scanBus();
  detectAndConfigureSensors();
  printHelp();
  printStatus();
  g_last_stream_ms = millis();
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  const uint32_t now_ms = millis();
  g_tof.update(now_ms);
  readSerialCommands();
  if (g_stream_enabled && now_ms - g_last_stream_ms >= STREAM_PERIOD_MS) {
    g_last_stream_ms = now_ms;
    printStatus();
  }
}
