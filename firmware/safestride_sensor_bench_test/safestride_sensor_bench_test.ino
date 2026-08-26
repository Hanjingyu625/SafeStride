#include <Arduino.h>
#include <avr/wdt.h>
#include <math.h>

#if !defined(ARDUINO_ARCH_AVR)
#error "This sensor bench is intended for an AVR Arduino Uno/Nano."
#endif

namespace {

// Mirrored from firmware/safestride_mcu/config.h. The repository sync test
// checks the safety-relevant values.
namespace cfg {
constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint8_t LEFT_HALL_PIN = 2U;
constexpr uint8_t RIGHT_HALL_PIN = 3U;
constexpr uint8_t HALL_ACTIVE_LEVEL = LOW;
constexpr uint32_t HALL_MIN_PULSE_INTERVAL_US = 500UL;
constexpr uint32_t HALL_ZERO_TIMEOUT_US = 1500000UL;
constexpr uint32_t HALL_PULSES_PER_WHEEL_REV = 1UL;
constexpr bool HALL_CALIBRATED = false;
// Matches the current ROS default. Replace with the measured wheel radius
// before treating the linear-speed fields as calibrated measurements.
constexpr float WHEEL_RADIUS_M = 0.15F;
constexpr uint8_t MOTOR_PWM_PIN = 5U;
constexpr uint8_t MOTOR_IN1_PIN = 6U;
constexpr uint8_t MOTOR_IN2_PIN = 8U;
constexpr uint8_t ESTOP_PIN = A2;
constexpr uint8_t ESTOP_ACTIVE_LEVEL = LOW;
constexpr uint8_t PRESSURE_LEFT_PIN = A0;
constexpr uint8_t PRESSURE_RIGHT_PIN = A1;
constexpr uint16_t PRESSURE_SAMPLE_PERIOD_MS = 100U;
constexpr float PRESSURE_FILTER_ALPHA = 0.2F;
constexpr bool PRESSURE_LEFT_ACTIVE_HIGH = true;
constexpr bool PRESSURE_RIGHT_ACTIVE_HIGH = true;
constexpr float PRESSURE_LEFT_PRESENT_THRESHOLD = 100.0F;
constexpr float PRESSURE_RIGHT_PRESENT_THRESHOLD = 100.0F;
constexpr bool PRESSURE_THRESHOLDS_CALIBRATED = false;
constexpr float PRESSURE_IMBALANCE_THRESHOLD = 300.0F;
constexpr float PRESSURE_SUDDEN_CHANGE_THRESHOLD = 150.0F;
constexpr bool USE_DRIVER_FAULT_PIN = false;
constexpr uint8_t DRIVER_FAULT_PIN = 13U;
constexpr uint8_t DRIVER_FAULT_ACTIVE_LEVEL = LOW;
}  // namespace cfg

enum class PressureAlert : uint8_t {
  NORMAL = 0U,
  WARNING = 1U,
  HANDS_OFF = 2U,
};

bool channelPresent(float value, bool active_high, float threshold) {
  return active_high ? value >= threshold : value <= threshold;
}

class BenchPressureSensorPair {
 public:
  BenchPressureSensorPair()
      : initialized_(false),
        last_sample_ms_(0UL),
        left_(0.0F),
        right_(0.0F),
        previous_left_(0.0F),
        previous_right_(0.0F),
        alert_(PressureAlert::HANDS_OFF) {}

  void begin(uint32_t now_ms) {
    left_ = static_cast<float>(analogRead(cfg::PRESSURE_LEFT_PIN));
    right_ = static_cast<float>(analogRead(cfg::PRESSURE_RIGHT_PIN));
    previous_left_ = left_;
    previous_right_ = right_;
    initialized_ = true;
    alert_ = bothHandsPresent()
        ? PressureAlert::NORMAL
        : PressureAlert::HANDS_OFF;
    last_sample_ms_ = now_ms;
  }

  void update(uint32_t now_ms) {
    if (now_ms - last_sample_ms_ < cfg::PRESSURE_SAMPLE_PERIOD_MS) {
      return;
    }
    last_sample_ms_ = now_ms;
    const float raw_left =
        static_cast<float>(analogRead(cfg::PRESSURE_LEFT_PIN));
    const float raw_right =
        static_cast<float>(analogRead(cfg::PRESSURE_RIGHT_PIN));
    left_ = cfg::PRESSURE_FILTER_ALPHA * raw_left +
        (1.0F - cfg::PRESSURE_FILTER_ALPHA) * left_;
    right_ = cfg::PRESSURE_FILTER_ALPHA * raw_right +
        (1.0F - cfg::PRESSURE_FILTER_ALPHA) * right_;
    const float maximum_delta = max(
        fabsf(left_ - previous_left_),
        fabsf(right_ - previous_right_));
    const float difference = fabsf(left_ - right_);
    if (!bothHandsPresent()) {
      alert_ = PressureAlert::HANDS_OFF;
    } else if (
        difference > cfg::PRESSURE_IMBALANCE_THRESHOLD ||
        maximum_delta > cfg::PRESSURE_SUDDEN_CHANGE_THRESHOLD) {
      alert_ = PressureAlert::WARNING;
    } else {
      alert_ = PressureAlert::NORMAL;
    }
    previous_left_ = left_;
    previous_right_ = right_;
  }

  bool leftPresent() const {
    return initialized_ && channelPresent(
        left_,
        cfg::PRESSURE_LEFT_ACTIVE_HIGH,
        cfg::PRESSURE_LEFT_PRESENT_THRESHOLD);
  }

  bool rightPresent() const {
    return initialized_ && channelPresent(
        right_,
        cfg::PRESSURE_RIGHT_ACTIVE_HIGH,
        cfg::PRESSURE_RIGHT_PRESENT_THRESHOLD);
  }

  bool bothHandsPresent() const {
    return leftPresent() && rightPresent();
  }

  float leftFiltered() const { return left_; }
  float rightFiltered() const { return right_; }
  PressureAlert alert() const { return alert_; }

 private:
  bool initialized_;
  uint32_t last_sample_ms_;
  float left_;
  float right_;
  float previous_left_;
  float previous_right_;
  PressureAlert alert_;
};

struct HallReading {
  uint32_t pulses;
  uint32_t period_us;
  uint32_t age_us;
};

constexpr uint16_t STREAM_PERIOD_MS = 100U;
constexpr size_t COMMAND_BUFFER_SIZE = 40U;

volatile uint32_t g_left_hall_pulses = 0UL;
volatile uint32_t g_right_hall_pulses = 0UL;
volatile uint32_t g_left_hall_last_us = 0UL;
volatile uint32_t g_right_hall_last_us = 0UL;
volatile uint32_t g_left_hall_period_us = 0UL;
volatile uint32_t g_right_hall_period_us = 0UL;
BenchPressureSensorPair g_pressure;

char g_command_buffer[COMMAND_BUFFER_SIZE];
size_t g_command_length = 0U;
bool g_stream_enabled = true;
bool g_hall_interrupts_ok = false;
uint32_t g_last_stream_ms = 0UL;

void holdMotorOutputSafe() {
  analogWrite(cfg::MOTOR_PWM_PIN, 0);
  digitalWrite(cfg::MOTOR_IN1_PIN, LOW);
  digitalWrite(cfg::MOTOR_IN2_PIN, LOW);
}

void preloadOutputLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void recordHallPulse(
    volatile uint32_t& pulses,
    volatile uint32_t& last_us,
    volatile uint32_t& period_us) {
  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - last_us;
  if (last_us != 0UL &&
      elapsed_us < cfg::HALL_MIN_PULSE_INTERVAL_US) {
    return;
  }
  if (last_us != 0UL) {
    period_us = elapsed_us;
  }
  last_us = now_us;
  ++pulses;
}

void leftHallIsr() {
  recordHallPulse(
      g_left_hall_pulses,
      g_left_hall_last_us,
      g_left_hall_period_us);
}

void rightHallIsr() {
  recordHallPulse(
      g_right_hall_pulses,
      g_right_hall_last_us,
      g_right_hall_period_us);
}

void readHall(uint32_t now_us, HallReading& left, HallReading& right) {
  uint32_t left_last_us = 0UL;
  uint32_t right_last_us = 0UL;
  noInterrupts();
  left.pulses = g_left_hall_pulses;
  right.pulses = g_right_hall_pulses;
  left.period_us = g_left_hall_period_us;
  right.period_us = g_right_hall_period_us;
  left_last_us = g_left_hall_last_us;
  right_last_us = g_right_hall_last_us;
  interrupts();
  left.age_us = left_last_us == 0UL
      ? 0xFFFFFFFFUL
      : now_us - left_last_us;
  right.age_us = right_last_us == 0UL
      ? 0xFFFFFFFFUL
      : now_us - right_last_us;
}

void zeroHall() {
  noInterrupts();
  g_left_hall_pulses = 0UL;
  g_right_hall_pulses = 0UL;
  g_left_hall_last_us = 0UL;
  g_right_hall_last_us = 0UL;
  g_left_hall_period_us = 0UL;
  g_right_hall_period_us = 0UL;
  interrupts();
}

float hallFrequencyHz(const HallReading& reading) {
  if (reading.period_us == 0UL ||
      reading.age_us >= cfg::HALL_ZERO_TIMEOUT_US) {
    return 0.0F;
  }
  return 1000000.0F / static_cast<float>(reading.period_us);
}

float hallRpm(const HallReading& reading) {
  return hallFrequencyHz(reading) * 60.0F /
      static_cast<float>(cfg::HALL_PULSES_PER_WHEEL_REV);
}

float hallLinearSpeedMps(const HallReading& reading) {
  const float revolutions_per_second = hallFrequencyHz(reading) /
      static_cast<float>(cfg::HALL_PULSES_PER_WHEEL_REV);
  return revolutions_per_second * 2.0F * static_cast<float>(PI) *
      cfg::WHEEL_RADIUS_M;
}

bool hallStopped(const HallReading& reading) {
  return reading.period_us == 0UL ||
      reading.age_us >= cfg::HALL_ZERO_TIMEOUT_US;
}

const __FlashStringHelper* pressureAlertText(PressureAlert alert) {
  switch (alert) {
    case PressureAlert::NORMAL:
      return F("NORMAL");
    case PressureAlert::WARNING:
      return F("WARNING");
    case PressureAlert::HANDS_OFF:
    default:
      return F("HANDS_OFF");
  }
}

void printStatus() {
  HallReading left = {0UL, 0UL, 0xFFFFFFFFUL};
  HallReading right = {0UL, 0UL, 0xFFFFFFFFUL};
  readHall(micros(), left, right);
  const int left_raw = analogRead(cfg::PRESSURE_LEFT_PIN);
  const int right_raw = analogRead(cfg::PRESSURE_RIGHT_PIN);
  const bool estop =
      digitalRead(cfg::ESTOP_PIN) == cfg::ESTOP_ACTIVE_LEVEL;
  const bool driver_fault = cfg::USE_DRIVER_FAULT_PIN &&
      digitalRead(cfg::DRIVER_FAULT_PIN) ==
          cfg::DRIVER_FAULT_ACTIVE_LEVEL;

  Serial.print(F("DRIVE_SENSOR ms="));
  Serial.print(millis());
  Serial.print(F(" estop="));
  Serial.print(estop ? 1 : 0);
  Serial.print(F(" deadman="));
  Serial.print(g_pressure.bothHandsPresent() ? 1 : 0);
  Serial.print(F(" pressure_calibrated="));
  Serial.print(cfg::PRESSURE_THRESHOLDS_CALIBRATED ? 1 : 0);
  Serial.print(F(" pressure_l_raw="));
  Serial.print(left_raw);
  Serial.print(F(" pressure_l_filtered="));
  Serial.print(g_pressure.leftFiltered(), 1);
  Serial.print(F(" pressure_l_present="));
  Serial.print(g_pressure.leftPresent() ? 1 : 0);
  Serial.print(F(" pressure_r_raw="));
  Serial.print(right_raw);
  Serial.print(F(" pressure_r_filtered="));
  Serial.print(g_pressure.rightFiltered(), 1);
  Serial.print(F(" pressure_r_present="));
  Serial.print(g_pressure.rightPresent() ? 1 : 0);
  Serial.print(F(" pressure_alert="));
  Serial.print(pressureAlertText(g_pressure.alert()));
  Serial.print(F(" hall_l_pulses="));
  Serial.print(left.pulses);
  Serial.print(F(" hall_l_period_us="));
  Serial.print(left.period_us);
  Serial.print(F(" hall_l_age_us="));
  Serial.print(left.age_us);
  Serial.print(F(" hall_l_hz="));
  Serial.print(hallFrequencyHz(left), 3);
  Serial.print(F(" hall_l_rpm="));
  Serial.print(hallRpm(left), 2);
  Serial.print(F(" hall_l_speed_mps="));
  Serial.print(hallLinearSpeedMps(left), 3);
  Serial.print(F(" hall_l_speed_kmh="));
  Serial.print(hallLinearSpeedMps(left) * 3.6F, 3);
  Serial.print(F(" hall_l_stopped="));
  Serial.print(hallStopped(left) ? 1 : 0);
  Serial.print(F(" hall_r_pulses="));
  Serial.print(right.pulses);
  Serial.print(F(" hall_r_period_us="));
  Serial.print(right.period_us);
  Serial.print(F(" hall_r_age_us="));
  Serial.print(right.age_us);
  Serial.print(F(" hall_r_hz="));
  Serial.print(hallFrequencyHz(right), 3);
  Serial.print(F(" hall_r_rpm="));
  Serial.print(hallRpm(right), 2);
  Serial.print(F(" hall_r_speed_mps="));
  Serial.print(hallLinearSpeedMps(right), 3);
  Serial.print(F(" hall_r_speed_kmh="));
  Serial.print(hallLinearSpeedMps(right) * 3.6F, 3);
  Serial.print(F(" hall_r_stopped="));
  Serial.print(hallStopped(right) ? 1 : 0);
  Serial.print(F(" hall_irq_ok="));
  Serial.print(g_hall_interrupts_ok ? 1 : 0);
  Serial.print(F(" hall_calibrated="));
  Serial.print(cfg::HALL_CALIBRATED ? 1 : 0);
  Serial.print(F(" hall_pulses_per_rev="));
  Serial.print(cfg::HALL_PULSES_PER_WHEEL_REV);
  Serial.print(F(" driver_fault="));
  Serial.println(driver_fault ? 1 : 0);
}

void printHelp() {
  Serial.println(F("SafeStride sensor bench (motor output always disabled)"));
  Serial.println(F("Pins: Hall L=D2 R=D3, FSR=A0/A1, E-stop=A2"));
  Serial.println(F("Commands: STATUS, ZERO, STREAM ON, STREAM OFF, HELP"));
  Serial.println(F("ZERO clears Hall pulse counters for PPR calibration."));
  Serial.print(F("Wheel radius for speed fields (m): "));
  Serial.println(cfg::WHEEL_RADIUS_M, 3);
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
  if (strcmp(command, "ZERO") == 0 || strcmp(command, "zero") == 0) {
    zeroHall();
    Serial.println(F("HALL COUNTERS ZEROED"));
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

  preloadOutputLow(cfg::MOTOR_PWM_PIN);
  preloadOutputLow(cfg::MOTOR_IN1_PIN);
  preloadOutputLow(cfg::MOTOR_IN2_PIN);
  holdMotorOutputSafe();

  pinMode(cfg::ESTOP_PIN, INPUT_PULLUP);
  if (cfg::USE_DRIVER_FAULT_PIN) {
    pinMode(cfg::DRIVER_FAULT_PIN, INPUT_PULLUP);
  }
  pinMode(cfg::LEFT_HALL_PIN, INPUT_PULLUP);
  pinMode(cfg::RIGHT_HALL_PIN, INPUT_PULLUP);

  const int left_interrupt = digitalPinToInterrupt(cfg::LEFT_HALL_PIN);
  const int right_interrupt = digitalPinToInterrupt(cfg::RIGHT_HALL_PIN);
  g_hall_interrupts_ok =
      left_interrupt != NOT_AN_INTERRUPT &&
      right_interrupt != NOT_AN_INTERRUPT;
  if (left_interrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(
        left_interrupt,
        leftHallIsr,
        cfg::HALL_ACTIVE_LEVEL == LOW ? FALLING : RISING);
  }
  if (right_interrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(
        right_interrupt,
        rightHallIsr,
        cfg::HALL_ACTIVE_LEVEL == LOW ? FALLING : RISING);
  }

  Serial.begin(cfg::SERIAL_BAUD);
  g_pressure.begin(millis());
  printHelp();
  printStatus();
  g_last_stream_ms = millis();
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  holdMotorOutputSafe();
  const uint32_t now_ms = millis();
  g_pressure.update(now_ms);
  readSerialCommands();
  if (g_stream_enabled && now_ms - g_last_stream_ms >= STREAM_PERIOD_MS) {
    g_last_stream_ms = now_ms;
    printStatus();
  }
}
