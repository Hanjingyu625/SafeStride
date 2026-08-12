#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
#include <EEPROM.h>
#include <avr/wdt.h>
#endif

#include "config.h"
#include "controller_state.h"
#include "motor_control.h"
#include "protocol.h"

#if !defined(ARDUINO_ARCH_AVR) && !defined(SAFESTRIDE_HOST_BUILD)
#error "Non-AVR port needs a persistent boot ID and hardware watchdog."
#endif

namespace cfg = safestride_config;
namespace proto = safestride_protocol;

// Status bits shared with PROTOCOL.md and the ROS bridge.
constexpr uint16_t STATUS_SESSION_ACTIVE = 1U << 0U;
constexpr uint16_t STATUS_MOTOR_ENABLED = 1U << 1U;
constexpr uint16_t STATUS_DEADMAN_ACTIVE = 1U << 2U;
constexpr uint16_t STATUS_ESTOP_ACTIVE = 1U << 3U;
constexpr uint16_t STATUS_WATCHDOG_TIMEOUT = 1U << 4U;
constexpr uint16_t STATUS_VALID_COMMAND_SEEN = 1U << 5U;
constexpr uint8_t STATUS_STATE_SHIFT = 8U;

// Fault values match safestride_interfaces/msg/WalkerStatus.msg.
constexpr uint16_t FAULT_LEFT_MOTOR = 1U << 1U;
constexpr uint16_t FAULT_RIGHT_MOTOR = 1U << 2U;
constexpr uint16_t FAULT_LEFT_ENCODER = 1U << 3U;
constexpr uint16_t FAULT_RIGHT_ENCODER = 1U << 4U;

constexpr uint32_t CAP_TWO_ENCODERS = 1UL << 0U;
constexpr uint32_t CAP_TWO_RANGES = 1UL << 1U;
constexpr uint32_t CAP_BATTERY = 1UL << 2U;
constexpr uint32_t CAP_TWO_CURRENTS = 1UL << 3U;
constexpr uint32_t CAP_DEADMAN = 1UL << 4U;
constexpr uint32_t CAP_ESTOP = 1UL << 5U;

volatile uint32_t g_left_encoder_count = 0UL;
volatile uint32_t g_right_encoder_count = 0UL;

proto::FrameReceiver g_receiver;
DriveController g_drive;

ControllerState g_state = ControllerState::BOOT;
uint16_t g_fault_bits = 0U;
bool g_watchdog_timed_out = false;
bool g_valid_command_seen = false;
bool g_session_active = false;
bool g_session_offer_active = false;
uint32_t g_boot_id = 0UL;
uint32_t g_session_id = 0UL;
uint16_t g_tx_sequence = 0U;
uint16_t g_last_command_sequence = 0U;
bool g_have_command_sequence = false;
uint16_t g_active_command_ttl_ms = cfg::COMMAND_WATCHDOG_MAX_MS;
uint32_t g_last_valid_command_ms = 0UL;
uint32_t g_last_session_activity_ms = 0UL;
bool g_stationary_tracking = false;
uint32_t g_stationary_since_ms = 0UL;

int32_t g_left_requested_mrad_s = 0L;
int32_t g_right_requested_mrad_s = 0L;

uint32_t g_last_control_us = 0UL;
uint32_t g_last_telemetry_ms = 0UL;
uint32_t g_last_hello_ms = 0UL;

uint32_t elapsedMs(uint32_t now, uint32_t then) {
  return now - then;
}

bool estopActive() {
  return digitalRead(cfg::ESTOP_PIN) == cfg::ESTOP_ACTIVE_LEVEL;
}

bool deadmanActive() {
  if (!cfg::REQUIRE_DEADMAN) {
    return true;
  }
  return digitalRead(cfg::DEADMAN_PIN) == cfg::DEADMAN_ACTIVE_LEVEL;
}

bool driverFaultActive() {
  return cfg::USE_DRIVER_FAULT_PIN &&
         digitalRead(cfg::DRIVER_FAULT_PIN) ==
             cfg::DRIVER_FAULT_ACTIVE_LEVEL;
}

void leftEncoderIsr() {
  // A fires on both edges. Comparing A with B keeps the inferred direction
  // constant across A's rising and falling edges for a quadrature encoder.
  const bool positive =
      digitalRead(cfg::LEFT_ENCODER_A_PIN) ==
      digitalRead(cfg::LEFT_ENCODER_B_PIN);
  const int8_t direction =
      (positive ? 1 : -1) * cfg::LEFT_ENCODER_SIGN;
  if (direction > 0) {
    ++g_left_encoder_count;
  } else {
    --g_left_encoder_count;
  }
}

void rightEncoderIsr() {
  const bool positive =
      digitalRead(cfg::RIGHT_ENCODER_A_PIN) ==
      digitalRead(cfg::RIGHT_ENCODER_B_PIN);
  const int8_t direction =
      (positive ? 1 : -1) * cfg::RIGHT_ENCODER_SIGN;
  if (direction > 0) {
    ++g_right_encoder_count;
  } else {
    --g_right_encoder_count;
  }
}

void readEncoderCounts(uint32_t& left, uint32_t& right) {
  noInterrupts();
  left = g_left_encoder_count;
  right = g_right_encoder_count;
  interrupts();
}

uint32_t makeBootId() {
  uint32_t value = 0UL;
#if defined(ARDUINO_ARCH_AVR)
  uint32_t boot_counter = 0UL;
  EEPROM.get(cfg::AVR_BOOT_COUNTER_EEPROM_ADDRESS, boot_counter);
  if (boot_counter == 0UL || boot_counter == 0xFFFFFFFFUL) {
    boot_counter = 1UL;
  } else {
    ++boot_counter;
    if (boot_counter == 0UL || boot_counter == 0xFFFFFFFFUL) {
      boot_counter = 1UL;
    }
  }
  EEPROM.put(cfg::AVR_BOOT_COUNTER_EEPROM_ADDRESS, boot_counter);
  // Multiplication by an odd constant and xorshift below are bijective over
  // uint32, so boot IDs do not repeat before the counter itself wraps.
  value = boot_counter * 0x9E3779B9UL;
#else
  // Host-only syntax tests never communicate with real motor hardware.
  value = 0x53465331UL;
#endif
  // xorshift32: uniqueness aid, not a cryptographic random source.
  value ^= value << 13U;
  value ^= value >> 17U;
  value ^= value << 5U;
  if (value == 0UL) {
    value = 0x53465331UL;  // "SFS1"
  }
  return value;
}

void immediateStop(ControllerState state, bool watchdog_timeout) {
  g_left_requested_mrad_s = 0L;
  g_right_requested_mrad_s = 0L;
  g_watchdog_timed_out = watchdog_timeout;
  g_state = state;
  g_drive.disableImmediately();
}

void clearSession() {
  immediateStop(
      estopActive() ? ControllerState::ESTOP : ControllerState::DISARMED,
      false);
  if (g_fault_bits != 0U) {
    g_state = ControllerState::FAULT;
  }
  g_session_active = false;
  g_session_offer_active = false;
  g_session_id = 0UL;
  g_valid_command_seen = false;
  g_have_command_sequence = false;
  g_last_hello_ms = millis() - cfg::HELLO_PERIOD_MS;
}

void invalidateSessionForWatchdog() {
  immediateStop(ControllerState::SAFE_STOP, true);
  g_session_active = false;
  g_session_offer_active = false;
  g_session_id = 0UL;
  g_valid_command_seen = false;
  g_have_command_sequence = false;
  g_last_hello_ms = millis() - cfg::HELLO_PERIOD_MS;
}

void refreshPhysicalSafety() {
  if (estopActive()) {
    if (g_state != ControllerState::ESTOP) {
      immediateStop(ControllerState::ESTOP, false);
    } else {
      g_drive.disableImmediately();
    }
    return;
  }

  if (driverFaultActive()) {
    g_fault_bits |= FAULT_LEFT_MOTOR | FAULT_RIGHT_MOTOR;
    immediateStop(ControllerState::FAULT, false);
    return;
  }

  if (g_state == ControllerState::ARMED && !deadmanActive()) {
    immediateStop(ControllerState::SAFE_STOP, false);
  }
}

uint16_t currentStatusBits() {
  uint16_t status = 0U;
  if (g_session_active) {
    status |= STATUS_SESSION_ACTIVE;
  }
  if (g_state == ControllerState::ARMED) {
    status |= STATUS_MOTOR_ENABLED;
  }
  if (deadmanActive()) {
    status |= STATUS_DEADMAN_ACTIVE;
  }
  if (estopActive()) {
    status |= STATUS_ESTOP_ACTIVE;
  }
  if (g_watchdog_timed_out) {
    status |= STATUS_WATCHDOG_TIMEOUT;
  }
  if (g_valid_command_seen) {
    status |= STATUS_VALID_COMMAND_SEEN;
  }
  status |= static_cast<uint16_t>(g_state) << STATUS_STATE_SHIFT;
  return status;
}

uint16_t readRangeLeftMm() {
  // TODO: Replace with a non-blocking real sensor driver.
  return 0xFFFFU;
}

uint16_t readRangeRightMm() {
  // TODO: Replace with a non-blocking real sensor driver.
  return 0xFFFFU;
}

uint16_t readBatteryMv() {
  if (!cfg::ENABLE_BATTERY_SENSE) {
    return 0xFFFFU;
  }
  const float pin_voltage =
      static_cast<float>(analogRead(cfg::BATTERY_SENSE_PIN)) *
      cfg::ADC_REFERENCE_V / 1023.0F;
  const float millivolts =
      pin_voltage * cfg::BATTERY_DIVIDER_RATIO * 1000.0F;
  if (millivolts <= 0.0F || millivolts >= 65535.0F) {
    return 0xFFFFU;
  }
  return static_cast<uint16_t>(millivolts + 0.5F);
}

int16_t readCurrentMa(uint8_t pin) {
  if (!cfg::ENABLE_CURRENT_SENSE) {
    return static_cast<int16_t>(-32767 - 1);
  }
  const float voltage =
      static_cast<float>(analogRead(pin)) *
      cfg::ADC_REFERENCE_V / 1023.0F;
  float current_ma =
      (voltage - cfg::CURRENT_ZERO_V) * cfg::CURRENT_MA_PER_V;
  if (current_ma > 32767.0F) {
    current_ma = 32767.0F;
  }
  if (current_ma < -32767.0F) {
    current_ma = -32767.0F;
  }
  return static_cast<int16_t>(
      current_ma >= 0.0F ? current_ma + 0.5F : current_ma - 0.5F);
}

void sendHello() {
  uint8_t payload[proto::HELLO_PAYLOAD_SIZE];
  uint32_t capabilities = CAP_TWO_ENCODERS | CAP_TWO_RANGES |
                          CAP_DEADMAN | CAP_ESTOP;
  if (cfg::ENABLE_BATTERY_SENSE) {
    capabilities |= CAP_BATTERY;
  }
  if (cfg::ENABLE_CURRENT_SENSE) {
    capabilities |= CAP_TWO_CURRENTS;
  }
  proto::writeU32(payload, g_boot_id);
  proto::writeU32(payload + 4U, capabilities);
  if (proto::sendFrame(
      Serial,
      proto::TYPE_HELLO,
      g_tx_sequence++,
      0UL,
      millis(),
      payload,
      sizeof(payload))) {
    g_session_offer_active = true;
  }
}

void sendTelemetry() {
  if (!g_session_active) {
    return;
  }
  uint32_t left_count = 0UL;
  uint32_t right_count = 0UL;
  readEncoderCounts(left_count, right_count);

  uint8_t payload[proto::TELEMETRY_PAYLOAD_SIZE];
  proto::writeU32(payload + 0U, left_count);
  proto::writeU32(payload + 4U, right_count);
  proto::writeI32(payload + 8U, g_drive.leftVelocityMradS());
  proto::writeI32(payload + 12U, g_drive.rightVelocityMradS());
  proto::writeU16(payload + 16U, readRangeLeftMm());
  proto::writeU16(payload + 18U, readRangeRightMm());
  proto::writeU16(payload + 20U, readBatteryMv());
  proto::writeI16(
      payload + 22U, readCurrentMa(cfg::LEFT_CURRENT_SENSE_PIN));
  proto::writeI16(
      payload + 24U, readCurrentMa(cfg::RIGHT_CURRENT_SENSE_PIN));
  proto::writeU16(payload + 26U, currentStatusBits());
  proto::writeU16(payload + 28U, g_fault_bits);
  proto::writeU16(payload + 30U, g_last_command_sequence);

  proto::sendFrame(
      Serial,
      proto::TYPE_TELEMETRY,
      g_tx_sequence++,
      g_session_id,
      millis(),
      payload,
      sizeof(payload));
}

bool stationaryDwellMet() {
  return g_stationary_tracking &&
         elapsedMs(millis(), g_stationary_since_ms) >=
             cfg::ARM_STATIONARY_DWELL_MS;
}

void markAcceptedCommand(
    const safestride_protocol::FrameView& frame,
    uint16_t ttl_ms) {
  g_last_command_sequence = frame.sequence;
  g_have_command_sequence = true;
  g_valid_command_seen = true;
  g_watchdog_timed_out = false;
  g_active_command_ttl_ms = ttl_ms;
  g_last_valid_command_ms = millis();
  g_last_session_activity_ms = g_last_valid_command_ms;
}

bool handleSessionStart(
    const safestride_protocol::FrameView& frame) {
  if (g_session_active || !g_session_offer_active ||
      frame.payload_length != proto::SESSION_START_PAYLOAD_SIZE ||
      frame.session_id == 0UL) {
    return false;
  }
  const uint32_t expected_boot_id = proto::readU32(frame.payload);
  if (expected_boot_id != g_boot_id) {
    return false;
  }

  const bool watchdog_latched = g_watchdog_timed_out;
  immediateStop(
      estopActive() ? ControllerState::ESTOP : ControllerState::DISARMED,
      watchdog_latched);
  if (g_fault_bits != 0U) {
    g_state = ControllerState::FAULT;
  }
  g_session_id = frame.session_id;
  g_session_active = true;
  g_session_offer_active = false;
  g_valid_command_seen = false;
  g_have_command_sequence = false;
  g_last_command_sequence = 0U;
  g_last_session_activity_ms = millis();
  g_last_telemetry_ms = millis() - cfg::TELEMETRY_PERIOD_MS;
  return true;
}

bool handleCommand(const safestride_protocol::FrameView& frame) {
  if (!g_session_active || frame.session_id != g_session_id ||
      frame.payload_length != proto::COMMAND_PAYLOAD_SIZE) {
    return false;
  }
  if (g_have_command_sequence &&
      !proto::sequenceIsNewer(
          frame.sequence, g_last_command_sequence)) {
    return false;
  }

  const int32_t left = proto::readI32(frame.payload + 0U);
  const int32_t right = proto::readI32(frame.payload + 4U);
  const uint16_t ttl_ms = proto::readU16(frame.payload + 8U);
  const uint8_t enable = frame.payload[10U];
  const uint8_t reserved = frame.payload[11U];
  if ((enable != 0U && enable != 1U) || reserved != 0U) {
    return false;
  }
  if (ttl_ms < cfg::COMMAND_TTL_MIN_MS ||
      ttl_ms > cfg::COMMAND_WATCHDOG_MAX_MS) {
    return false;
  }
  if (left < -cfg::MAX_WHEEL_TARGET_MRAD_S ||
      left > cfg::MAX_WHEEL_TARGET_MRAD_S ||
      right < -cfg::MAX_WHEEL_TARGET_MRAD_S ||
      right > cfg::MAX_WHEEL_TARGET_MRAD_S) {
    return false;
  }

  if (enable == 0U) {
    markAcceptedCommand(frame, ttl_ms);
    immediateStop(
        estopActive() ? ControllerState::ESTOP
                      : ControllerState::DISARMED,
        false);
    if (!driverFaultActive()) {
      // Only currently implemented motor-driver fault bits are resettable.
      g_fault_bits &= static_cast<uint16_t>(
          ~(FAULT_LEFT_MOTOR | FAULT_RIGHT_MOTOR));
    }
    if (g_fault_bits != 0U) {
      g_state = ControllerState::FAULT;
    }
    return true;
  }

  // An enable command is not accepted while a hardware interlock or a latched
  // stop state is active. Releasing the input alone never restarts motion; the
  // host must first send a disabled command and explicitly arm again.
  if (estopActive() || !deadmanActive() || g_watchdog_timed_out ||
      g_fault_bits != 0U ||
      g_state == ControllerState::ESTOP ||
      g_state == ControllerState::SAFE_STOP ||
      g_state == ControllerState::FAULT) {
    return false;
  }

  if (g_state == ControllerState::DISARMED) {
    if (!stationaryDwellMet()) {
      return false;
    }
    markAcceptedCommand(frame, ttl_ms);
    g_state = ControllerState::ARMED;
    g_left_requested_mrad_s = left;
    g_right_requested_mrad_s = right;
    return true;
  }

  if (g_state != ControllerState::ARMED) {
    return false;
  }
  markAcceptedCommand(frame, ttl_ms);
  g_left_requested_mrad_s = left;
  g_right_requested_mrad_s = right;
  return true;
}

void processSerial() {
  proto::FrameView frame = {
      0U, 0U, 0U, 0U, 0UL, 0UL, NULL};
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      break;
    }
    const proto::ReceiveResult result =
        g_receiver.push(static_cast<uint8_t>(incoming), frame);
    if (result != proto::ReceiveResult::FRAME_READY) {
      continue;
    }

    if (frame.type == proto::TYPE_SESSION_START) {
      handleSessionStart(frame);
    } else if (frame.type == proto::TYPE_COMMAND) {
      handleCommand(frame);
    }
  }
}

void enforceWatchdogs(uint32_t now_ms) {
  if (g_session_active &&
      elapsedMs(now_ms, g_last_session_activity_ms) >
          cfg::SESSION_LOSS_TIMEOUT_MS) {
    clearSession();
    return;
  }

  if (g_state == ControllerState::ARMED &&
      elapsedMs(now_ms, g_last_valid_command_ms) >
          g_active_command_ttl_ms) {
    invalidateSessionForWatchdog();
  }
}

void runControlLoop(uint32_t now_us) {
  const uint32_t elapsed_us = now_us - g_last_control_us;
  if (elapsed_us < cfg::CONTROL_PERIOD_US) {
    return;
  }
  g_last_control_us = now_us;

  uint32_t left_count = 0UL;
  uint32_t right_count = 0UL;
  readEncoderCounts(left_count, right_count);
  const bool output_allowed =
      g_session_active &&
      g_state == ControllerState::ARMED &&
      !estopActive() &&
      deadmanActive() &&
      g_fault_bits == 0U;
  g_drive.update(
      elapsed_us,
      left_count,
      right_count,
      g_left_requested_mrad_s,
      g_right_requested_mrad_s,
      output_allowed);
  const uint8_t encoder_faults = g_drive.encoderFaultMask();
  if (encoder_faults != 0U) {
    if ((encoder_faults & DriveController::ENCODER_FAULT_LEFT) != 0U) {
      g_fault_bits |= FAULT_LEFT_ENCODER;
    }
    if ((encoder_faults & DriveController::ENCODER_FAULT_RIGHT) != 0U) {
      g_fault_bits |= FAULT_RIGHT_ENCODER;
    }
    immediateStop(ControllerState::FAULT, false);
    g_stationary_tracking = false;
    return;
  }

  const int32_t left_speed = g_drive.leftVelocityMradS();
  const int32_t right_speed = g_drive.rightVelocityMradS();
  const bool stationary =
      g_drive.feedbackReady() &&
      left_speed >= -cfg::ARM_MAX_MEASURED_SPEED_MRAD_S &&
      left_speed <= cfg::ARM_MAX_MEASURED_SPEED_MRAD_S &&
      right_speed >= -cfg::ARM_MAX_MEASURED_SPEED_MRAD_S &&
      right_speed <= cfg::ARM_MAX_MEASURED_SPEED_MRAD_S;
  if (stationary) {
    if (!g_stationary_tracking) {
      g_stationary_tracking = true;
      g_stationary_since_ms = millis();
    }
  } else {
    g_stationary_tracking = false;
  }
}

void setup() {
#if defined(ARDUINO_ARCH_AVR)
  // Avoid a watchdog-reset loop before enabling the configured timeout below.
  MCUSR = 0U;
  wdt_disable();
#endif

  // Motor enable is driven to its configured inactive level first.
  g_drive.begin();

  pinMode(cfg::ESTOP_PIN, INPUT_PULLUP);
  pinMode(cfg::DEADMAN_PIN, INPUT_PULLUP);
  if (cfg::USE_DRIVER_FAULT_PIN) {
    pinMode(cfg::DRIVER_FAULT_PIN, INPUT_PULLUP);
  }
  pinMode(cfg::LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(cfg::LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(cfg::RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(cfg::RIGHT_ENCODER_B_PIN, INPUT_PULLUP);

  const int left_interrupt =
      digitalPinToInterrupt(cfg::LEFT_ENCODER_A_PIN);
  const int right_interrupt =
      digitalPinToInterrupt(cfg::RIGHT_ENCODER_A_PIN);
  if (left_interrupt == NOT_AN_INTERRUPT) {
    g_fault_bits |= FAULT_LEFT_ENCODER;
  } else {
    attachInterrupt(left_interrupt, leftEncoderIsr, CHANGE);
  }
  if (right_interrupt == NOT_AN_INTERRUPT) {
    g_fault_bits |= FAULT_RIGHT_ENCODER;
  } else {
    attachInterrupt(right_interrupt, rightEncoderIsr, CHANGE);
  }

  Serial.begin(cfg::SERIAL_BAUD);
  g_boot_id = makeBootId();
  g_state = estopActive()
      ? ControllerState::ESTOP
      : ControllerState::DISARMED;
  if (g_fault_bits != 0U) {
    g_state = ControllerState::FAULT;
  }
  g_last_control_us = micros();
  g_last_hello_ms = millis() - cfg::HELLO_PERIOD_MS;

#if defined(ARDUINO_ARCH_AVR)
  wdt_enable(WDTO_500MS);
#endif
}

void loop() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_reset();
#endif

  refreshPhysicalSafety();
  // Enforce an already-missed deadline before parsing buffered commands.
  // Otherwise a late but valid frame could overwrite the receive timestamp
  // and conceal a control-loop stall.
  uint32_t now_ms = millis();
  enforceWatchdogs(now_ms);
  processSerial();
  now_ms = millis();
  enforceWatchdogs(now_ms);
  runControlLoop(micros());

  if (!g_session_active &&
      elapsedMs(now_ms, g_last_hello_ms) >= cfg::HELLO_PERIOD_MS) {
    g_last_hello_ms = now_ms;
    sendHello();
  }
  if (g_session_active &&
      elapsedMs(now_ms, g_last_telemetry_ms) >=
          cfg::TELEMETRY_PERIOD_MS) {
    g_last_telemetry_ms = now_ms;
    sendTelemetry();
  }
}
