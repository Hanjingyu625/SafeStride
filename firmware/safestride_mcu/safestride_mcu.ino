// Drive Uno 진입점: Pi 명령 수신 → 양손/통신/고장 조건 확인 → 모터 제어 → 상태 송신.
// 처음 읽을 때 setup(), loop(), handleCommand(), runControlLoop() 순서로 확인한다.
// 목표속도와 경사 보정은 Pi가 정하고, 실제 PWM 계산과 즉시 정지는 이 보드가 수행한다.

#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
#include <EEPROM.h>
#include <avr/wdt.h>
#endif

#include "config.h"
#include "controller_state.h"
#include "analog_hall_sensor.h"
#include "motor_control.h"
#include "pressure_sensor.h"
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
constexpr uint16_t STATUS_HALL_CALIBRATED = 1U << 6U;
constexpr uint16_t STATUS_MAGNET_BENCH_MODE = 1U << 7U;
constexpr uint8_t STATUS_STATE_SHIFT = 8U;

// Fault values match safestride_interfaces/msg/WalkerStatus.msg.
constexpr uint16_t FAULT_MOTOR_DRIVER = 1U << 1U;
constexpr uint16_t FAULT_LEFT_HALL = 1U << 3U;
constexpr uint32_t CAP_SINGLE_LEFT_HALL = 1UL << 0U;
constexpr uint32_t CAP_TWO_RANGES = 1UL << 1U;
constexpr uint32_t CAP_BATTERY = 1UL << 2U;
constexpr uint32_t CAP_TWO_CURRENTS = 1UL << 3U;
constexpr uint32_t CAP_DEADMAN = 1UL << 4U;
constexpr uint32_t CAP_ESTOP = 1UL << 5U;
constexpr uint32_t CAP_PRESSURE_TELEMETRY = 1UL << 6U;
constexpr uint32_t CAP_MAGNET_BENCH_MODE = 1UL << 7U;

constexpr uint8_t PRESSURE_FLAG_LEFT_PRESENT = 1U << 0U;
constexpr uint8_t PRESSURE_FLAG_RIGHT_PRESENT = 1U << 1U;
constexpr uint8_t PRESSURE_FLAG_CALIBRATED = 1U << 2U;

proto::FrameReceiver g_receiver;
DriveController g_drive;
PressureSensorPair g_pressure;
AnalogHallSensor g_hall;

ControllerState g_state = ControllerState::BOOT;
uint16_t g_fault_bits = 0U;
bool g_watchdog_timed_out = false;
int16_t g_slope_ff_pwm = 0;
uint8_t g_drive_pwm_cap = cfg::MAX_PWM;
bool g_brake_requested = false;
bool g_new_pulse_since_telemetry = false;
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

int32_t g_requested_mrad_s = 0L;
bool g_deadman_release_ramp_active = false;
uint32_t g_deadman_release_decel_mrad_s2 =
    cfg::MAX_DECEL_MRAD_S2;

uint32_t g_last_control_us = 0UL;
uint32_t g_last_telemetry_ms = 0UL;
uint32_t g_last_hello_ms = 0UL;

// unsigned 뺄셈으로 millis() 카운터가 한 바퀴 돌아도 짧은 경과시간을 계산한다.
uint32_t elapsedMs(uint32_t now, uint32_t then) {
  return now - then;
}

bool estopActive() {
  if (!cfg::ENABLE_ESTOP) {
    return false;
  }
  return digitalRead(cfg::ESTOP_PIN) == cfg::ESTOP_ACTIVE_LEVEL;
}

bool deadmanActive() {
  return g_pressure.bothHandsPresent();
}

bool motionDeadmanSatisfied() {
  return cfg::MAGNET_BENCH_MODE || !cfg::REQUIRE_DEADMAN ||
         deadmanActive();
}

bool driverFaultActive() {
  return cfg::USE_DRIVER_FAULT_PIN &&
         digitalRead(cfg::DRIVER_FAULT_PIN) ==
             cfg::DRIVER_FAULT_ACTIVE_LEVEL;
}

// 실제 Hall은 왼쪽 하나뿐이다. 오른쪽 필드는 호환성을 위한 복제값이므로 독립 측정이 아니다.
void readHallSamples(
    uint32_t now_us,
    HallSample& left,
    HallSample& right) {
  left.pulse_count = g_hall.pulseCount();
  left.period_us = g_hall.periodUs();
  left.age_us = g_hall.ageUs(now_us);
  // The shared drive has one physical Hall input. Keep the legacy two-wheel
  // telemetry layout by mirroring the left measurement into the right field.
  right = left;
}

// EEPROM 부팅 카운터로 재부팅 전후를 구분한다. 이전 부팅에 대한 세션 요청을 거부하기 위한 식별자다.
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

// 목표와 손 해제 ramp를 지우고 즉시 BRAKE 출력으로 전환한다. 정상 PWM slew를 기다리지 않는다.
void immediateStop(ControllerState state, bool watchdog_timeout) {
  g_requested_mrad_s = 0L;
  g_deadman_release_ramp_active = false;
  g_deadman_release_decel_mrad_s2 = cfg::MAX_DECEL_MRAD_S2;
  g_watchdog_timed_out = watchdog_timeout;
  g_state = state;
  g_drive.disableImmediately();
}

// 현재 적용 목표를 600ms 내 0으로 내릴 감속률을 계산한다. 모터 모듈은 직전 실제 PWM도 함께 줄인다.
void beginDeadmanReleaseRamp() {
  const int32_t applied_target = g_drive.appliedTargetMradS();
  const uint32_t magnitude = static_cast<uint32_t>(
      applied_target < 0L ? -applied_target : applied_target);
  const uint32_t numerator =
      magnitude * 1000UL + cfg::DEADMAN_RELEASE_RAMP_MS - 1UL;
  g_deadman_release_decel_mrad_s2 =
      numerator / cfg::DEADMAN_RELEASE_RAMP_MS;
  if (g_deadman_release_decel_mrad_s2 == 0UL) {
    g_deadman_release_decel_mrad_s2 = 1UL;
  }
  g_requested_mrad_s = 0L;
  g_deadman_release_ramp_active = true;
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

// 물리 입력 우선 처리: E-stop/드라이버 고장은 즉시 정지, 정상 손 해제는 별도 감속 경로로 보낸다.
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
    g_fault_bits |= FAULT_MOTOR_DRIVER;
    immediateStop(ControllerState::FAULT, false);
    return;
  }

  if (g_state == ControllerState::ARMED) {
    if (!motionDeadmanSatisfied() && !g_deadman_release_ramp_active) {
      beginDeadmanReleaseRamp();
    } else if (motionDeadmanSatisfied() &&
               g_deadman_release_ramp_active) {
      // Dead-man is level-triggered. If a confirmed release is followed by a
      // new grasp before the local ramp finishes, accept the next fresh ROS
      // command instead of rejecting it until the command watchdog expires.
      g_deadman_release_ramp_active = false;
      g_deadman_release_decel_mrad_s2 = cfg::MAX_DECEL_MRAD_S2;
    }
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
  if (cfg::HALL_CALIBRATED) {
    status |= STATUS_HALL_CALIBRATED;
  }
  if (cfg::MAGNET_BENCH_MODE) {
    status |= STATUS_MAGNET_BENCH_MODE;
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

// 보드 역할/기능/부팅 ID/규약 버전을 알려 Pi가 맞는 보드와 세션을 만들게 한다.
void sendHello() {
  uint8_t payload[proto::HELLO_PAYLOAD_SIZE];
  uint32_t capabilities = CAP_SINGLE_LEFT_HALL | CAP_DEADMAN |
                          CAP_PRESSURE_TELEMETRY;
  if (cfg::ENABLE_ESTOP) {
    capabilities |= CAP_ESTOP;
  }
  if (cfg::ENABLE_FRONT_RANGE_SENSORS) {
    capabilities |= CAP_TWO_RANGES;
  }
  if (cfg::ENABLE_BATTERY_SENSE) {
    capabilities |= CAP_BATTERY;
  }
  if (cfg::ENABLE_CURRENT_SENSE) {
    capabilities |= CAP_TWO_CURRENTS;
  }
  if (cfg::MAGNET_BENCH_MODE) {
    capabilities |= CAP_MAGNET_BENCH_MODE;
  }
  proto::writeU32(payload, g_boot_id);
  proto::writeU32(payload + 4U, capabilities);
  payload[8U] = proto::BOARD_ROLE_DRIVE;
  payload[9U] = proto::VERSION;
  proto::writeU16(payload + 10U, proto::SCHEMA_ID);
  proto::writeU32(payload + 12U, proto::FIRMWARE_RELEASE_ID);
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

// 고정 offset에 상태를 직렬화한다. 위치는 펄스 수, 속도는 mrad/s, speed age는 us이다.
void sendTelemetry() {
  if (!g_session_active) {
    return;
  }
  uint8_t payload[proto::TELEMETRY_PAYLOAD_SIZE];
  proto::writeI32(payload + 0U, g_drive.leftHallPulsePosition());
  proto::writeI32(payload + 4U, g_drive.rightHallPulsePosition());
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
  proto::writeU16(
      payload + 32U,
      g_pressure.leftRaw());
  proto::writeU16(
      payload + 34U,
      g_pressure.rightRaw());
  proto::writeU16(
      payload + 36U,
      static_cast<uint16_t>(g_pressure.leftFiltered() + 0.5F));
  proto::writeU16(
      payload + 38U,
      static_cast<uint16_t>(g_pressure.rightFiltered() + 0.5F));
  uint8_t pressure_flags = 0U;
  if (g_pressure.leftPresent()) {
    pressure_flags |= PRESSURE_FLAG_LEFT_PRESENT;
  }
  if (g_pressure.rightPresent()) {
    pressure_flags |= PRESSURE_FLAG_RIGHT_PRESENT;
  }
  if (g_pressure.calibrated()) {
    pressure_flags |= PRESSURE_FLAG_CALIBRATED;
  }
  payload[40U] = pressure_flags;
  payload[41U] = static_cast<uint8_t>(g_pressure.alert());
  proto::writeI16(payload + 42U, g_drive.feedforwardPwm());
  proto::writeI16(payload + 44U, g_drive.feedbackPwm());
  proto::writeI16(payload + 46U, g_drive.appliedPwm());
  proto::writeU32(payload + 48U, g_drive.speedAgeUs());
  payload[52U] = (g_drive.speedValid() ? 1U : 0U) | (g_new_pulse_since_telemetry ? 2U : 0U);
  payload[53U] = g_drive.braking() ? 1U : 0U;

  if (proto::sendFrame(
      Serial,
      proto::TYPE_TELEMETRY,
      g_tx_sequence++,
      g_session_id,
      millis(),
      payload,
      sizeof(payload))) {
    g_new_pulse_since_telemetry = false;
  }
}

bool stationaryDwellMet() {
  return g_stationary_tracking &&
         elapsedMs(millis(), g_stationary_since_ms) >=
             cfg::ARM_STATIONARY_DWELL_MS;
}

// 검증을 통과한 새 명령만 watchdog 시각을 갱신한다. 수신 바이트가 있다는 이유만으로 갱신하지 않는다.
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

// 현재 부팅/보드 역할/규약과 일치하는 세션만 수락하며, 연결만으로 모터를 켜지는 않는다.
bool handleSessionStart(
    const safestride_protocol::FrameView& frame) {
  if (g_session_active || !g_session_offer_active ||
      frame.payload_length != proto::SESSION_START_PAYLOAD_SIZE ||
      frame.session_id == 0UL) {
    return false;
  }
  const uint32_t expected_boot_id = proto::readU32(frame.payload);
  if (expected_boot_id != g_boot_id ||
      frame.payload[4U] != proto::BOARD_ROLE_DRIVE ||
      frame.payload[5U] != proto::VERSION ||
      proto::readU16(frame.payload + 6U) != proto::SCHEMA_ID ||
      proto::readU32(frame.payload + 8U) !=
          proto::FIRMWARE_RELEASE_ID) {
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

// 세션 → 순번 → 값 범위 → 물리 조건 → 상태 순서로 명령을 검사한다.
// 목표(mrad/s), TTL(ms), 경사 FF(count), 상한(count), mode를 하나의 명령으로 적용한다.
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

  const int32_t target = proto::readI32(frame.payload + 0U);
  const uint16_t ttl_ms = proto::readU16(frame.payload + 4U);
  const uint8_t enable = frame.payload[6U];
  const uint8_t reserved = frame.payload[7U];
  const int16_t slope_ff = static_cast<int16_t>(proto::readU16(frame.payload + 8U));
  const uint8_t pwm_cap = frame.payload[10U];
  const uint8_t mode = frame.payload[11U];
  if (slope_ff < -60 || slope_ff > 30 || pwm_cap > cfg::MAX_PWM || mode > 1U ||
      (mode == 1U && (target != 0L || slope_ff != 0))) return false;
  if ((enable != 0U && enable != 1U) || reserved != 0U) {
    return false;
  }
  if (ttl_ms < cfg::COMMAND_TTL_MIN_MS ||
      ttl_ms > cfg::COMMAND_WATCHDOG_MAX_MS) {
    return false;
  }
  if (target < -cfg::MAX_WHEEL_TARGET_MRAD_S ||
      target > cfg::MAX_WHEEL_TARGET_MRAD_S) {
    return false;
  }

  if (enable == 0U) {
    markAcceptedCommand(frame, ttl_ms);
    immediateStop(
        estopActive() ? ControllerState::ESTOP
                      : ControllerState::DISARMED,
        false);
    const bool physical_reset_safe =
        !estopActive() && (!cfg::REQUIRE_DEADMAN || !deadmanActive());
    if (physical_reset_safe) {
      if (!driverFaultActive()) {
        g_fault_bits &= static_cast<uint16_t>(~FAULT_MOTOR_DRIVER);
      }
      g_drive.clearRecoverableFaults();
      g_fault_bits &= static_cast<uint16_t>(~FAULT_LEFT_HALL);
    }
    if (g_fault_bits != 0U) {
      g_state = ControllerState::FAULT;
    }
    return true;
  }

  // While the physical dead-man remains released, only enabled zero commands
  // may refresh the watchdog as the MCU completes its local ramp. A new grasp
  // cancels this state in refreshPhysicalSafety() before serial processing.
  if (g_deadman_release_ramp_active) {
    if (g_state != ControllerState::ARMED || target != 0L) {
      return false;
    }
    markAcceptedCommand(frame, ttl_ms);
    g_requested_mrad_s = 0L;
    return true;
  }

  // An enable command is not accepted while a hardware interlock or latched
  // stop state is active. The bridge first clears a stop with a disabled frame;
  // a later fresh supervised command may then arm automatically.
  if ((!cfg::MAGNET_BENCH_MODE &&
       cfg::REQUIRE_HALL_CALIBRATION_FOR_ARM &&
       !cfg::HALL_CALIBRATED) ||
      estopActive() || !motionDeadmanSatisfied() || g_watchdog_timed_out ||
      g_fault_bits != 0U ||
      g_state == ControllerState::ESTOP ||
      g_state == ControllerState::SAFE_STOP ||
      g_state == ControllerState::FAULT) {
    return false;
  }

  if (g_state == ControllerState::DISARMED) {
    if (mode != 1U && !cfg::MAGNET_BENCH_MODE && !cfg::DEADMAN_DIRECT_DRIVE &&
        !stationaryDwellMet()) {
      return false;
    }
    markAcceptedCommand(frame, ttl_ms);
    g_state = ControllerState::ARMED;
    g_slope_ff_pwm = slope_ff;
    g_drive_pwm_cap = pwm_cap;
    g_brake_requested = mode == 1U;
    g_requested_mrad_s = target;
    return true;
  }

  if (g_state != ControllerState::ARMED) {
    return false;
  }
  markAcceptedCommand(frame, ttl_ms);
  g_slope_ff_pwm = slope_ff;
  g_drive_pwm_cap = pwm_cap;
  g_brake_requested = mode == 1U;
  g_requested_mrad_s = target;
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

// 명령 유효기간(TTL) 초과와 세션 활동 중단을 감시한다. ARMED에서 TTL 초과 시 세션도 무효화한다.
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

// 최소 5ms(200Hz) 간격으로 실행하되 실제 경과시간으로 ramp를 계산한다.
// 계산 주기가 빠르더라도 Hall의 새 속도 측정은 자석이 통과할 때만 생긴다.
void runControlLoop(uint32_t now_us) {
  const uint32_t elapsed_us = now_us - g_last_control_us;
  if (elapsed_us < cfg::CONTROL_PERIOD_US) {
    return;
  }
  g_last_control_us = now_us;

  g_hall.update(now_us);
  HallSample left_hall = {0UL, 0UL, 0xFFFFFFFFUL};
  HallSample right_hall = {0UL, 0UL, 0xFFFFFFFFUL};
  readHallSamples(now_us, left_hall, right_hall);
  const bool output_allowed =
      g_session_active &&
      g_state == ControllerState::ARMED &&
      !estopActive() &&
      (motionDeadmanSatisfied() || g_deadman_release_ramp_active) &&
      g_fault_bits == 0U;
  if (cfg::MAGNET_BENCH_MODE) {
    const uint32_t pulse_hold_us =
        static_cast<uint32_t>(cfg::MAGNET_BENCH_PULSE_HOLD_MS) * 1000UL;
    const bool magnet_pulse_recent = left_hall.age_us <= pulse_hold_us;
    g_drive.updateMagnetBench(
        elapsed_us,
        left_hall,
        right_hall,
        g_requested_mrad_s,
        output_allowed && magnet_pulse_recent);
  } else {
    g_drive.update(
        elapsed_us,
        left_hall,
        right_hall,
        g_requested_mrad_s,
        output_allowed,
        !cfg::DEADMAN_DIRECT_DRIVE,
        g_deadman_release_ramp_active
            ? g_deadman_release_decel_mrad_s2
            : cfg::MAX_DECEL_MRAD_S2,
        g_deadman_release_ramp_active,
        g_slope_ff_pwm,
        g_drive_pwm_cap,
        g_brake_requested);
  }
  g_new_pulse_since_telemetry |= g_drive.newPulse();
  const uint8_t hall_faults = g_drive.hallFaultMask();
  if (hall_faults != 0U) {
    if ((hall_faults & DriveController::HALL_FAULT_LEFT) != 0U) {
      g_fault_bits |= FAULT_LEFT_HALL;
    }
    immediateStop(ControllerState::FAULT, false);
    g_stationary_tracking = false;
    return;
  }

  if (g_deadman_release_ramp_active &&
      g_drive.appliedTargetMradS() == 0L) {
    immediateStop(ControllerState::SAFE_STOP, false);
    g_stationary_tracking = false;
    return;
  }

  const int32_t left_speed = g_drive.leftVelocityMradS();
  const int32_t right_speed = g_drive.rightVelocityMradS();
  // 필터 속도와 초기화 상태로 ARM 대기 조건을 판단한다. 방향 검증이나 물리적 정지의 직접 증명은 아니다.
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

// 전원을 넣으면 모터 출력을 먼저 정지 상태로 만든 뒤 센서/통신과 하드웨어 watchdog을 초기화한다.
void setup() {
#if defined(ARDUINO_ARCH_AVR)
  // Avoid a watchdog-reset loop before enabling the configured timeout below.
  MCUSR = 0U;
  wdt_disable();
#endif

  // The shared motor driver is driven to a zero/LOW state first.
  g_drive.begin();

  if (cfg::ENABLE_ESTOP) {
    pinMode(cfg::ESTOP_PIN, INPUT_PULLUP);
  }
  if (cfg::USE_DRIVER_FAULT_PIN) {
    pinMode(cfg::DRIVER_FAULT_PIN, INPUT_PULLUP);
  }
  Serial.begin(cfg::SERIAL_BAUD);
  g_hall.begin(micros());
  g_pressure.begin(millis());
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

// 안전 입력/기한 확인 → 명령 수신 → 제어 → telemetry 순서.
// 버퍼의 늦은 명령이 이미 지난 watchdog 기한을 덮어쓰지 않도록 수신 전에 검사한다.
void loop() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_reset();
#endif

  refreshPhysicalSafety();
  // Enforce an already-missed deadline before parsing buffered commands.
  // Otherwise a late but valid frame could overwrite the receive timestamp
  // and conceal a control-loop stall.
  uint32_t now_ms = millis();
  g_pressure.update(now_ms);
  refreshPhysicalSafety();
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
