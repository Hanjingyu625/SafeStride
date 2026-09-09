// Terrain Uno 진입점: ToF 거리와 MPU 자세를 읽어 Pi에 전송한다. 모터를 직접 제어하지 않는다.
// Pi가 여기서 받은 pitch/유효성으로 경사 제한과 BRAKE를 결정하여 Drive Uno로 보낸다.

#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_ARCH_AVR)
#include <EEPROM.h>
#include <avr/wdt.h>
#endif

#include "config.h"
#include "mpu6050_sensor.h"
#include "protocol.h"
#include "tof10120_sensor.h"

#if !defined(ARDUINO_ARCH_AVR) && !defined(SAFESTRIDE_HOST_BUILD)
#error "Non-AVR port needs a persistent boot ID and hardware watchdog."
#endif

namespace cfg = safestride_terrain_config;
namespace proto = safestride_protocol;

constexpr uint32_t CAP_TOF10120 = 1UL << 8U;
constexpr uint32_t CAP_MPU6050 = 1UL << 9U;
constexpr uint16_t FAULT_TOF_INVALID = 1U << 0U;
constexpr uint16_t FAULT_MPU_INVALID = 1U << 1U;

proto::FrameReceiver g_receiver;
Tof10120Sensor g_tof;
Mpu6050Sensor g_mpu;
uint32_t g_boot_id = 0UL;
uint32_t g_session_id = 0UL;
bool g_session_active = false;
uint16_t g_tx_sequence = 0U;
uint32_t g_last_hello_ms = 0UL;
uint32_t g_last_telemetry_ms = 0UL;

// 재부팅마다 바뀌는 식별자로 이전 부팅의 세션 시작 요청을 걸러낸다.
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
  value = boot_counter * 0x9E3779B9UL;
#else
  value = 0x544F4631UL;  // Host-test value, "TOF1".
#endif
  value ^= value << 13U;
  value ^= value >> 17U;
  value ^= value << 5U;
  return value == 0UL ? 0x544F4631UL : value;
}

uint16_t roundedUnsigned16(float value) {
  if (value <= 0.0F) {
    return 0U;
  }
  if (value >= 65535.0F) {
    return 65535U;
  }
  return static_cast<uint16_t>(value + 0.5F);
}

int16_t roundedSigned16(float value) {
  if (value >= 32767.0F) {
    return 32767;
  }
  if (value <= -32768.0F) {
    return static_cast<int16_t>(-32768);
  }
  return static_cast<int16_t>(
      value >= 0.0F ? value + 0.5F : value - 0.5F);
}

// Terrain 역할과 ToF/MPU 지원 여부를 Pi에 알린다.
void sendHello() {
  uint8_t payload[proto::HELLO_PAYLOAD_SIZE];
  proto::writeU32(payload + 0U, g_boot_id);
  uint32_t capabilities = CAP_TOF10120;
  if (cfg::ENABLE_MPU6050) {
    capabilities |= CAP_MPU6050;
  }
  proto::writeU32(payload + 4U, capabilities);
  payload[8U] = proto::BOARD_ROLE_TERRAIN;
  payload[9U] = proto::VERSION;
  proto::writeU16(payload + 10U, proto::SCHEMA_ID);
  proto::writeU32(payload + 12U, proto::FIRMWARE_RELEASE_ID);
  proto::sendFrame(
      Serial,
      proto::TYPE_HELLO,
      g_tx_sequence++,
      0UL,
      millis(),
      payload,
      sizeof(payload));
}

// 거리(mm), 가속도(mg), 자이로(mrad/s), 자세(mrad), 유효성/고장을 고정 offset으로 보낸다.
// 31~44 바이트는 예약 영역으로 0을 유지하며 GPS는 Pi가 직접 수집한다.
void sendTelemetry() {
  if (!g_session_active) {
    return;
  }
  // Keep the protocol-v4 payload size stable while the former GPS fields at
  // offsets 31..44 become reserved. GPS is acquired directly by the Pi.
  uint8_t payload[proto::TERRAIN_TELEMETRY_PAYLOAD_SIZE] = {0U};
  proto::writeU16(payload + 0U, g_tof.distanceMm());
  payload[2U] = g_tof.valid() ? 1U : 0U;
  payload[3U] = static_cast<uint8_t>(g_tof.alert());
  proto::writeU16(
      payload + 4U, roundedUnsigned16(g_tof.filteredDistanceMm()));
  proto::writeU16(
      payload + 6U, roundedUnsigned16(g_tof.referenceDistanceMm()));
  proto::writeI16(payload + 8U, roundedSigned16(g_tof.errorMm()));
  proto::writeI16(payload + 10U, roundedSigned16(g_tof.changeMm()));
  proto::writeI16(payload + 12U, g_mpu.accelXMg());
  proto::writeI16(payload + 14U, g_mpu.accelYMg());
  proto::writeI16(payload + 16U, g_mpu.accelZMg());
  proto::writeI16(payload + 18U, g_mpu.gyroXMradS());
  proto::writeI16(payload + 20U, g_mpu.gyroYMradS());
  proto::writeI16(payload + 22U, g_mpu.gyroZMradS());
  proto::writeI16(payload + 24U, g_mpu.rollMrad());
  proto::writeI16(payload + 26U, g_mpu.pitchMrad());
  payload[28U] = g_mpu.valid() ? 1U : 0U;
  uint16_t faults = g_tof.valid() ? 0U : FAULT_TOF_INVALID;
  if (cfg::ENABLE_MPU6050 && !g_mpu.valid()) {
    faults |= FAULT_MPU_INVALID;
  }
  proto::writeU16(payload + 29U, faults);
  proto::sendFrame(
      Serial,
      proto::TYPE_TERRAIN_TELEMETRY,
      g_tx_sequence++,
      g_session_id,
      millis(),
      payload,
      sizeof(payload));
}

// 현재 부팅 ID와 보드 역할/규약이 맞는 SESSION_START만 수락한다. 주행 명령은 여기서 처리하지 않는다.
void processHostProtocol() {
  proto::FrameView frame = {0U, 0U, 0U, 0U, 0UL, 0UL, NULL};
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      break;
    }
    if (g_receiver.push(static_cast<uint8_t>(incoming), frame) !=
        proto::ReceiveResult::FRAME_READY) {
      continue;
    }
    if (frame.type != proto::TYPE_SESSION_START ||
        frame.payload_length != proto::SESSION_START_PAYLOAD_SIZE ||
        frame.session_id == 0UL ||
        proto::readU32(frame.payload) != g_boot_id ||
        frame.payload[4U] != proto::BOARD_ROLE_TERRAIN ||
        frame.payload[5U] != proto::VERSION ||
        proto::readU16(frame.payload + 6U) != proto::SCHEMA_ID ||
        proto::readU32(frame.payload + 8U) !=
            proto::FIRMWARE_RELEASE_ID) {
      continue;
    }
    g_session_id = frame.session_id;
    g_session_active = true;
    g_last_telemetry_ms = millis() - cfg::TELEMETRY_PERIOD_MS;
  }
}

// I2C/직렬과 센서를 초기화하고 AVR의 500ms 하드웨어 watchdog을 켠다.
void setup() {
#if defined(ARDUINO_ARCH_AVR)
  MCUSR = 0U;
  wdt_disable();
#endif
  Wire.begin();
  Serial.begin(cfg::SERIAL_BAUD);
  const uint32_t now_ms = millis();
  g_tof.begin(now_ms);
  g_mpu.begin(now_ms);
  g_boot_id = makeBootId();
  g_last_hello_ms = now_ms - cfg::HELLO_PERIOD_MS;
  g_last_telemetry_ms = now_ms;
#if defined(ARDUINO_ARCH_AVR)
  wdt_enable(WDTO_500MS);
#endif
}

// 통신 처리와 주기별 센서 갱신을 반복한다. 센서 오류/무효 정보도 Pi가 판단하도록 전송한다.
void loop() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_reset();
#endif
  processHostProtocol();
  const uint32_t now_ms = millis();
  g_tof.update(now_ms);
  g_mpu.update(now_ms);
  if (now_ms - g_last_hello_ms >= cfg::HELLO_PERIOD_MS) {
    g_last_hello_ms = now_ms;
    sendHello();
  }
  if (g_session_active &&
      now_ms - g_last_telemetry_ms >= cfg::TELEMETRY_PERIOD_MS) {
    g_last_telemetry_ms = now_ms;
    sendTelemetry();
  }
}
