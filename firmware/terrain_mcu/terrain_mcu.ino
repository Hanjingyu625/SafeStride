#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_ARCH_AVR)
#include <EEPROM.h>
#include <avr/wdt.h>
#endif

#include "config.h"
#include "gps_receiver.h"
#include "protocol.h"
#include "tof10120_sensor.h"

#if !defined(ARDUINO_ARCH_AVR) && !defined(SAFESTRIDE_HOST_BUILD)
#error "Non-AVR port needs a persistent boot ID and hardware watchdog."
#endif

namespace cfg = safestride_terrain_config;
namespace proto = safestride_protocol;

constexpr uint32_t CAP_TOF10120 = 1UL << 8U;
constexpr uint32_t CAP_GPS_NMEA = 1UL << 9U;
constexpr uint16_t FAULT_TOF_INVALID = 1U << 0U;

enum class LegState : uint8_t {
  STOWED,
  DEPLOYING,
  DEPLOYED,
  RETRACTING,
  SAFE_STOP,
  FAULT,
};

proto::FrameReceiver g_receiver;
Tof10120Sensor g_tof;
GpsReceiver g_gps;
LegState g_leg_state = LegState::SAFE_STOP;
uint32_t g_boot_id = 0UL;
uint32_t g_session_id = 0UL;
bool g_session_active = false;
bool g_session_offer_active = false;
uint16_t g_tx_sequence = 0U;
uint32_t g_last_hello_ms = 0UL;
uint32_t g_last_telemetry_ms = 0UL;
uint32_t g_last_session_activity_ms = 0UL;
uint32_t g_last_gps_telemetry_ms = 0UL;

void disableLegImmediately() {
  // Leg hardware is intentionally not armed by this sensor-only firmware.
  // Add verified enable/direction pins, limit inputs and a command watchdog
  // before changing this state.
  g_leg_state = LegState::SAFE_STOP;
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

void sendHello() {
  uint8_t payload[proto::HELLO_PAYLOAD_SIZE];
  uint32_t capabilities = CAP_TOF10120;
  if (cfg::ENABLE_GPS) {
    capabilities |= CAP_GPS_NMEA;
  }
  proto::writeU32(payload + 0U, g_boot_id);
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
  uint8_t payload[proto::TERRAIN_TELEMETRY_PAYLOAD_SIZE];
  proto::writeU16(payload + 0U, g_tof.distanceMm());
  payload[2U] = g_tof.valid() ? 1U : 0U;
  payload[3U] = static_cast<uint8_t>(g_tof.alert());
  proto::writeU16(
      payload + 4U, roundedUnsigned16(g_tof.filteredDistanceMm()));
  proto::writeU16(
      payload + 6U, roundedUnsigned16(g_tof.referenceDistanceMm()));
  proto::writeI16(payload + 8U, roundedSigned16(g_tof.errorMm()));
  proto::writeI16(payload + 10U, roundedSigned16(g_tof.changeMm()));
  proto::writeU16(
      payload + 12U, g_tof.valid() ? 0U : FAULT_TOF_INVALID);
  proto::sendFrame(
      Serial,
      proto::TYPE_TERRAIN_TELEMETRY,
      g_tx_sequence++,
      g_session_id,
      millis(),
      payload,
      sizeof(payload));
}

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
    if (!g_session_offer_active ||
        frame.type != proto::TYPE_SESSION_START ||
        frame.payload_length != proto::SESSION_START_PAYLOAD_SIZE ||
        frame.session_id == 0UL ||
        proto::readU32(frame.payload) != g_boot_id) {
      continue;
    }
    // A delayed packet from an older host must not replace a live session.
    // A restarted host can establish its new ID after the heartbeat timeout.
    if (g_session_active && frame.session_id != g_session_id) {
      continue;
    }
    g_session_id = frame.session_id;
    g_session_active = true;
    g_session_offer_active = false;
    g_last_session_activity_ms = millis();
    g_last_telemetry_ms = millis() - cfg::TELEMETRY_PERIOD_MS;
  }
}

void sendGpsTelemetry(uint32_t now_ms) {
  if (!g_session_active || !cfg::ENABLE_GPS) {
    return;
  }
  const GpsSample sample = g_gps.sample(now_ms);
  uint8_t payload[proto::GPS_TELEMETRY_PAYLOAD_SIZE];
  proto::writeU32(payload + 0U, static_cast<uint32_t>(sample.latitude_e7));
  proto::writeU32(payload + 4U, static_cast<uint32_t>(sample.longitude_e7));
  proto::writeU32(payload + 8U, sample.speed_mm_s);
  payload[12U] = sample.flags;
  payload[13U] = sample.satellites;
  proto::sendFrame(
      Serial,
      proto::TYPE_GPS_TELEMETRY,
      g_tx_sequence++,
      g_session_id,
      now_ms,
      payload,
      sizeof(payload));
}

void enforceHostSessionTimeout(uint32_t now_ms) {
  if (g_session_active &&
      now_ms - g_last_session_activity_ms >
          cfg::SESSION_LOSS_TIMEOUT_MS) {
    g_session_active = false;
    g_session_offer_active = false;
    g_session_id = 0UL;
    disableLegImmediately();
  }
}

void setup() {
#if defined(ARDUINO_ARCH_AVR)
  MCUSR = 0U;
  wdt_disable();
#endif
  disableLegImmediately();
  Wire.begin();
  Serial.begin(cfg::SERIAL_BAUD);
  g_gps.begin();
  const uint32_t now_ms = millis();
  g_tof.begin(now_ms);
  g_boot_id = makeBootId();
  g_last_hello_ms = now_ms - cfg::HELLO_PERIOD_MS;
  g_last_telemetry_ms = now_ms;
  g_last_session_activity_ms = now_ms;
  g_last_gps_telemetry_ms = now_ms;
#if defined(ARDUINO_ARCH_AVR)
  wdt_enable(WDTO_500MS);
#endif
}

void loop() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_reset();
#endif
  disableLegImmediately();
  g_gps.poll();
  processHostProtocol();
  const uint32_t now_ms = millis();
  enforceHostSessionTimeout(now_ms);
  g_tof.update(now_ms);
  if (now_ms - g_last_hello_ms >= cfg::HELLO_PERIOD_MS) {
    g_last_hello_ms = now_ms;
    sendHello();
  }
  if (g_session_active &&
      now_ms - g_last_telemetry_ms >= cfg::TELEMETRY_PERIOD_MS) {
    g_last_telemetry_ms = now_ms;
    sendTelemetry();
  }
  if (g_session_active && cfg::ENABLE_GPS &&
      now_ms - g_last_gps_telemetry_ms >=
          cfg::GPS_TELEMETRY_PERIOD_MS) {
    g_last_gps_telemetry_ms = now_ms;
    sendGpsTelemetry(now_ms);
  }
}
