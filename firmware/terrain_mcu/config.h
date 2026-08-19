#pragma once

#include <Arduino.h>

namespace safestride_terrain_config {

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t HELLO_PERIOD_MS = 500U;
constexpr uint16_t TELEMETRY_PERIOD_MS = 50U;
constexpr uint16_t SESSION_LOSS_TIMEOUT_MS = 1500U;
constexpr int AVR_BOOT_COUNTER_EEPROM_ADDRESS = 8;

// BE-220 NMEA input on Terrain Uno. AltSoftSerial uses fixed ATmega328P pins:
// D8 is RX from GPS TX, and D9 is TX to GPS RX. The module must be configured
// to 9600 baud before connection. The factory 115200 baud leaves too little
// interrupt-latency margin on a 16 MHz Uno while TOF and USB serial also run.
constexpr bool ENABLE_GPS = true;
constexpr uint8_t GPS_RX_PIN = 8U;
constexpr uint8_t GPS_TX_PIN = 9U;
constexpr uint32_t GPS_BAUD = 9600UL;
constexpr uint16_t GPS_TELEMETRY_PERIOD_MS = 200U;
constexpr uint16_t GPS_STALE_TIMEOUT_MS = 2000U;

// The TOF-10120 datasheet gives the 8-bit address as 0xA4. Arduino Wire uses
// the corresponding 7-bit address, 0x52.
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

static_assert(
    TOF_MIN_VALID_DISTANCE_MM < TOF_MAX_VALID_DISTANCE_MM,
    "TOF valid range is invalid");
static_assert(
    TELEMETRY_PERIOD_MS >= TOF_SAMPLE_PERIOD_MS,
    "telemetry must not run faster than TOF sampling");
static_assert(
    SESSION_LOSS_TIMEOUT_MS > HELLO_PERIOD_MS * 2U,
    "session timeout must allow multiple HELLO replies");
static_assert(
    TOF_FILTER_ALPHA > 0.0F && TOF_FILTER_ALPHA <= 1.0F,
    "TOF filter alpha must be in (0, 1]");
static_assert(
    TOF_REFERENCE_ALPHA > 0.0F && TOF_REFERENCE_ALPHA <= 1.0F,
    "TOF reference alpha must be in (0, 1]");
static_assert(
    TOF_REQUIRED_FRAMES > 0U,
    "TOF required frame count must be positive");
static_assert(
    GPS_RX_PIN == 8U && GPS_TX_PIN == 9U,
    "AltSoftSerial on Arduino Uno requires GPS RX/TX pins D8/D9");
static_assert(
    GPS_BAUD >= 4800UL && GPS_BAUD <= 31250UL,
    "Terrain Uno GPS baud must be reliable with AltSoftSerial");
static_assert(
    GPS_STALE_TIMEOUT_MS > GPS_TELEMETRY_PERIOD_MS,
    "GPS stale timeout must exceed its telemetry period");

}  // namespace safestride_terrain_config
