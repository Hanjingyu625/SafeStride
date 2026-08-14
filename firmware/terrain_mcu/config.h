#pragma once

#include <Arduino.h>

namespace safestride_terrain_config {

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t HELLO_PERIOD_MS = 500U;
constexpr uint16_t TELEMETRY_PERIOD_MS = 50U;
constexpr int AVR_BOOT_COUNTER_EEPROM_ADDRESS = 8;

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

}  // namespace safestride_terrain_config
