#pragma once

#include <Arduino.h>

namespace safestride_terrain_config {

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t HELLO_PERIOD_MS = 500U;
constexpr uint16_t TELEMETRY_PERIOD_MS = 50U;
constexpr uint16_t SESSION_LOSS_TIMEOUT_MS = 1500U;
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
constexpr float TOF_REFERENCE_FREEZE_THRESHOLD_MM = 30.0F;
constexpr uint8_t TOF_BASELINE_SAMPLES = 10U;
constexpr uint8_t TOF_REQUIRED_FRAMES = 4U;
constexpr uint16_t TOF_RED_HOLD_MS = 1000U;

// GY-521 MPU6050 shares A4/A5 with the TOF. AD0 selects 0x68 or 0x69.
constexpr bool ENABLE_MPU6050 = true;
constexpr uint8_t MPU6050_ADDRESS_LOW = 0x68U;
constexpr uint8_t MPU6050_ADDRESS_HIGH = 0x69U;
constexpr uint16_t MPU6050_SAMPLE_PERIOD_MS = 50U;
// DLPF is enabled, so the MPU6050 register source rate is 1 kHz. A divider
// of 49 makes the sensor output rate match the firmware's 20 Hz read rate.
constexpr uint8_t MPU6050_SAMPLE_RATE_DIVIDER = 49U;
constexpr uint16_t MPU6050_RECONNECT_PERIOD_MS = 1000U;
constexpr uint8_t MPU6050_MAX_CONSECUTIVE_ERRORS = 3U;
// Mounting convention used by the attitude equations: +X forward, +Y left,
// +Z up. The runtime pitch offset and sign are calibrated on the Pi.
constexpr float MPU6050_ATTITUDE_ALPHA = 0.15F;

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
    TOF_BASELINE_SAMPLES >= TOF_REQUIRED_FRAMES,
    "TOF baseline must contain enough samples");
static_assert(
    (1000UL / (static_cast<uint32_t>(MPU6050_SAMPLE_RATE_DIVIDER) + 1UL)) *
            MPU6050_SAMPLE_PERIOD_MS ==
        1000UL,
    "MPU6050 output and firmware sample periods must match");
static_assert(
    MPU6050_ATTITUDE_ALPHA > 0.0F && MPU6050_ATTITUDE_ALPHA <= 1.0F,
    "MPU6050 attitude alpha must be in (0, 1]");

}  // namespace safestride_terrain_config
