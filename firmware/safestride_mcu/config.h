#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// SafeStride example hardware configuration
// ---------------------------------------------------------------------------
// Every value in this file is a placeholder. Verify pin voltage, polarity,
// PWM capability, interrupt capability, Hall pulse count, and gearbox ratio
// against the actual hardware before connecting motor power.

namespace safestride_config {

constexpr uint32_t SERIAL_BAUD = 115200UL;

// Scheduler.
constexpr uint32_t CONTROL_PERIOD_US = 5000UL;   // 200 Hz
constexpr uint16_t TELEMETRY_PERIOD_MS = 10U;    // 100 Hz
constexpr uint16_t HELLO_PERIOD_MS = 500U;
constexpr uint16_t SESSION_LOSS_TIMEOUT_MS = 1000U;

// Only an accepted, newer COMMAND frame refreshes this watchdog.  The effective
// timeout is the smaller of this value and the TTL carried in the command.
constexpr uint16_t COMMAND_WATCHDOG_MAX_MS = 250U;
constexpr uint16_t COMMAND_TTL_MIN_MS = 20U;

// Motion limits at the wheel output shaft.
constexpr int32_t MAX_WHEEL_TARGET_MRAD_S = 3000L;
constexpr int32_t MAX_ACCEL_MRAD_S2 = 1200L;
constexpr int32_t MAX_DECEL_MRAD_S2 = 2500L;
constexpr int32_t ARM_MAX_MEASURED_SPEED_MRAD_S = 100L;
constexpr uint16_t ARM_STATIONARY_DWELL_MS = 250U;

// One single-output Hall sensor is installed per wheel. The interrupt counts
// falling edges and the controller estimates speed from the pulse period.
// A single channel cannot measure direction independently; velocity sign is
// derived from the commanded direction of the shared motor driver.
constexpr uint8_t LEFT_HALL_PIN = 2U;
constexpr uint8_t RIGHT_HALL_PIN = 3U;
constexpr uint8_t HALL_ACTIVE_LEVEL = LOW;
// Reject sub-millisecond electrical chatter without discarding valid pulses
// if the measured pulse-per-revolution value is increased later.
constexpr uint32_t HALL_MIN_PULSE_INTERVAL_US = 500UL;
constexpr uint32_t HALL_ZERO_TIMEOUT_US = 1500000UL;
// Measure this with the standalone sensor bench. Keep HALL_CALIBRATED false
// until both channels produce the verified number of pulses per wheel turn.
constexpr uint32_t HALL_PULSES_PER_WHEEL_REV = 1UL;
constexpr bool HALL_CALIBRATED = false;

// Runtime Hall plausibility monitor. Tune from lifted-wheel logs. A fault
// latches until MCU reset so an intermittent sensor cannot silently re-arm.
constexpr int32_t HALL_STALL_TARGET_MIN_MRAD_S = 300L;
constexpr uint16_t HALL_STALL_TIMEOUT_MS = 1500U;
constexpr int32_t HALL_MAX_PLAUSIBLE_MRAD_S = 5000L;
constexpr uint16_t HALL_OVERSPEED_TIMEOUT_MS = 100U;

// One SZH-GNP521 drives the two motors as one electrical load. The wiring must
// be checked independently for voltage, polarity and combined stall current.
constexpr uint8_t MOTOR_PWM_PIN = 5U;
constexpr uint8_t MOTOR_IN1_PIN = 6U;
constexpr uint8_t MOTOR_IN2_PIN = 8U;
constexpr int8_t MOTOR_SIGN = 1;
constexpr uint16_t MAX_PWM = 100U;  // deliberately low for first lifted test

// E-stop hardware is not implemented in the current build. Keep this false so
// the input is not configured, the reported state stays normal, and the
// capability is not advertised. The pin/polarity are reserved for a future
// normally-closed, independently validated hardware interlock.
constexpr bool ENABLE_ESTOP = false;
constexpr uint8_t ESTOP_PIN = A2;
constexpr uint8_t ESTOP_ACTIVE_LEVEL = HIGH;

// The two FSR channels replace the single digital dead-man switch. Each FSR
// must be wired as a voltage divider that reads near zero when released.
constexpr bool REQUIRE_DEADMAN = true;
constexpr uint8_t PRESSURE_LEFT_PIN = A0;
constexpr uint8_t PRESSURE_RIGHT_PIN = A1;
constexpr uint16_t PRESSURE_SAMPLE_PERIOD_MS = 100U;
constexpr float PRESSURE_FILTER_ALPHA = 0.2F;
// Watch /handle/pressure with the motors isolated, then set each channel's
// polarity and threshold halfway between its released and held readings.
constexpr bool PRESSURE_LEFT_ACTIVE_HIGH = true;
constexpr bool PRESSURE_RIGHT_ACTIVE_HIGH = true;
constexpr float PRESSURE_LEFT_PRESENT_THRESHOLD = 100.0F;
constexpr float PRESSURE_RIGHT_PRESENT_THRESHOLD = 100.0F;
constexpr bool PRESSURE_THRESHOLDS_CALIBRATED = false;
constexpr float PRESSURE_IMBALANCE_THRESHOLD = 300.0F;
constexpr float PRESSURE_SUDDEN_CHANGE_THRESHOLD = 150.0F;

// Set to a real input and true only after wiring a driver's fault output.
constexpr bool USE_DRIVER_FAULT_PIN = false;
constexpr uint8_t DRIVER_FAULT_PIN = 13U;
constexpr uint8_t DRIVER_FAULT_ACTIVE_LEVEL = LOW;

// Optional analogue telemetry. Disabled values are sent using protocol
// sentinels and are never interpreted by ROS as real measurements.
constexpr bool ENABLE_BATTERY_SENSE = false;
constexpr uint8_t BATTERY_SENSE_PIN = A5;
constexpr float ADC_REFERENCE_V = 5.0F;
constexpr float BATTERY_DIVIDER_RATIO = 3.0F;

constexpr bool ENABLE_CURRENT_SENSE = false;
constexpr uint8_t LEFT_CURRENT_SENSE_PIN = A3;
constexpr uint8_t RIGHT_CURRENT_SENSE_PIN = A4;
constexpr float CURRENT_ZERO_V = 2.5F;
constexpr float CURRENT_MA_PER_V = 1000.0F;

// The v2 telemetry layout reserves two front-range fields, but no such sensors
// are installed in the current pin map. Do not advertise the capability or
// require the ROS topics until real non-blocking drivers replace the sentinels.
constexpr bool ENABLE_FRONT_RANGE_SENSORS = false;

// PID output is PWM counts. Keep both wheels lifted, tune the shared output,
// and keep integrator gain at zero until direction and both Hall channels are
// proven. These example gains are intentionally mild.
constexpr float MOTOR_PID_KP = 12.0F;
constexpr float MOTOR_PID_KI = 0.0F;
constexpr float MOTOR_PID_KD = 0.0F;
constexpr float MOTOR_FEEDFORWARD = 10.0F;
constexpr float PID_INTEGRAL_LIMIT = 30.0F;
constexpr float VELOCITY_FILTER_ALPHA = 0.35F;

// AVR's hardware watchdog protects against a frozen main loop. The driver's
// PWM input still needs an external pull-down while the MCU resets.
// A persistent boot counter prevents buffered pre-reset commands from matching
// a newly booted controller. EEPROM.put() updates only changed bytes.
constexpr int AVR_BOOT_COUNTER_EEPROM_ADDRESS = 0;

static_assert(
    MOTOR_SIGN == 1 || MOTOR_SIGN == -1,
    "MOTOR_SIGN must be +1 or -1");
static_assert(
    MAX_PWM > 0U && MAX_PWM <= 255U,
    "MAX_PWM must fit the Arduino analogue output range");
static_assert(
    HALL_PULSES_PER_WHEEL_REV > 0UL,
    "Hall pulses per wheel revolution must be positive");
static_assert(
    HALL_MIN_PULSE_INTERVAL_US > 0UL &&
        HALL_ZERO_TIMEOUT_US > HALL_MIN_PULSE_INTERVAL_US,
    "Hall timing limits are invalid");
static_assert(
    COMMAND_WATCHDOG_MAX_MS >= COMMAND_TTL_MIN_MS,
    "command TTL range is invalid");
static_assert(
    ARM_MAX_MEASURED_SPEED_MRAD_S >= 0L,
    "arming stationary-speed threshold must not be negative");
static_assert(
    ARM_STATIONARY_DWELL_MS > 0U,
    "arming stationary dwell must be positive");
static_assert(
    HALL_STALL_TARGET_MIN_MRAD_S > 0L &&
        HALL_STALL_TARGET_MIN_MRAD_S <=
            MAX_WHEEL_TARGET_MRAD_S,
    "Hall stall target threshold is invalid");
static_assert(
    HALL_STALL_TIMEOUT_MS > 0U &&
        HALL_OVERSPEED_TIMEOUT_MS > 0U,
    "Hall plausibility timeouts must be positive");
static_assert(
    HALL_MAX_PLAUSIBLE_MRAD_S >
        MAX_WHEEL_TARGET_MRAD_S,
    "Hall plausible speed must exceed maximum target");
static_assert(
    AVR_BOOT_COUNTER_EEPROM_ADDRESS >= 0,
    "boot-counter EEPROM address must not be negative");
static_assert(
    PRESSURE_LEFT_PRESENT_THRESHOLD >= 0.0F &&
        PRESSURE_LEFT_PRESENT_THRESHOLD <= 1023.0F &&
        PRESSURE_RIGHT_PRESENT_THRESHOLD >= 0.0F &&
        PRESSURE_RIGHT_PRESENT_THRESHOLD <= 1023.0F,
    "pressure thresholds must fit the ADC range");

}  // namespace safestride_config
