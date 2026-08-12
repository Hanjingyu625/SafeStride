#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// SafeStride example hardware configuration
// ---------------------------------------------------------------------------
// Every value in this file is a placeholder.  Verify pin voltage, polarity,
// PWM capability, interrupt capability, gearbox ratio, and encoder convention
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

// Runtime encoder plausibility monitor. Tune from lifted-wheel logs. A fault
// latches until MCU reset so an intermittent encoder cannot silently re-arm.
constexpr int32_t ENCODER_STALL_TARGET_MIN_MRAD_S = 150L;
constexpr uint16_t ENCODER_STALL_TIMEOUT_MS = 400U;
constexpr int32_t ENCODER_REVERSE_MIN_MRAD_S = 100L;
constexpr uint16_t ENCODER_REVERSE_TIMEOUT_MS = 150U;
constexpr int32_t ENCODER_MAX_PLAUSIBLE_MRAD_S = 5000L;
constexpr uint16_t ENCODER_OVERSPEED_TIMEOUT_MS = 50U;

// Encoder A is sampled on CHANGE and encoder B determines direction, so this
// value must match that 2x-A-edge convention. Include the gearbox ratio.
constexpr uint32_t ENCODER_COUNTS_PER_WHEEL_REV = 1024UL;
constexpr int8_t LEFT_ENCODER_SIGN = 1;
constexpr int8_t RIGHT_ENCODER_SIGN = 1;

// Example Arduino Uno-compatible pins.
constexpr uint8_t LEFT_ENCODER_A_PIN = 2U;
constexpr uint8_t LEFT_ENCODER_B_PIN = 4U;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 3U;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 7U;

// SZH-GNP521 single-channel drivers use one dedicated PWM input plus IN1/IN2
// direction inputs. One driver is required per motor.
constexpr uint8_t LEFT_MOTOR_PWM_PIN = 5U;
constexpr uint8_t LEFT_MOTOR_IN1_PIN = 6U;
constexpr uint8_t LEFT_MOTOR_IN2_PIN = 8U;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 9U;
constexpr uint8_t RIGHT_MOTOR_IN1_PIN = 10U;
constexpr uint8_t RIGHT_MOTOR_IN2_PIN = 12U;
constexpr int8_t LEFT_MOTOR_SIGN = 1;
constexpr int8_t RIGHT_MOTOR_SIGN = -1;
constexpr uint16_t MAX_PWM = 100U;  // deliberately low for first lifted test

// Normally-closed E-stop example: normal contact pulls the pin to ground and
// pressing/disconnecting it produces HIGH through INPUT_PULLUP.
constexpr uint8_t ESTOP_PIN = A2;
constexpr uint8_t ESTOP_ACTIVE_LEVEL = HIGH;

// The two FSR channels replace the single digital dead-man switch. Each FSR
// must be wired as a voltage divider that reads near zero when released.
constexpr bool REQUIRE_DEADMAN = true;
constexpr uint8_t PRESSURE_LEFT_PIN = A0;
constexpr uint8_t PRESSURE_RIGHT_PIN = A1;
constexpr uint16_t PRESSURE_SAMPLE_PERIOD_MS = 100U;
constexpr float PRESSURE_FILTER_ALPHA = 0.2F;
constexpr float PRESSURE_HANDS_OFF_THRESHOLD = 100.0F;
constexpr float PRESSURE_IMBALANCE_THRESHOLD = 300.0F;
constexpr float PRESSURE_SUDDEN_CHANGE_THRESHOLD = 150.0F;

// Pressure-state LEDs. Analogue inputs can also act as digital outputs. A3/A4
// and A5 are available because optional current/battery sensing is disabled.
constexpr uint8_t LED_GREEN_PIN = A3;
constexpr uint8_t LED_YELLOW_PIN = A4;
constexpr uint8_t LED_RED_PIN = A5;

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

// PID output is PWM counts. Start with one wheel lifted, tune one wheel at a
// time, and keep integrator gain at zero until direction and feedback are
// proven. These example gains are intentionally mild.
constexpr float LEFT_PID_KP = 12.0F;
constexpr float LEFT_PID_KI = 0.0F;
constexpr float LEFT_PID_KD = 0.0F;
constexpr float LEFT_FEEDFORWARD = 10.0F;
constexpr float RIGHT_PID_KP = 12.0F;
constexpr float RIGHT_PID_KI = 0.0F;
constexpr float RIGHT_PID_KD = 0.0F;
constexpr float RIGHT_FEEDFORWARD = 10.0F;
constexpr float PID_INTEGRAL_LIMIT = 30.0F;
constexpr float VELOCITY_FILTER_ALPHA = 0.35F;

// AVR's hardware watchdog protects against a frozen main loop. The driver's
// PWM input still needs an external pull-down while the MCU resets.
// A persistent boot counter prevents buffered pre-reset commands from matching
// a newly booted controller. EEPROM.put() updates only changed bytes.
constexpr int AVR_BOOT_COUNTER_EEPROM_ADDRESS = 0;

static_assert(
    LEFT_MOTOR_SIGN == 1 || LEFT_MOTOR_SIGN == -1,
    "LEFT_MOTOR_SIGN must be +1 or -1");
static_assert(
    RIGHT_MOTOR_SIGN == 1 || RIGHT_MOTOR_SIGN == -1,
    "RIGHT_MOTOR_SIGN must be +1 or -1");
static_assert(
    LEFT_ENCODER_SIGN == 1 || LEFT_ENCODER_SIGN == -1,
    "LEFT_ENCODER_SIGN must be +1 or -1");
static_assert(
    RIGHT_ENCODER_SIGN == 1 || RIGHT_ENCODER_SIGN == -1,
    "RIGHT_ENCODER_SIGN must be +1 or -1");
static_assert(
    MAX_PWM > 0U && MAX_PWM <= 255U,
    "MAX_PWM must fit the Arduino analogue output range");
static_assert(
    ENCODER_COUNTS_PER_WHEEL_REV > 0UL,
    "encoder counts per wheel revolution must be positive");
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
    ENCODER_STALL_TARGET_MIN_MRAD_S > 0L &&
        ENCODER_STALL_TARGET_MIN_MRAD_S <=
            MAX_WHEEL_TARGET_MRAD_S,
    "encoder stall target threshold is invalid");
static_assert(
    ENCODER_STALL_TIMEOUT_MS > 0U &&
        ENCODER_REVERSE_TIMEOUT_MS > 0U &&
        ENCODER_OVERSPEED_TIMEOUT_MS > 0U,
    "encoder plausibility timeouts must be positive");
static_assert(
    ENCODER_REVERSE_MIN_MRAD_S > 0L,
    "encoder reverse threshold must be positive");
static_assert(
    ENCODER_MAX_PLAUSIBLE_MRAD_S >
        MAX_WHEEL_TARGET_MRAD_S,
    "encoder plausible speed must exceed maximum target");
static_assert(
    AVR_BOOT_COUNTER_EEPROM_ADDRESS >= 0,
    "boot-counter EEPROM address must not be negative");

}  // namespace safestride_config
