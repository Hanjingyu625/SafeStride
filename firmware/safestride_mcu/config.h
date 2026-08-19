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

// Current hardware uses one Hall sensor as shared wheel-speed feedback. The
// firmware mirrors that sensor into both telemetry channels so the ROS bridge
// and odometry path keep their existing two-wheel message shape.
constexpr bool USE_SINGLE_HALL_SENSOR = true;
constexpr uint8_t LEFT_HALL_PIN = 2U;
constexpr uint8_t RIGHT_HALL_PIN = 3U;
constexpr uint8_t HALL_ACTIVE_LEVEL = LOW;
// Reject sub-millisecond electrical chatter without discarding valid pulses
// if the measured pulse-per-revolution value is increased later.
constexpr uint32_t HALL_MIN_PULSE_INTERVAL_US = 500UL;
// With one magnet per revolution the interval can be several seconds. Keep the
// last period estimate long enough for a 0.5 rad/s wheel to produce a new pulse.
constexpr uint32_t HALL_ZERO_TIMEOUT_US = 15000000UL;
// The current wheel setup has one magnet and therefore one pulse per complete
// wheel revolution. Change this value in both MCU and ROS configuration if the
// number of magnets or the sensing geometry changes.
constexpr uint32_t HALL_PULSES_PER_WHEEL_REV = 1UL;
constexpr bool HALL_CALIBRATED = true;

// Runtime Hall plausibility monitor. Tune from lifted-wheel logs. A fault
// latches until MCU reset so an intermittent sensor cannot silently re-arm.
constexpr int32_t HALL_STALL_TARGET_MIN_MRAD_S = 500L;
constexpr uint16_t HALL_STALL_TIMEOUT_MS = 15000U;
constexpr int32_t HALL_MAX_PLAUSIBLE_MRAD_S = 5000L;
constexpr uint16_t HALL_OVERSPEED_TIMEOUT_MS = 100U;

// One SZH-GNP521 drives the two motors as one electrical load. The wiring must
// be checked independently for voltage, polarity and combined stall current.
constexpr uint8_t MOTOR_PWM_PIN = 5U;
constexpr uint8_t MOTOR_IN1_PIN = 6U;
constexpr uint8_t MOTOR_IN2_PIN = 8U;
constexpr int8_t MOTOR_SIGN = 1;
constexpr uint16_t MAX_PWM = 100U;  // deliberately low for first lifted test
// The tested motor/driver pair does not start reliably below this PWM. The
// closed-loop controller applies this as dead-zone compensation, then uses Hall
// feedback to coast when measured speed reaches or exceeds the target.
constexpr uint8_t MOTOR_MIN_ACTIVE_PWM = 90U;

// E-stop hardware is not implemented in the current build. Keep this false so
// the input is not configured, the reported state stays normal, and the
// capability is not advertised. A0 is known bad on the current Drive Uno, so
// keep the placeholder away from the pressure inputs.
constexpr bool ENABLE_ESTOP = false;
constexpr uint8_t ESTOP_PIN = 12U;
constexpr uint8_t ESTOP_ACTIVE_LEVEL = HIGH;

// The two FSR channels replace the single digital dead-man switch. Each FSR
// must be wired as a voltage divider that reads near zero when released.
// Temporary hardware-test setting requested for the current build. Pressure is
// still published, but deadmanActive() remains true while this is false.
constexpr bool REQUIRE_DEADMAN = false;
constexpr uint8_t PRESSURE_LEFT_PIN = A1;
constexpr uint8_t PRESSURE_RIGHT_PIN = A2;
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
    MOTOR_MIN_ACTIVE_PWM > 0U && MOTOR_MIN_ACTIVE_PWM <= MAX_PWM,
    "minimum active motor PWM must be positive and no higher than MAX_PWM");
static_assert(
    HALL_PULSES_PER_WHEEL_REV > 0UL,
    "Hall pulses per wheel revolution must be positive");
static_assert(
    !USE_SINGLE_HALL_SENSOR || LEFT_HALL_PIN == 2U || LEFT_HALL_PIN == 3U,
    "single Hall sensor must use an Arduino Uno interrupt pin");
static_assert(
    HALL_MIN_PULSE_INTERVAL_US > 0UL &&
        HALL_ZERO_TIMEOUT_US > HALL_MIN_PULSE_INTERVAL_US,
    "Hall timing limits are invalid");
static_assert(
    COMMAND_WATCHDOG_MAX_MS >= COMMAND_TTL_MIN_MS,
    "command TTL range is invalid");
static_assert(
    TELEMETRY_PERIOD_MS < SESSION_LOSS_TIMEOUT_MS,
    "telemetry period must be shorter than session timeout");
static_assert(
    MOTOR_PWM_PIN == 3U || MOTOR_PWM_PIN == 5U ||
        MOTOR_PWM_PIN == 6U || MOTOR_PWM_PIN == 9U ||
        MOTOR_PWM_PIN == 10U || MOTOR_PWM_PIN == 11U,
    "motor PWM pin must support PWM on Arduino Uno");
static_assert(
    MOTOR_PWM_PIN != MOTOR_IN1_PIN &&
        MOTOR_PWM_PIN != MOTOR_IN2_PIN &&
        MOTOR_IN1_PIN != MOTOR_IN2_PIN,
    "motor control pins must be distinct");
static_assert(
    PRESSURE_LEFT_PIN == A1 && PRESSURE_RIGHT_PIN == A2,
    "current Drive Uno must use A1/A2 because A0 is unavailable");
static_assert(
    PRESSURE_LEFT_PIN != PRESSURE_RIGHT_PIN,
    "pressure sensor pins must be distinct");
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
