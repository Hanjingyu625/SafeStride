// Drive Uno의 핀 배치, 센서 판정값, 제어 게인과 시간 제한을 모은 설정.
// 접미사 US/MS는 시간, MRAD_S는 각속도, PWM은 백분율이 아닌 0~255 count를 뜻한다.
// Pi의 선속도 제한과 이 파일의 MCU 제한은 서로 다른 단계에서 적용된다.

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
// 제어 계산은 200Hz, 상태 송신은 50Hz. 속도 센서의 실제 갱신률은 펄스 간격에 달려 있다.
constexpr uint32_t CONTROL_PERIOD_US = 5000UL;   // 200 Hz
constexpr uint16_t TELEMETRY_PERIOD_MS = 20U;    // 50 Hz
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

// Normal deployment follows the supervised ROS velocity target and closes the
// speed loop with the single installed Hall sensor. The pressure inputs remain
// the physical motion level, but no longer replace /cmd_vel_safe.
// false인 현재 설정은 Pi가 감독한 목표속도와 Hall FF+P를 사용한다.
constexpr bool DEADMAN_DIRECT_DRIVE = false;
// A normal pressure release ramps the current wheel target to zero over this
// interval. E-stop, watchdog and hardware faults still call immediateStop().
constexpr uint16_t DEADMAN_RELEASE_RAMP_MS = 600U;

// WSH135 is a linear analogue Hall sensor on the LEFT wheel. At 5 V its
// no-field output is near 2.5 V. A3 is sampled around that boot-time baseline;
// either magnetic polarity counts once, then must return inside the release
// band before another pulse can be counted. Twelve magnets are fitted.
constexpr uint8_t HALL_ANALOG_PIN = A3;
constexpr uint32_t HALL_SAMPLE_PERIOD_US = CONTROL_PERIOD_US;
constexpr uint8_t HALL_ADC_SAMPLES = 8U;
constexpr uint8_t HALL_BASELINE_SAMPLES = 64U;
constexpr uint16_t HALL_BASELINE_SAMPLE_DELAY_US = 250U;
constexpr int32_t HALL_BASELINE_TRACK_DIVISOR = 128L;
constexpr uint16_t HALL_TRIGGER_DELTA_ADC = 30U;
constexpr uint16_t HALL_RELEASE_DELTA_ADC = 12U;
// Keep the ADC trigger/release hysteresis separate from speed detection.
// With twelve magnets, 8 km/h produces a period of about 27.1 ms.
// The 20 ms glitch window still admits the 25 rad/s plausibility threshold.
// Verify magnetic pulse width and EMI with the actual harness.
constexpr uint32_t HALL_MIN_PULSE_INTERVAL_US = 20000UL;
// 5초는 측정 유효기간이다. 아래 스톨 감시 시간과 역할이 다르다.
constexpr uint32_t HALL_ZERO_TIMEOUT_US = 5000000UL;
constexpr uint32_t HALL_PULSES_PER_WHEEL_REV = 12UL;
constexpr bool HALL_CALIBRATED = true;
// Do not prevent arming solely because calibration has not been certified.
// Closed-loop pulse feedback and Hall stall/overspeed checks remain enabled.
constexpr bool REQUIRE_HALL_CALIBRATION_FOR_ARM = false;

// Temporary no-wheel hardware test. When enabled, either Hall input pulse
// opens a short, low-PWM motor window after an explicit ROS arm request.
// This deliberately bypasses Hall calibration, pressure dead-man, stationary
// dwell, and Hall plausibility checks. Session and command watchdogs remain
// active. Set this false before fitting wheels or beginning normal operation.
constexpr bool MAGNET_BENCH_MODE = false;
constexpr uint8_t MAGNET_BENCH_PWM = 60U;
constexpr uint16_t MAGNET_BENCH_PULSE_HOLD_MS = 750U;
// Hand-moved magnets can take several seconds between approaches. Retain the
// pulse-pair speed long enough to see it in `ros2 topic echo /wheel/hall`.
constexpr uint32_t MAGNET_BENCH_VELOCITY_HOLD_US = 5000000UL;
constexpr float MAGNET_BENCH_VELOCITY_FILTER_ALPHA = 1.0F;

// Runtime Hall plausibility monitor used when DEADMAN_DIRECT_DRIVE is false.
constexpr int32_t HALL_STALL_TARGET_MIN_MRAD_S = 20L;
constexpr uint16_t HALL_STALL_TIMEOUT_MS = 5000U;
// At slow walking speeds one magnet can legitimately take several seconds to
// reach the Hall sensor. Require 2.5 expected pulse periods before declaring a
// stall, while HALL_STALL_TIMEOUT_MS remains the absolute minimum.
constexpr float HALL_STALL_EXPECTED_PULSE_PERIODS = 2.5F;
constexpr uint32_t HALL_STALL_MAX_TIMEOUT_US = 10000000UL;
constexpr int32_t HALL_MAX_PLAUSIBLE_MRAD_S = 25000L;
constexpr uint8_t HALL_OVERSPEED_CONFIRM_PULSES = 2U;

// One SZH-GNP521 drives the two motors as one electrical load. The wiring must
// be checked independently for voltage, polarity and combined stall current.
constexpr uint8_t MOTOR_PWM_PIN = 5U;
constexpr uint8_t MOTOR_IN1_PIN = 6U;
constexpr uint8_t MOTOR_IN2_PIN = 8U;
constexpr int8_t MOTOR_SIGN = 1;
constexpr uint16_t MAX_PWM = 100U;  // deliberately low for first lifted test
// Initial feed-forward model; PWM counts are on Arduino's 0..255 scale.
// 30 is a bias, NOT a minimum output. Calibrate 60 at 0.08 m/s under load.
// 평지 0.08m/s에서 기본 FF=60count(약 23.5% duty). 실측 보정 전 초기 모델이다.
constexpr uint8_t MOTOR_FF_BIAS_PWM = 30U;
constexpr uint8_t MOTOR_FF_NOMINAL_PWM = 60U;
constexpr float MOTOR_NOMINAL_MRAD_S = 0.08F / 0.115F * 1000.0F;
constexpr float MOTOR_PWM_RISE_PER_S = 20.0F;
constexpr float TERRAIN_RECOVERY_PWM_RISE_PER_S = 10.0F;
constexpr float MOTOR_PWM_FALL_PER_S = 60.0F;
constexpr uint32_t MOTOR_REVERSAL_BRAKE_US = 150000UL;
// Target tracking error is handled by the Hall P term. A hard BRAKE is reserved
// for an absolute safety-speed violation confirmed by consecutive Hall periods;
// one low-resolution or noisy period must not stop an otherwise healthy drive.
constexpr float SPEED_BRAKE_ABSOLUTE_MRAD_S = (8.0F / 3.6F) / 0.115F * 1000.0F;
constexpr float SPEED_BRAKE_RELEASE_MRAD_S = (7.0F / 3.6F) / 0.115F * 1000.0F;
constexpr uint8_t SPEED_BRAKE_CONFIRM_PULSES = 2U;

// E-stop hardware is not implemented in the current build. Keep this false so
// the input is not configured, the reported state stays normal, and the
// capability is not advertised. Keep the placeholder away from the analogue
// pressure inputs so A1/A2 can be used for the FSR dead-man circuit.
constexpr bool ENABLE_ESTOP = false;
constexpr uint8_t ESTOP_PIN = 12U;
constexpr uint8_t ESTOP_ACTIVE_LEVEL = HIGH;

// The two FSR channels replace the single digital dead-man switch. Each FSR
// must be wired as a voltage divider that reads near zero when released. The
// installed harness routes the physical left sensor to A2 and right to A1.
// 현재 양손 모두 접촉해야 주행을 허용한다. 핀은 물리적 왼쪽 A2, 오른쪽 A1이다.
constexpr bool REQUIRE_DEADMAN = true;
constexpr uint8_t PRESSURE_LEFT_PIN = A2;
constexpr uint8_t PRESSURE_RIGHT_PIN = A1;
constexpr uint16_t PRESSURE_SAMPLE_PERIOD_MS = 100U;
constexpr float PRESSURE_FILTER_ALPHA = 0.2F;
constexpr uint8_t PRESSURE_ADC_SAMPLES = 8U;
constexpr float PRESSURE_PRESENT_HYSTERESIS = 3.0F;
// Reject one isolated 100 ms ADC dropout while a hand is held. Two
// consecutive below-threshold samples still release the dead-man promptly.
constexpr uint8_t PRESSURE_RELEASE_DEBOUNCE_SAMPLES = 2U;
// Watch /handle/pressure with the motors isolated, then set each channel's
// polarity and threshold halfway between its released and held readings.
constexpr bool PRESSURE_LEFT_ACTIVE_HIGH = true;
constexpr bool PRESSURE_RIGHT_ACTIVE_HIGH = true;
constexpr float PRESSURE_LEFT_PRESENT_THRESHOLD = 80.0F;
constexpr float PRESSURE_RIGHT_PRESENT_THRESHOLD = 80.0F;
constexpr bool PRESSURE_THRESHOLDS_CALIBRATED = true;
constexpr float PRESSURE_IMBALANCE_THRESHOLD = 300.0F;
constexpr float PRESSURE_SUDDEN_CHANGE_THRESHOLD = 150.0F;

// Set to a real input and true only after wiring a driver's fault output.
constexpr bool USE_DRIVER_FAULT_PIN = false;
constexpr uint8_t DRIVER_FAULT_PIN = 13U;
constexpr uint8_t DRIVER_FAULT_ACTIVE_LEVEL = LOW;

// Optional analogue telemetry. Disabled values are sent using protocol
// sentinels and are never interpreted by ROS as real measurements.
// 미사용 센서 값은 무효 표시값으로 전송한다. 측정값 0과 무효값을 구분해야 한다.
constexpr bool ENABLE_BATTERY_SENSE = false;
constexpr uint8_t BATTERY_SENSE_PIN = A5;
constexpr float ADC_REFERENCE_V = 5.0F;
constexpr float BATTERY_DIVIDER_RATIO = 3.0F;

constexpr bool ENABLE_CURRENT_SENSE = false;
constexpr uint8_t LEFT_CURRENT_SENSE_PIN = A0;
constexpr uint8_t RIGHT_CURRENT_SENSE_PIN = A4;
constexpr float CURRENT_ZERO_V = 2.5F;
constexpr float CURRENT_MA_PER_V = 1000.0F;

// The telemetry layout reserves two front-range fields, but no such sensors
// are installed in the current pin map. Do not advertise the capability or
// require the ROS topics until real non-blocking drivers replace the sentinels.
constexpr bool ENABLE_FRONT_RANGE_SENSORS = false;

// 컴파일 시 설정 모순을 검출한다. I/D 활성화에는 별도의 포화 처리와 펄스 시간 설계가 필요하다.
static_assert(MOTOR_FF_NOMINAL_PWM >= MOTOR_FF_BIAS_PWM &&
    MOTOR_FF_NOMINAL_PWM <= MAX_PWM, "feed-forward bounds");

// PID output is PWM counts. Keep both wheels lifted, tune the shared output,
// and keep integrator gain at zero until direction and both Hall channels are
// proven. These example gains are intentionally mild.
constexpr float MOTOR_PID_KP = 12.0F;
constexpr float MOTOR_PID_KI = 0.0F;
constexpr float MOTOR_PID_KD = 0.0F;

static_assert(MOTOR_PID_KI == 0.0F && MOTOR_PID_KD == 0.0F,
    "Implement final-output anti-windup and pulse timing before enabling I/D");
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
    MOTOR_FF_BIAS_PWM > 0U && MOTOR_FF_BIAS_PWM <= MAX_PWM,
    "feed-forward PWM bias must be positive and no higher than MAX_PWM");
static_assert(
    DEADMAN_RELEASE_RAMP_MS > 0U && DEADMAN_RELEASE_RAMP_MS <= 5000U,
    "dead-man release ramp must be between 1 and 5000 ms");
static_assert(
    MAGNET_BENCH_PWM > 0U && MAGNET_BENCH_PWM <= MAX_PWM,
    "magnet bench PWM must be positive and no higher than MAX_PWM");
static_assert(
    MAGNET_BENCH_PULSE_HOLD_MS > 0U &&
        MAGNET_BENCH_PULSE_HOLD_MS <= 1000U,
    "magnet bench pulse hold must be between 1 and 1000 ms");
static_assert(
    MAGNET_BENCH_VELOCITY_HOLD_US >= HALL_ZERO_TIMEOUT_US,
    "magnet bench velocity hold must cover the normal Hall timeout");
static_assert(
    MAGNET_BENCH_VELOCITY_FILTER_ALPHA > 0.0F &&
        MAGNET_BENCH_VELOCITY_FILTER_ALPHA <= 1.0F &&
        VELOCITY_FILTER_ALPHA > 0.0F &&
        VELOCITY_FILTER_ALPHA <= 1.0F,
    "velocity filter alpha values must be in (0, 1]");
static_assert(
    HALL_PULSES_PER_WHEEL_REV > 0UL,
    "Hall pulses per wheel revolution must be positive");
static_assert(
    HALL_MIN_PULSE_INTERVAL_US > 0UL &&
        HALL_ZERO_TIMEOUT_US > HALL_MIN_PULSE_INTERVAL_US,
    "Hall timing limits are invalid");
static_assert(
    HALL_ADC_SAMPLES > 0U && HALL_BASELINE_SAMPLES > 0U &&
        HALL_SAMPLE_PERIOD_US > 0UL &&
        HALL_BASELINE_TRACK_DIVISOR > 0L,
    "analogue Hall sampling configuration is invalid");
static_assert(
    HALL_TRIGGER_DELTA_ADC > HALL_RELEASE_DELTA_ADC &&
        HALL_TRIGGER_DELTA_ADC <= 1023U,
    "analogue Hall thresholds require trigger/release hysteresis");
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
    HALL_STALL_TIMEOUT_MS > 0U,
    "Hall stall timeout must be positive");
static_assert(
    HALL_OVERSPEED_CONFIRM_PULSES >= 2U,
    "Hall overspeed fault must require consecutive observations");
static_assert(
    HALL_STALL_EXPECTED_PULSE_PERIODS >= 1.0F,
    "Hall stall detection must allow at least one expected pulse period");
static_assert(
    HALL_MAX_PLAUSIBLE_MRAD_S >
        MAX_WHEEL_TARGET_MRAD_S,
    "Hall plausible speed must exceed maximum target");
static_assert(
    SPEED_BRAKE_CONFIRM_PULSES >= 2U,
    "Absolute speed BRAKE must require consecutive Hall observations");
static_assert(
    AVR_BOOT_COUNTER_EEPROM_ADDRESS >= 0,
    "boot-counter EEPROM address must not be negative");
static_assert(
    PRESSURE_LEFT_PRESENT_THRESHOLD >= 0.0F &&
        PRESSURE_LEFT_PRESENT_THRESHOLD <= 1023.0F &&
        PRESSURE_RIGHT_PRESENT_THRESHOLD >= 0.0F &&
        PRESSURE_RIGHT_PRESENT_THRESHOLD <= 1023.0F,
    "pressure thresholds must fit the ADC range");
static_assert(
    PRESSURE_ADC_SAMPLES > 0U,
    "pressure ADC averaging requires at least one sample");
static_assert(
    PRESSURE_RELEASE_DEBOUNCE_SAMPLES > 0U,
    "pressure release debounce requires at least one sample");
static_assert(
    PRESSURE_PRESENT_HYSTERESIS >= 0.0F &&
        PRESSURE_PRESENT_HYSTERESIS <
            PRESSURE_LEFT_PRESENT_THRESHOLD &&
        PRESSURE_PRESENT_HYSTERESIS <
            PRESSURE_RIGHT_PRESENT_THRESHOLD,
    "pressure hysteresis must be below both thresholds");
static_assert(
    PRESSURE_LEFT_PIN == A2 && PRESSURE_RIGHT_PIN == A1,
    "Drive pressure channels must match the installed A2/A1 harness");
static_assert(
    PRESSURE_LEFT_PIN != PRESSURE_RIGHT_PIN &&
        PRESSURE_LEFT_PIN != HALL_ANALOG_PIN &&
        PRESSURE_RIGHT_PIN != HALL_ANALOG_PIN &&
        PRESSURE_LEFT_PIN != ESTOP_PIN &&
        PRESSURE_RIGHT_PIN != ESTOP_PIN,
    "Drive pressure, Hall, and E-stop pins must be distinct");

}  // namespace safestride_config
