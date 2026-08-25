#include <assert.h>
#include <stdio.h>

#include "config.h"
#include "motor_control.h"

namespace cfg = safestride_config;

HardwareSerial Serial;

namespace {

int g_motor_pwm = 0;
uint8_t g_motor_in1_level = LOW;
uint8_t g_motor_in2_level = LOW;

}  // namespace

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin == cfg::MOTOR_IN1_PIN) {
    g_motor_in1_level = value;
  } else if (pin == cfg::MOTOR_IN2_PIN) {
    g_motor_in2_level = value;
  }
}

int digitalRead(uint8_t) { return LOW; }

void analogWrite(uint8_t pin, int value) {
  if (pin == cfg::MOTOR_PWM_PIN) {
    g_motor_pwm = value;
  }
}

int analogRead(uint8_t) { return 0; }
int digitalPinToInterrupt(uint8_t) { return 0; }
void attachInterrupt(int, void (*)(), int) {}
void noInterrupts() {}
void interrupts() {}
uint32_t millis() { return 0UL; }
uint32_t micros() { return 0UL; }

void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t) { return 1U; }
size_t HardwareSerial::write(const uint8_t*, size_t length) {
  return length;
}
size_t HardwareSerial::print(const char*) { return 1U; }
size_t HardwareSerial::println(const char*) { return 1U; }

int main() {
  static_assert(cfg::ENABLE_HALL_FEEDBACK, "test expects Hall feedback");
  static_assert(
      cfg::USE_SINGLE_HALL_SENSOR,
      "test expects one left-wheel Hall channel");
  HallSample hall = {0UL, 1000000UL, 100000UL};
  DriveController drive;
  drive.begin();

  drive.update(5000UL, hall, hall, 1500L, false);
  assert(g_motor_pwm == 0);
  assert(g_motor_in1_level == LOW);
  assert(g_motor_in2_level == LOW);

  // A one-second period at six pulses/revolution is about 1.047 rad/s.
  // Two disabled updates establish feedback without ever energizing the motor.
  ++hall.pulse_count;
  drive.update(5000UL, hall, hall, 0L, false);
  assert(drive.feedbackReady());
  assert(drive.leftHallPulsePosition() == 1L);
  assert(drive.rightHallPulsePosition() == 1L);
  assert(drive.leftVelocityMradS() > 500L);
  assert(drive.leftVelocityMradS() < 800L);

  // With no measured overspeed, the closed-loop request ramps up and applies
  // the configured low bench PWM after the controller crosses its dead zone.
  hall.age_us = cfg::HALL_ZERO_TIMEOUT_US;
  for (uint16_t i = 0U; i < 200U; ++i) {
    drive.update(5000UL, hall, hall, 1500L, true);
  }
  assert(drive.appliedTargetMradS() == 1200L);
  assert(g_motor_pwm == cfg::MOTOR_MIN_ACTIVE_PWM);
  assert(g_motor_in1_level == HIGH);
  assert(g_motor_in2_level == LOW);
  assert(drive.hallFaultMask() == 0U);

  for (uint16_t i = 0U; i < 100U; ++i) {
    drive.update(5000UL, hall, hall, 0L, true);
  }
  assert(drive.appliedTargetMradS() == 0L);
  assert(g_motor_pwm == 0);
  assert(g_motor_in1_level == LOW);
  assert(g_motor_in2_level == LOW);
  assert(drive.leftVelocityMradS() == 0L);
  assert(drive.rightVelocityMradS() == 0L);
  assert(drive.feedbackReady());

  // Pulses arriving well inside the four-second window keep the installed
  // left Hall monitor healthy during a sustained command.
  DriveController healthy_drive;
  healthy_drive.begin();
  HallSample healthy = {0UL, 1500000UL, 0UL};
  healthy_drive.update(
      5000UL, healthy, healthy, 0L, false);
  for (uint16_t i = 0U; i < 1200U; ++i) {
    healthy.age_us += 5000UL;
    if (i % 300U == 0U) {
      ++healthy.pulse_count;
      healthy.age_us = 0UL;
    }
    healthy_drive.update(
        5000UL, healthy, healthy, 1500L, true);
  }
  assert(healthy_drive.hallFaultMask() == 0U);

  // A commanded wheel with no pulse latches the installed left channel fault.
  DriveController stalled_drive;
  stalled_drive.begin();
  HallSample stalled = {0UL, 0UL, 0xFFFFFFFFUL};
  stalled_drive.update(5000UL, stalled, stalled, 0L, false);
  for (uint16_t i = 0U; i < 1000U; ++i) {
    stalled_drive.update(5000UL, stalled, stalled, 1500L, true);
  }
  assert(
      stalled_drive.hallFaultMask() == DriveController::HALL_FAULT_LEFT);

  printf("firmware single-Hall single-driver tests: OK\n");
  return 0;
}
