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

HallSample sample(uint32_t pulses, uint32_t period_us = 6283185UL) {
  HallSample value = {pulses, period_us, 0UL};
  return value;
}

void primeFeedback(DriveController& drive) {
  const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
  drive.update(5000UL, stopped, stopped, 0L, false);
  drive.update(5000UL, stopped, stopped, 0L, false);
  assert(drive.feedbackReady());
}

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
  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    for (uint32_t count = 1UL; count <= 220UL; ++count) {
      const HallSample moving = sample(count);
      drive.update(5000UL, moving, moving, 1000L, true);
    }
    assert(drive.hallFaultMask() == 0U);
    assert(g_motor_pwm > 0);
    assert(g_motor_in1_level == HIGH);
    assert(g_motor_in2_level == LOW);
    assert(drive.leftHallPulsePosition() == 220L);
    assert(drive.rightHallPulsePosition() == 220L);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 420; ++i) {
      drive.update(5000UL, stopped, stopped, 3000L, true);
    }
    assert(
        drive.hallFaultMask() ==
        (DriveController::HALL_FAULT_LEFT |
         DriveController::HALL_FAULT_RIGHT));
    assert(g_motor_pwm == 0);
    assert(g_motor_in1_level == LOW);
    assert(g_motor_in2_level == LOW);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    uint32_t right_count = 0UL;
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 420; ++i) {
      const HallSample right = sample(++right_count);
      drive.update(5000UL, stopped, right, 3000L, true);
    }
    assert(
        drive.hallFaultMask() == DriveController::HALL_FAULT_LEFT);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    for (uint32_t count = 1UL; count <= 40UL; ++count) {
      const HallSample too_fast = sample(count, 1000UL);
      drive.update(5000UL, too_fast, too_fast, 0L, true);
    }
    assert(
        drive.hallFaultMask() ==
        (DriveController::HALL_FAULT_LEFT |
         DriveController::HALL_FAULT_RIGHT));
  }

  {
    DriveController drive;
    drive.begin();
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    const HallSample magnet_pulse = sample(1UL);
    drive.updateMagnetBench(
        5000UL, magnet_pulse, stopped, 500L, true);
    assert(drive.hallFaultMask() == 0U);
    assert(g_motor_pwm == cfg::MAGNET_BENCH_PWM);
    assert(g_motor_in1_level == HIGH);
    assert(g_motor_in2_level == LOW);
    assert(drive.leftHallPulsePosition() == 0L);
    assert(drive.leftVelocityMradS() >= 999L);
    assert(drive.leftVelocityMradS() <= 1001L);

    drive.updateMagnetBench(
        5000UL, magnet_pulse, stopped, 500L, false);
    assert(g_motor_pwm == 0);
    assert(g_motor_in1_level == LOW);
    assert(g_motor_in2_level == LOW);

    const HallSample visible_for_echo = {
        1UL, 6283185UL, 4000000UL};
    drive.updateMagnetBench(
        5000UL, visible_for_echo, stopped, 500L, false);
    assert(drive.leftVelocityMradS() >= 999L);
    const HallSample expired = {
        1UL, 6283185UL, cfg::MAGNET_BENCH_VELOCITY_HOLD_US};
    drive.updateMagnetBench(
        5000UL, expired, stopped, 500L, false);
    assert(drive.leftVelocityMradS() == 0L);

    drive.updateMagnetBench(
        5000UL, magnet_pulse, stopped, -500L, true);
    assert(g_motor_pwm == cfg::MAGNET_BENCH_PWM);
    assert(g_motor_in1_level == LOW);
    assert(g_motor_in2_level == HIGH);
  }

  printf("firmware Hall feedback and single-driver tests: OK\n");
  return 0;
}
