#include <assert.h>
#include <stdio.h>

#include "config.h"
#include "motor_control.h"

namespace cfg = safestride_config;

HardwareSerial Serial;

namespace {

uint8_t g_enable_level = cfg::MOTOR_DRIVER_ENABLE_INACTIVE_LEVEL;
int g_left_pwm = 0;
int g_right_pwm = 0;

void primeFeedback(DriveController& drive) {
  drive.update(5000UL, 0UL, 0UL, 0L, 0L, false);
  drive.update(5000UL, 0UL, 0UL, 0L, 0L, false);
  assert(drive.feedbackReady());
}

}  // namespace

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin == cfg::MOTOR_DRIVER_ENABLE_PIN) {
    g_enable_level = value;
  }
}

int digitalRead(uint8_t) { return LOW; }

void analogWrite(uint8_t pin, int value) {
  if (pin == cfg::LEFT_MOTOR_PWM_PIN) {
    g_left_pwm = value;
  } else if (pin == cfg::RIGHT_MOTOR_PWM_PIN) {
    g_right_pwm = value;
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

int main() {
  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    uint32_t count = 0UL;
    for (int i = 0; i < 220; ++i) {
      ++count;
      drive.update(5000UL, count, count, 1000L, 1000L, true);
    }
    assert(drive.encoderFaultMask() == 0U);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    for (int i = 0; i < 220; ++i) {
      drive.update(5000UL, 0UL, 0UL, 3000L, 3000L, true);
    }
    assert(
        drive.encoderFaultMask() ==
        (DriveController::ENCODER_FAULT_LEFT |
         DriveController::ENCODER_FAULT_RIGHT));
    assert(g_enable_level == cfg::MOTOR_DRIVER_ENABLE_INACTIVE_LEVEL);
    assert(g_left_pwm == 0);
    assert(g_right_pwm == 0);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    uint32_t reverse_count = 0UL;
    for (int i = 0; i < 120; ++i) {
      --reverse_count;
      drive.update(
          5000UL,
          reverse_count,
          0UL,
          1000L,
          0L,
          true);
    }
    assert(
        drive.encoderFaultMask() ==
        DriveController::ENCODER_FAULT_LEFT);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    uint32_t count = 0UL;
    for (int i = 0; i < 40; ++i) {
      count += 10UL;
      drive.update(5000UL, count, count, 0L, 0L, true);
    }
    assert(
        drive.encoderFaultMask() ==
        (DriveController::ENCODER_FAULT_LEFT |
         DriveController::ENCODER_FAULT_RIGHT));
  }

  printf("firmware motor-control plausibility tests: OK\n");
  return 0;
}
