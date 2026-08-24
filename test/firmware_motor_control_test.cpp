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
  static_assert(!cfg::ENABLE_ENCODER_FEEDBACK, "test expects open-loop mode");
  static_assert(cfg::ALLOW_OPEN_LOOP_MOTOR, "test expects bench open-loop mode");
  const WheelEncoderSample unavailable_encoder = {0L, 0L, 0L, 0L, false};
  DriveController drive;
  drive.begin();

  drive.update(5000UL, unavailable_encoder, 1500L, false);
  assert(g_motor_pwm == 0);
  assert(g_motor_in1_level == LOW);
  assert(g_motor_in2_level == LOW);

  drive.update(5000UL, unavailable_encoder, 1500L, true);
  assert(drive.appliedTargetMradS() == 1500L);
  assert(g_motor_pwm == 95);
  assert(g_motor_in1_level == HIGH);
  assert(g_motor_in2_level == LOW);

  for (uint32_t i = 0UL; i < 10000UL; ++i) {
    drive.update(5000UL, unavailable_encoder, 3000L, true);
  }
  assert(drive.encoderFaultMask() == 0U);
  assert(g_motor_pwm == cfg::MAX_PWM);
  assert(g_motor_in1_level == HIGH);
  assert(g_motor_in2_level == LOW);

  drive.update(5000UL, unavailable_encoder, -1500L, true);
  assert(drive.appliedTargetMradS() == -1500L);
  assert(g_motor_pwm == 95);
  assert(g_motor_in1_level == LOW);
  assert(g_motor_in2_level == HIGH);

  drive.update(5000UL, unavailable_encoder, 0L, true);
  assert(g_motor_pwm == 0);
  assert(g_motor_in1_level == LOW);
  assert(g_motor_in2_level == LOW);
  assert(drive.leftVelocityMradS() == 0L);
  assert(drive.rightVelocityMradS() == 0L);
  assert(!drive.feedbackReady());

  printf("firmware open-loop single-driver tests: OK\n");
  return 0;
}
