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
    assert(g_motor_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);
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
    for (int i = 0; i < 720; ++i) {
      drive.update(5000UL, stopped, stopped, 3000L, true);
    }
    assert(drive.hallFaultMask() == DriveController::HALL_FAULT_LEFT);
    assert(g_motor_pwm == 0);
    assert(g_motor_in1_level == LOW);
    assert(g_motor_in2_level == LOW);
    drive.clearRecoverableFaults();
    assert(drive.hallFaultMask() == 0U);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    // At 0.5 rad/s a six-magnet wheel takes about 2.1 s per pulse. The fixed
    // 3 s limit would falsely trip before two legitimate pulse periods.
    for (int i = 0; i < 720; ++i) {
      drive.update(5000UL, stopped, stopped, 500L, true, true);
    }
    assert(drive.hallFaultMask() == 0U);
    for (int i = 0; i < 400; ++i) {
      drive.update(5000UL, stopped, stopped, 500L, true, true);
    }
    assert(drive.hallFaultMask() == DriveController::HALL_FAULT_LEFT);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 100; ++i) {
      drive.update(5000UL, stopped, stopped, 500L, true, true);
    }
    assert(drive.appliedTargetMradS() == 500L);
    const int driving_pwm = g_motor_pwm;
    assert(driving_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);
    for (int i = 0; i < 60; ++i) {
      drive.update(
          5000UL, stopped, stopped, 0L, true, true, 834UL, true);
    }
    assert(g_motor_pwm >= driving_pwm / 2 - 1);
    assert(g_motor_pwm <= driving_pwm / 2 + 1);
    for (int i = 60; i < 119; ++i) {
      drive.update(
          5000UL, stopped, stopped, 0L, true, true, 834UL, true);
    }
    assert(drive.appliedTargetMradS() > 0L);
    drive.update(
        5000UL, stopped, stopped, 0L, true, true, 834UL, true);
    assert(drive.appliedTargetMradS() == 0L);
    assert(g_motor_pwm == 0);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 100; ++i) {
      drive.update(5000UL, stopped, stopped, 500L, true, true);
    }
    assert(g_motor_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);

    // A slow command can briefly overshoot after startup. Closed-loop
    // correction may reduce torque, but a non-zero target must retain the
    // measured minimum sustaining PWM.
    const HallSample moving = sample(1UL, 500000UL);
    for (int i = 0; i < 10; ++i) {
      drive.update(5000UL, moving, moving, 500L, true, true);
    }
    assert(g_motor_pwm == cfg::MOTOR_MIN_ACTIVE_PWM);
    for (int i = 0; i < 720; ++i) {
      drive.update(5000UL, moving, moving, 500L, true, true);
    }
    assert(drive.hallFaultMask() == 0U);
    assert(g_motor_pwm == cfg::MOTOR_MIN_ACTIVE_PWM);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 100; ++i) {
      drive.update(5000UL, stopped, stopped, 500L, true, true);
    }
    assert(g_motor_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);

    const HallSample first_pulse = {1UL, 0UL, 0UL};
    drive.update(5000UL, first_pulse, first_pulse, 500L, true, true);
    assert(drive.leftVelocityMradS() == 0L);
    assert(g_motor_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);
    assert(drive.hallFaultMask() == 0U);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 100; ++i) {
      drive.update(5000UL, stopped, stopped, 500L, true, true);
    }
    const HallSample overspeed = sample(1UL, 500000UL);
    for (int i = 0; i < 10; ++i) {
      drive.update(5000UL, overspeed, overspeed, 500L, true, true);
    }
    const int closed_loop_pwm = g_motor_pwm;
    assert(closed_loop_pwm == cfg::MOTOR_MIN_ACTIVE_PWM);
    drive.update(
        5000UL, overspeed, overspeed, 0L, true, true, 10UL, true);
    // Releasing the dead-man fades from the actual closed-loop output and
    // never jumps back to the breakaway PWM.
    assert(g_motor_pwm > 0);
    assert(g_motor_pwm <= closed_loop_pwm);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 720; ++i) {
      drive.update(5000UL, stopped, stopped, 667L, true, false);
    }
    assert(drive.hallFaultMask() == 0U);
    const int expected_open_loop_pwm =
        static_cast<int>(cfg::MOTOR_MIN_ACTIVE_PWM) +
        (667 * static_cast<int>(
                   cfg::MAX_PWM - cfg::MOTOR_MIN_ACTIVE_PWM) +
         cfg::MAX_WHEEL_TARGET_MRAD_S / 2) /
            cfg::MAX_WHEEL_TARGET_MRAD_S;
    assert(g_motor_pwm == expected_open_loop_pwm);
    assert(g_motor_in1_level == HIGH);
    assert(g_motor_in2_level == LOW);
  }

  {
    DriveController drive;
    drive.begin();
    primeFeedback(drive);
    uint32_t right_count = 0UL;
    const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
    for (int i = 0; i < 720; ++i) {
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
      const HallSample chatter = sample(
          count, cfg::HALL_MIN_PULSE_INTERVAL_US - 1UL);
      drive.update(5000UL, chatter, chatter, 500L, true);
    }
    // Periods rejected by the analogue input filter cannot become speed
    // feedback or a latched Hall fault in the motor controller.
    assert(drive.leftVelocityMradS() == 0L);
    assert(drive.hallFaultMask() == 0U);
    assert(g_motor_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);
  }

  {
    // Exercise closed-loop correction across both directions, the complete
    // requested-speed range and slow/fast Hall periods. While the ramped
    // target requests motion, no active output may fall below PWM 80.
    const int32_t targets[] = {
        20L, 100L, 500L, 1000L, 2000L, 3000L,
        -20L, -100L, -500L, -1000L, -2000L, -3000L};
    const uint32_t periods[] = {
        cfg::HALL_MIN_PULSE_INTERVAL_US, 500000UL,
        1500000UL, 3000000UL};
    for (size_t target_index = 0U;
         target_index < sizeof(targets) / sizeof(targets[0]);
         ++target_index) {
      for (size_t period_index = 0U;
           period_index < sizeof(periods) / sizeof(periods[0]);
           ++period_index) {
        DriveController drive;
        drive.begin();
        primeFeedback(drive);
        uint32_t pulse_count = 0UL;
        for (int step = 0; step < 160; ++step) {
          const HallSample moving =
              sample(++pulse_count, periods[period_index]);
          drive.update(
              5000UL, moving, moving, targets[target_index], true, true);
          const int32_t applied = drive.appliedTargetMradS();
          if (applied >= 20L || applied <= -20L) {
            assert(g_motor_pwm >= cfg::MOTOR_MIN_ACTIVE_PWM);
            assert(g_motor_pwm <= cfg::MAX_PWM);
            if (applied > 0L) {
              assert(g_motor_in1_level == HIGH);
              assert(g_motor_in2_level == LOW);
            } else {
              assert(g_motor_in1_level == LOW);
              assert(g_motor_in2_level == HIGH);
            }
          }
          assert(drive.hallFaultMask() == 0U);
        }
      }
    }
  }

  printf("firmware Hall feedback and single-driver tests: OK\n");
  return 0;
}
