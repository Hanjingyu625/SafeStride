#include <assert.h>
#include <stdio.h>
#include "../firmware/safestride_mcu/digital_hall_sensor.h"
#include "../firmware/safestride_mcu/motor_control.h"
#include "../firmware/safestride_mcu/config.h"

namespace cfg = safestride_config;
namespace {
uint32_t g_now = 0UL;
int g_level = HIGH;
void (*g_irq)() = nullptr;
bool g_interrupts = true;
bool g_pullup = false;

void edge(uint32_t now, int level) {
  g_now = now;
  if (g_level != level) {
    g_level = level;
    assert(g_interrupts && g_irq != nullptr);
    g_irq();
  }
}

void snapshot(DigitalHallSensor& hall, uint32_t now) {
  g_now = now;
  hall.update(now);
  assert(g_interrupts);
}
}

HardwareSerial Serial;
void pinMode(uint8_t pin, uint8_t mode) {
  if (pin == cfg::HALL_DIGITAL_PIN) {
    assert(mode == INPUT_PULLUP);
    g_pullup = true;
  }
}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t pin) { assert(pin == cfg::HALL_DIGITAL_PIN); return g_level; }
void analogWrite(uint8_t, int) {}
int analogRead(uint8_t) { assert(false); return 0; }
int digitalPinToInterrupt(uint8_t pin) { assert(pin == 2U); return 0; }
void attachInterrupt(int number, void (*callback)(), int mode) {
  assert(number == 0 && mode == CHANGE);
  g_irq = callback;
}
void noInterrupts() { g_interrupts = false; }
void interrupts() { g_interrupts = true; }
uint32_t millis() { return g_now / 1000UL; }
uint32_t micros() { return g_now; }
void delayMicroseconds(unsigned int) {}
void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t) { return 1U; }
size_t HardwareSerial::write(const uint8_t*, size_t n) { return n; }
size_t HardwareSerial::print(const char*) { return 1U; }
size_t HardwareSerial::println(const char*) { return 1U; }

int main() {
  DigitalHallSensor hall;
  hall.begin(0UL);
  assert(g_pullup && hall.pulseCount() == 0UL && !hall.magnetPresent());
  assert(hall.ageUs(0UL) == 0xFFFFFFFFUL);

  // A real edge at micros()==0 is a valid first pulse, not the no-pulse sentinel.
  edge(0UL, LOW);
  snapshot(hall, 1000UL);
  assert(hall.pulseCount() == 1UL && hall.periodUs() == 0UL);
  assert(hall.ageUs(1000UL) == 1000UL && hall.magnetPresent());
  snapshot(hall, 5000UL);
  assert(hall.pulseCount() == 1UL);
  edge(5100UL, HIGH);
  edge(5200UL, LOW);  // Reject chatter inside the 20 ms interval.
  snapshot(hall, 6000UL);
  assert(hall.pulseCount() == 1UL && hall.periodUs() == 0UL);
  assert(hall.ageUs(6000UL) == 6000UL);
  edge(7000UL, HIGH);
  edge(20000UL, LOW);
  edge(20100UL, HIGH);  // Pulse ends before the next 5 ms control tick.
  snapshot(hall, 25000UL);
  assert(hall.pulseCount() == 2UL && hall.periodUs() == 20000UL);
  assert(!hall.magnetPresent() && hall.ageUs(25000UL) == 5000UL);

  // Booting under a stationary magnet does not create a rotation event.
  g_level = LOW;
  hall.begin(30000UL);
  snapshot(hall, 35000UL);
  assert(hall.pulseCount() == 0UL && hall.magnetPresent());
  assert(hall.ageUs(35000UL) == 0xFFFFFFFFUL);
  edge(40000UL, HIGH);
  snapshot(hall, 45000UL);
  assert(hall.pulseCount() == 0UL);
  edge(50000UL, LOW);
  snapshot(hall, 51000UL);
  assert(hall.pulseCount() == 1UL && hall.periodUs() == 0UL);

  // Interrupt occurs after the loop timestamp: age must not underflow to stale.
  edge(70000UL, HIGH);
  edge(80001UL, LOW);
  g_now = 80002UL;
  hall.update(80000UL);
  assert(hall.ageUs(80000UL) == 1UL);
  assert(hall.periodUs() == 30001UL);

  g_level = HIGH;
  hall.begin(0xFFFF0000UL);
  edge(0xFFFFF000UL, LOW);
  edge(0xFFFFF100UL, HIGH);
  edge(0x00004000UL, LOW);
  snapshot(hall, 0x00004100UL);
  assert(hall.pulseCount() == 2UL && hall.periodUs() == 0x5000UL);
  assert(hall.ageUs(0x4100UL) == 0x100UL);
  // Aging saturates instead of making a stopped wheel look fresh after wrap.
  snapshot(hall, 0x70004100UL);
  snapshot(hall, 0xE0004100UL);
  snapshot(hall, 0x50004100UL);
  assert(hall.ageUs(0x50004100UL) == 0xFFFFFFFEUL);
  snapshot(hall, 0x50005100UL);
  assert(hall.ageUs(0x50005100UL) == 0xFFFFFFFEUL);

  // Short pulses between ticks still reach the production overspeed path.
  g_level = HIGH;
  hall.begin(0UL);
  DriveController drive;
  drive.begin();
  for (uint32_t t = 5000UL; t <= 1000000UL; t += 5000UL) {
    if (t % 20000UL == 0UL) {
      edge(t - 200UL, LOW);
      edge(t - 100UL, HIGH);
    }
    snapshot(hall, t);
    HallSample sample = {hall.pulseCount(), hall.periodUs(), hall.ageUs(t)};
    drive.update(5000UL, sample, sample, 696L, true);
  }
  assert(hall.pulseCount() == 50UL && hall.periodUs() == 20000UL);
  assert(drive.speedValid());
  assert(drive.braking() || drive.hallFaultMask() != 0U);
  printf("digital A314x Hall and overspeed input-path tests: OK\n");
  return 0;
}
