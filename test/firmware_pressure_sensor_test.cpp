#include <assert.h>
#include <stdio.h>

#include "config.h"
#include "pressure_sensor.h"

namespace cfg = safestride_config;

HardwareSerial Serial;

namespace {

uint32_t g_now_ms = 0UL;
int g_left_raw = 0;
int g_right_raw = 0;

}  // namespace

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return LOW; }
void analogWrite(uint8_t, int) {}
int analogRead(uint8_t pin) {
  return pin == cfg::PRESSURE_LEFT_PIN ? g_left_raw : g_right_raw;
}
int digitalPinToInterrupt(uint8_t) { return 0; }
void attachInterrupt(int, void (*)(), int) {}
void noInterrupts() {}
void interrupts() {}
uint32_t millis() { return g_now_ms; }
uint32_t micros() { return g_now_ms * 1000UL; }

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
  PressureSensorPair pressure;
  g_left_raw = 500;
  g_right_raw = 500;
  pressure.begin(g_now_ms);
  assert(pressure.initialized());
  assert(pressure.bothHandsPresent());
  assert(pressure.leftPresent());
  assert(pressure.rightPresent());
  assert(pressure.leftRaw() == 500U);
  assert(pressure.rightRaw() == 500U);
  assert(pressure.leftFiltered() == 500.0F);
  assert(pressure.rightFiltered() == 500.0F);
  assert(
      pressure.calibrated() == cfg::PRESSURE_THRESHOLDS_CALIBRATED);
  assert(pressure.alert() == PressureAlert::NORMAL);

  g_left_raw = 1000;
  g_right_raw = 500;
  for (int i = 0; i < 6; ++i) {
    g_now_ms += cfg::PRESSURE_SAMPLE_PERIOD_MS;
    pressure.update(g_now_ms);
  }
  assert(pressure.bothHandsPresent());
  assert(pressure.alert() == PressureAlert::WARNING);

  g_left_raw = 0;
  g_now_ms += cfg::PRESSURE_SAMPLE_PERIOD_MS;
  pressure.update(g_now_ms);
  // One isolated low ADC sample is rejected while the handle is held.
  assert(pressure.bothHandsPresent());
  g_now_ms += cfg::PRESSURE_SAMPLE_PERIOD_MS;
  pressure.update(g_now_ms);
  // Two consecutive low samples confirm release.
  assert(!pressure.bothHandsPresent());
  assert(pressure.alert() == PressureAlert::HANDS_OFF);

  // The low-pass filter remains above the presence threshold after release.
  // It must not re-arm the channel while the live ADC value remains low.
  for (int i = 0; i < 6; ++i) {
    g_now_ms += cfg::PRESSURE_SAMPLE_PERIOD_MS;
    pressure.update(g_now_ms);
    assert(!pressure.leftPresent());
    assert(!pressure.bothHandsPresent());
  }

  // The published raw channel follows the ADC immediately while the filtered
  // channel is intentionally smoothed. A real raw crossing can reacquire the
  // channel once both values are above the configured threshold.
  g_left_raw = 512;
  g_now_ms += cfg::PRESSURE_SAMPLE_PERIOD_MS;
  pressure.update(g_now_ms);
  assert(pressure.leftRaw() == 512U);
  assert(pressure.leftFiltered() > cfg::PRESSURE_LEFT_PRESENT_THRESHOLD);
  assert(pressure.leftPresent());
  assert(pressure.bothHandsPresent());

  printf("firmware pressure-sensor tests: OK\n");
  return 0;
}
