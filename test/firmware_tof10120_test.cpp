#include <assert.h>
#include <stdio.h>

#include "tof10120_sensor.h"
#include <Wire.h>

HardwareSerial Serial;
TwoWire Wire;

namespace {

uint32_t g_now_ms = 0UL;
uint16_t g_distance_mm = 500U;
uint8_t g_wire_byte_index = 0U;

}  // namespace

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return LOW; }
void analogWrite(uint8_t, int) {}
int analogRead(uint8_t) { return 0; }
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

void TwoWire::begin() {}
void TwoWire::beginTransmission(uint8_t) {}
size_t TwoWire::write(uint8_t) { return 1U; }
uint8_t TwoWire::endTransmission() { return 0U; }
uint8_t TwoWire::requestFrom(uint8_t, uint8_t quantity) {
  g_wire_byte_index = 0U;
  return quantity;
}
int TwoWire::available() { return 2 - g_wire_byte_index; }
int TwoWire::read() {
  if (g_wire_byte_index++ == 0U) {
    return static_cast<int>((g_distance_mm >> 8U) & 0xFFU);
  }
  return static_cast<int>(g_distance_mm & 0xFFU);
}

int main() {
  Tof10120Sensor tof;
  tof.begin(g_now_ms);
  tof.update(g_now_ms);
  assert(tof.valid());
  assert(tof.distanceMm() == 500U);
  assert(tof.alert() == TofAlert::NORMAL);

  // A sustained larger distance must eventually satisfy both the filtered
  // error and positive-change checks for four consecutive frames.
  g_distance_mm = 900U;
  for (int i = 0; i < 8; ++i) {
    g_now_ms += 50UL;
    tof.update(g_now_ms);
  }
  assert(tof.alert() == TofAlert::STEP);

  // The red state is retained for one second after the last confirmation.
  g_distance_mm = 500U;
  g_now_ms += 500UL;
  tof.update(g_now_ms);
  assert(tof.alert() == TofAlert::STEP);
  g_now_ms += 1000UL;
  tof.update(g_now_ms);
  assert(tof.alert() != TofAlert::STEP);

  printf("firmware TOF-10120 tests: OK\n");
  return 0;
}
