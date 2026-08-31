#include <assert.h>
#include <stdio.h>

#include "tof10120_sensor.h"
#include <Wire.h>

HardwareSerial Serial;
TwoWire Wire;

namespace {
uint32_t g_now_ms = 0UL;
uint16_t g_distance_mm = 250U;
uint8_t g_wire_byte_index = 0U;

void sample(Tof10120Sensor& tof, uint16_t distance, int count = 1) {
  g_distance_mm = distance;
  for (int index = 0; index < count; ++index) {
    g_now_ms += 50UL;
    tof.update(g_now_ms);
  }
}

void establishBaseline(Tof10120Sensor& tof) {
  tof.begin(g_now_ms);
  tof.update(g_now_ms);
  sample(tof, 250U, 9);
  assert(tof.valid());
  assert(tof.alert() == TofAlert::NORMAL);
}
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
void delayMicroseconds(unsigned int) {}

void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t) { return 1U; }
size_t HardwareSerial::write(const uint8_t*, size_t length) { return length; }
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
  {
    Tof10120Sensor tof;
    establishBaseline(tof);
    // One raw spike must decay without reaching the four-frame decision.
    sample(tof, 650U);
    sample(tof, 250U, 4);
    assert(tof.alert() != TofAlert::DROP);
    assert(tof.alert() != TofAlert::RAISED);
  }

  {
    Tof10120Sensor tof;
    g_distance_mm = 250U;
    establishBaseline(tof);
    sample(tof, 650U, 6);
    assert(tof.alert() == TofAlert::DROP);
  }

  {
    Tof10120Sensor tof;
    g_distance_mm = 250U;
    establishBaseline(tof);
    sample(tof, 120U, 8);
    assert(tof.alert() == TofAlert::RAISED);
    sample(tof, 250U, 10);
    assert(tof.alert() == TofAlert::RAISED);
    sample(tof, 250U, 25);
    assert(tof.alert() == TofAlert::NORMAL);
  }

  printf("firmware TOF-10120 adaptive hazard tests: OK\n");
  return 0;
}
