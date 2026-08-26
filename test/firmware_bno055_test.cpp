#include <assert.h>
#include <stdio.h>

#include "bno055_sensor.h"
#include "config.h"
#include <Wire.h>

HardwareSerial Serial;
TwoWire Wire;

namespace {

namespace cfg = safestride_terrain_config;
uint32_t g_now_ms = 0UL;
uint8_t g_address = 0U;
uint8_t g_register = 0U;
uint8_t g_write_index = 0U;
uint8_t g_read_index = 0U;
uint8_t g_quantity = 0U;
bool g_fail_reads = false;

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
void TwoWire::beginTransmission(uint8_t address) {
  g_address = address;
  g_write_index = 0U;
}
size_t TwoWire::write(uint8_t value) {
  if (g_write_index++ == 0U) {
    g_register = value;
  }
  return 1U;
}
uint8_t TwoWire::endTransmission() { return g_fail_reads ? 1U : 0U; }
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  g_address = address;
  g_read_index = 0U;
  g_quantity = g_fail_reads ? 0U : quantity;
  return g_quantity;
}
int TwoWire::available() { return g_quantity - g_read_index; }
int TwoWire::read() {
  const uint8_t index = g_read_index++;
  if (g_address != cfg::BNO055_ADDRESS_LOW) {
    return 0;
  }
  if (g_register == 0x00U) {
    return 0xA0;
  }
  if (g_register == 0x1AU) {
    // heading=90 deg, roll=10 deg, pitch=-5 deg at 1/16 degree/LSB.
    const uint8_t euler[6U] = {0xA0U, 0x05U, 0xA0U, 0x00U, 0xB0U, 0xFFU};
    return euler[index];
  }
  if (g_register == 0x35U) {
    return 0xFF;
  }
  return 0;
}

int main() {
  Bno055Sensor bno;
  bno.begin(g_now_ms);
  bno.update(g_now_ms);
  assert(bno.valid());
  assert(bno.address() == cfg::BNO055_ADDRESS_LOW);
  assert(bno.headingMrad() >= 1570 && bno.headingMrad() <= 1572);
  assert(bno.rollMrad() >= 174 && bno.rollMrad() <= 176);
  assert(bno.pitchMrad() >= -88 && bno.pitchMrad() <= -86);
  assert(bno.calibration() == 0xFFU);

  g_fail_reads = true;
  for (uint8_t index = 0U; index < 3U; ++index) {
    g_now_ms += cfg::BNO055_SAMPLE_PERIOD_MS;
    bno.update(g_now_ms);
  }
  assert(!bno.valid());

  printf("firmware BNO055 tests: OK\n");
  return 0;
}
