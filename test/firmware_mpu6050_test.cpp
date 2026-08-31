#include <assert.h>
#include <stdio.h>

#include "config.h"
#include "mpu6050_sensor.h"
#include <Wire.h>

HardwareSerial Serial;
TwoWire Wire;
namespace cfg = safestride_terrain_config;

namespace {
uint8_t g_address = 0U;
uint8_t g_register = 0U;
uint8_t g_index = 0U;
uint8_t g_quantity = 0U;
uint8_t g_write_count = 0U;
uint32_t g_now_ms = 0UL;
bool g_request_ok = true;
uint8_t g_register_values[256] = {0U};

uint8_t g_sample[14U] = {
    0x00U, 0x00U, 0x00U, 0x00U, 0x40U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x83U, 0xFEU, 0xFAU, 0x00U, 0x00U};
}

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
  g_write_count = 0U;
}
size_t TwoWire::write(uint8_t value) {
  if (g_write_count == 0U) {
    g_register = value;
  } else {
    g_register_values[g_register] = value;
  }
  ++g_write_count;
  return 1U;
}
uint8_t TwoWire::endTransmission() { return 0U; }
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  g_address = address;
  g_quantity = quantity;
  g_index = 0U;
  return g_request_ok && address == 0x68U ? quantity : 0U;
}
int TwoWire::available() { return g_quantity - g_index; }
int TwoWire::read() {
  const uint8_t index = g_index++;
  if (g_register == 0x75U) {
    return 0x68;
  }
  if (g_register == 0x3BU && index < sizeof(g_sample)) {
    return g_sample[index];
  }
  return 0;
}

int main() {
  Mpu6050Sensor mpu;
  mpu.begin(g_now_ms);
  mpu.update(g_now_ms);
  assert(mpu.address() == 0x68U);
  assert(mpu.valid());
  assert(mpu.accelXMg() == 0);
  assert(mpu.accelYMg() == 0);
  assert(mpu.accelZMg() == 1000);
  assert(mpu.gyroXMradS() >= 17 && mpu.gyroXMradS() <= 18);
  assert(mpu.gyroYMradS() <= -34 && mpu.gyroYMradS() >= -36);
  assert(mpu.rollMrad() == 0);
  assert(mpu.pitchMrad() == 0);
  assert(g_register_values[0x19U] ==
         cfg::MPU6050_SAMPLE_RATE_DIVIDER);

  // Three failed reads force a reconnect. The first sample after reconnect
  // must initialize attitude from the new pose instead of blending stale
  // pre-disconnect state.
  g_request_ok = false;
  for (uint8_t index = 0U;
       index < cfg::MPU6050_MAX_CONSECUTIVE_ERRORS;
       ++index) {
    g_now_ms += cfg::MPU6050_SAMPLE_PERIOD_MS;
    mpu.update(g_now_ms);
    assert(!mpu.valid());
  }
  g_sample[2U] = 0x40U;
  g_sample[3U] = 0x00U;
  g_sample[4U] = 0x00U;
  g_sample[5U] = 0x00U;
  g_request_ok = true;
  g_now_ms += cfg::MPU6050_RECONNECT_PERIOD_MS;
  mpu.update(g_now_ms);
  assert(!mpu.valid());
  g_now_ms += cfg::MPU6050_SAMPLE_PERIOD_MS;
  mpu.update(g_now_ms);
  assert(mpu.valid());
  assert(mpu.rollMrad() >= 1569 && mpu.rollMrad() <= 1572);
  printf("firmware MPU6050 tests: OK\n");
  return 0;
}
