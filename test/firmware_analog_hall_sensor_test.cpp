#include <assert.h>
#include <stdio.h>

#include "../firmware/safestride_mcu/analog_hall_sensor.h"
#include "../firmware/safestride_mcu/motor_control.h"
#include "../firmware/safestride_mcu/config.h"

namespace {

uint16_t g_hall_adc = 512U;

}  // namespace

HardwareSerial Serial;

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return LOW; }
void analogWrite(uint8_t, int) {}
int analogRead(uint8_t pin) {
  assert(pin == safestride_config::HALL_ANALOG_PIN);
  return g_hall_adc;
}
int digitalPinToInterrupt(uint8_t) { return NOT_AN_INTERRUPT; }
void attachInterrupt(int, void (*)(), int) {}
void noInterrupts() {}
void interrupts() {}
uint32_t millis() { return 0UL; }
uint32_t micros() { return 0UL; }
void delayMicroseconds(unsigned int) {}

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
  namespace cfg = safestride_config;

  AnalogHallSensor hall;
  hall.begin(0UL);
  assert(hall.baselineAdc() == 512U);
  assert(hall.pulseCount() == 0UL);

  uint32_t now_us = cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  assert(hall.pulseCount() == 0UL);

  g_hall_adc = 650U;
  now_us += cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  assert(hall.pulseCount() == 1UL);
  assert(hall.magnetPresent());

  for (uint8_t sample = 0U; sample < 1U; ++sample) {
    g_hall_adc = sample % 2U == 0U ? 620U : 700U;
    now_us += cfg::HALL_SAMPLE_PERIOD_US;
    hall.update(now_us);
  }
  assert(hall.pulseCount() == 1UL);

  g_hall_adc = 520U;
  now_us += cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  assert(!hall.magnetPresent());

  // A threshold retrigger shortly after the first magnet is analogue chatter,
  // not a second wheel magnet. It must not change count or measured period.
  g_hall_adc = 350U;
  now_us += cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  assert(hall.pulseCount() == 1UL);
  assert(hall.periodUs() == 0UL);

  g_hall_adc = 512U;
  now_us += cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  assert(!hall.magnetPresent());

  g_hall_adc = 350U;
  now_us += cfg::HALL_MIN_PULSE_INTERVAL_US;
  hall.update(now_us);
  assert(hall.pulseCount() == 2UL);
  assert(hall.periodUs() > 0UL);
  assert(hall.ageUs(now_us) == 0UL);

  g_hall_adc = 512U;
  now_us += cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  g_hall_adc = 535U;
  now_us += cfg::HALL_SAMPLE_PERIOD_US;
  hall.update(now_us);
  assert(hall.pulseCount() == 2UL);

  // Actual ADC -> pulse detector -> drive path: 20 ms pulses must not be
  // discarded as they were with the former 250 ms blanking window.
  g_hall_adc = 512U;
  AnalogHallSensor fast_hall;
  fast_hall.begin(0UL);
  DriveController drive;
  drive.begin();
  for (uint32_t t=5000UL;t<=1000000UL;t+=5000UL) {
    g_hall_adc = (t % 20000UL == 0UL) ? 650U : 512U;
    fast_hall.update(t);
    HallSample sample={fast_hall.pulseCount(),fast_hall.periodUs(),fast_hall.ageUs(t)};
    drive.update(5000UL,sample,sample,696L,true);
  }
  assert(fast_hall.pulseCount()==50UL);
  assert(fast_hall.periodUs()==20000UL);
  assert(drive.speedValid());
  assert(drive.braking() || drive.hallFaultMask()!=0U);
  printf("analogue WSH135 Hall and overspeed input-path tests: OK\n");
  return 0;
}
