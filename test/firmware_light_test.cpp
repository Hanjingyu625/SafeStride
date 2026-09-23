#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../firmware/terrain_mcu/config.h"
#include "../firmware/terrain_mcu/light_controller.h"

static int adc = 0;
static int reads = 0;
static int writes = 0;
static int output_modes = 0;
static int level = -1;
void pinMode(uint8_t pin, uint8_t mode) {
  if (mode == OUTPUT) {
    assert(pin == 4);
    assert(writes > 0);  // OFF was preloaded before OUTPUT.
    ++output_modes;
  } else {
    assert(pin == A0 && mode == INPUT);
  }
}
void digitalWrite(uint8_t pin, uint8_t value) {
  assert(pin == 4);
  level = value;
  ++writes;
}
int analogRead(uint8_t pin) { assert(pin == A0); ++reads; return adc; }

LightController::Settings settings(bool enabled, int8_t polarity, bool bright_high) {
  return {A0, 4, polarity, enabled, bright_high, 350, 550, 50, 1000};
}

void exercisePolarity(int8_t polarity, bool bright_high) {
  LightController light(settings(true, polarity, bright_high));
  writes = output_modes = reads = 0;
  light.begin(0);
  assert(light.outputReady() && !light.requestedOn() && !light.hasSample());
  assert(writes == 1 && output_modes == 1 && level != polarity);
  adc = bright_high ? 350 : 673;
  light.update(0);
  light.update(49);
  assert(reads == 1 && light.rawAdc() == adc && light.hasSample());
  light.update(950);
  assert(!light.requestedOn());
  light.update(1000);
  assert(light.requestedOn() && level == polarity);
  adc = bright_high ? 450 : 573;
  light.update(2000);
  assert(light.requestedOn());  // dead band retains state
  adc = bright_high ? 550 : 473;
  light.update(2050);
  adc = bright_high ? 500 : 523;
  light.update(2100);  // transient bright sample must not switch off
  adc = bright_high ? 550 : 473;
  light.update(2150);
  light.update(3100);
  assert(light.requestedOn());
  light.update(3150);
  assert(!light.requestedOn() && level != polarity && writes == 3);
}

int main() {
  exercisePolarity(LOW, true);
  exercisePolarity(HIGH, true);
  exercisePolarity(LOW, false);
  exercisePolarity(HIGH, false);

  namespace cfg = safestride_terrain_config;
  LightController disabled({cfg::LIGHT_SENSOR_PIN, cfg::LIGHT_RELAY_PIN,
      cfg::LIGHT_RELAY_ON_LEVEL, cfg::LIGHT_OUTPUT_ENABLED,
      cfg::LIGHT_BRIGHT_IS_HIGH, cfg::LIGHT_ON_BRIGHTNESS,
      cfg::LIGHT_OFF_BRIGHTNESS, cfg::LIGHT_SAMPLE_MS, cfg::LIGHT_CONFIRM_MS});
  writes = output_modes = 0;
  disabled.begin(0);
  adc = 0;
  disabled.update(0);
  disabled.update(1000);
  assert(disabled.requestedOn() && !disabled.outputReady());
  assert(writes == 0 && output_modes == 0);

  // Runtime guard also refuses unknown polarity or occupied UART pins.
  auto invalid = settings(true, -1, true);
  LightController unknown(invalid);
  unknown.begin(0);
  invalid = settings(true, HIGH, true);
  invalid.relay_pin = 8;
  LightController occupied(invalid);
  occupied.begin(0);
  assert(!unknown.outputReady() && !occupied.outputReady() && writes == 0);

  LightController wrap(settings(true, LOW, true));
  wrap.begin(UINT32_MAX - 499U);
  wrap.update(UINT32_MAX - 499U);
  wrap.update(450U);
  assert(!wrap.requestedOn());
  wrap.update(500U);
  assert(wrap.requestedOn());
  wrap.begin(600U);
  assert(!wrap.requestedOn() && level == HIGH);
  puts("lighting tests passed");
}
