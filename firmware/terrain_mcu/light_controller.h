#pragma once

#include <Arduino.h>

// Local, non-blocking dusk switch. No dependency on the Pi session or motor.
class LightController {
 public:
  struct Settings {
    uint8_t sensor_pin;
    int8_t relay_pin;       // -1: not assigned
    int8_t relay_on_level;  // -1: unknown, otherwise LOW or HIGH
    bool output_enabled;
    bool bright_is_high;
    uint16_t on_brightness;
    uint16_t off_brightness;
    uint16_t sample_ms;
    uint16_t confirm_ms;
  };

  explicit LightController(const Settings& settings) : settings_(settings) {}

  void begin(uint32_t now) {
    on_ = false;
    pending_ = false;
    raw_ = 0U;
    sampled_ = false;
    last_sample_ = now - settings_.sample_ms;
    pinMode(settings_.sensor_pin, INPUT);
    output_ready_ = settings_.output_enabled &&
        settings_.relay_pin >= 2 && settings_.relay_pin <= 13 &&
        settings_.relay_pin != 8 && settings_.relay_pin != 9 &&
        (settings_.relay_on_level == LOW || settings_.relay_on_level == HIGH);
    if (output_ready_) {
      // Preload OFF before enabling the output driver.
      writeRelay();
      pinMode(static_cast<uint8_t>(settings_.relay_pin), OUTPUT);
    }
  }

  void update(uint32_t now) {
    if (static_cast<uint32_t>(now - last_sample_) < settings_.sample_ms) return;
    last_sample_ = now;
    raw_ = static_cast<uint16_t>(analogRead(settings_.sensor_pin));
    sampled_ = true;
    const uint16_t brightness = settings_.bright_is_high ? raw_ : 1023U - raw_;
    const bool wants_change = on_ ? brightness >= settings_.off_brightness
                                  : brightness <= settings_.on_brightness;
    if (!wants_change) {
      pending_ = false;
      return;
    }
    if (!pending_) {
      pending_ = true;
      pending_since_ = now;
    }
    if (static_cast<uint32_t>(now - pending_since_) >= settings_.confirm_ms) {
      on_ = !on_;
      pending_ = false;
      if (output_ready_) writeRelay();
    }
  }

  uint16_t rawAdc() const { return raw_; }
  bool hasSample() const { return sampled_; }
  bool requestedOn() const { return on_; }
  bool outputReady() const { return output_ready_; }

 private:
  void writeRelay() {
    digitalWrite(static_cast<uint8_t>(settings_.relay_pin),
                 on_ ? settings_.relay_on_level :
                       (settings_.relay_on_level == HIGH ? LOW : HIGH));
  }

  Settings settings_;
  uint32_t last_sample_ = 0UL;
  uint32_t pending_since_ = 0UL;
  uint16_t raw_ = 0U;
  bool on_ = false;
  bool pending_ = false;
  bool sampled_ = false;
  bool output_ready_ = false;
};
