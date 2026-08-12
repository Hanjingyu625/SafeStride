#pragma once

#include <Arduino.h>

class TwoWire {
 public:
  void begin();
  void beginTransmission(uint8_t address);
  size_t write(uint8_t value);
  uint8_t endTransmission();
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  int available();
  int read();
};

extern TwoWire Wire;
