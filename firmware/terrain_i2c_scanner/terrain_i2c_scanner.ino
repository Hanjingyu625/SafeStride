#include <Arduino.h>
#include <Wire.h>

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t SCAN_PERIOD_MS = 1000U;

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  delay(100U);
}

void loop() {
  uint8_t found = 0U;
  Serial.print(F("I2C:"));
  for (uint8_t address = 1U; address < 127U; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0U) {
      Serial.print(F(" 0x"));
      if (address < 0x10U) {
        Serial.print('0');
      }
      Serial.print(address, HEX);
      ++found;
    }
  }
  if (found == 0U) {
    Serial.print(F(" none"));
  }
  Serial.println();
  delay(SCAN_PERIOD_MS);
}
